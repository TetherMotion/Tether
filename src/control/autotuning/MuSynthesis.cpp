/**
 * @file MuSynthesis.cpp
 * @brief Implementation of μ-Synthesis (Mu Synthesis) Robust Controller Design
 * 
 * This file implements:
 * - MuSynthesisController: D-K iteration for structured uncertainty
 * - MuAnalysis: μ computation utilities
 * - StructuredUncertainModel: LFT model building
 */

#include "tether/control/autotuning/MuSynthesis.hpp"
#include <cmath>
#include <algorithm>
#include <cstring>
#include <numeric>

#include <Eigen/Dense>

namespace tether::control {

// ============================================================================
// MuSynthesisController Implementation
// ============================================================================

void MuSynthesisController::setNominalPlant(const double* A, const double* B,
                                            const double* C, const double* D,
                                            int n, int m, int p) {
    m_n = n;
    m_m = m;
    m_p = p;
    
    for (int i = 0; i < n * n; i++) m_A[i] = A[i];
    for (int i = 0; i < n * m; i++) m_B[i] = B[i];
    for (int i = 0; i < p * n; i++) m_C[i] = C[i];
    for (int i = 0; i < p * m; i++) m_D[i] = D[i];
    
    m_synthesized = false;
}

void MuSynthesisController::addUncertaintyBlock(const std::string& name, int size,
                                                 MuBlockType type, double bound) {
    UncertaintyBlock block;
    block.name = name;
    block.type = type;
    block.rows = size;
    block.cols = size;
    block.repetitions = 1;
    block.bound = bound;
    
    m_uncertaintyBlocks.push_back(block);
    
    m_totalDeltaRows += block.totalRows();
    m_totalDeltaCols += block.totalCols();
    m_synthesized = false;
}

void MuSynthesisController::addRepeatedScalar(const std::string& name, int repetitions,
                                               bool isReal, double bound) {
    UncertaintyBlock block;
    block.name = name;
    block.type = isReal ? MuBlockType::RepeatedReal : MuBlockType::RepeatedScalar;
    block.rows = 1;
    block.cols = 1;
    block.repetitions = repetitions;
    block.bound = bound;
    
    m_uncertaintyBlocks.push_back(block);
    
    m_totalDeltaRows += block.totalRows();
    m_totalDeltaCols += block.totalCols();
    m_synthesized = false;
}

void MuSynthesisController::clearUncertaintyBlocks() {
    m_uncertaintyBlocks.clear();
    m_totalDeltaRows = 0;
    m_totalDeltaCols = 0;
    m_synthesized = false;
}

void MuSynthesisController::setInterconnection(const double* P11, const double* P12,
                                                const double* P21, const double* P22,
                                                int nz, int ne, int nw, int nu) {
    if (!P11 || !P12 || !P21 || !P22 || nz <= 0 || ne <= 0 || nw <= 0 || nu <= 0) {
        m_hasInterconnection = false;
        m_P11.clear(); m_P12.clear(); m_P21.clear(); m_P22.clear();
        return;
    }
    m_P11.assign(P11, P11 + static_cast<size_t>(nz) * nw);
    m_P12.assign(P12, P12 + static_cast<size_t>(nz) * nu);
    m_P21.assign(P21, P21 + static_cast<size_t>(ne) * nw);
    m_P22.assign(P22, P22 + static_cast<size_t>(ne) * nu);
    m_nzI = nz; m_neI = ne; m_nwI = nw; m_nuI = nu;
    m_hasInterconnection = true;
    m_synthesized = false;
}

void MuSynthesisController::setSensitivityWeight(double M, double omegaB, double epsilon) {
    // W1(s) = (s/M + ωB) / (s + ωB·ε)
    m_W1.num[0] = 0.0;
    m_W1.num[1] = 1.0 / M;
    m_W1.num[2] = omegaB;
    m_W1.den[0] = 0.0;
    m_W1.den[1] = 1.0;
    m_W1.den[2] = omegaB * epsilon;
    m_W1.order = 1;
    m_hasW1 = true;
    m_w1M = M; m_w1wB = omegaB; m_w1eps = epsilon;
}

void MuSynthesisController::setControlWeight(double maxControl) {
    // W2(s) = 1/maxControl (static)
    m_W2.num[0] = 0.0;
    m_W2.num[1] = 0.0;
    m_W2.num[2] = 1.0 / maxControl;
    m_W2.den[0] = 0.0;
    m_W2.den[1] = 0.0;
    m_W2.den[2] = 1.0;
    m_W2.order = 0;
    m_hasW2 = true;
    m_w2maxU = maxControl;
}

void MuSynthesisController::setComplementarySensitivityWeight(double M, double omegaT, 
                                                               double epsilon) {
    // W3(s) = (s + ωT/M) / (ε·s + ωT)
    m_W3.num[0] = 0.0;
    m_W3.num[1] = 1.0;
    m_W3.num[2] = omegaT / M;
    m_W3.den[0] = 0.0;
    m_W3.den[1] = epsilon;
    m_W3.den[2] = omegaT;
    m_W3.order = 1;
    m_hasW3 = true;
    m_w3M = M; m_w3wT = omegaT; m_w3eps = epsilon;
}

namespace {

using Cmat = Eigen::Matrix<std::complex<double>, Eigen::Dynamic, Eigen::Dynamic>;

// Map the autotuning UncertaintyBlock list onto the block-structure
// description used by MuSynthesisFramework. Returns false when the blocks
// do not form a valid tiling (non-square or size mismatch).
bool structureFromBlocks(const std::vector<UncertaintyBlock>& blocks,
                         int* blockSizes, bool* repeated, int& numBlocks,
                         int& totalDim) {
    numBlocks = 0;
    totalDim = 0;
    for (const auto& b : blocks) {
        if (numBlocks >= 10) return false;
        const int dim = b.totalRows();
        if (dim <= 0 || dim != b.totalCols()) return false;
        blockSizes[numBlocks] = dim;
        repeated[numBlocks] = (b.type == MuBlockType::RepeatedScalar ||
                               b.type == MuBlockType::RepeatedReal ||
                               b.type == MuBlockType::Scalar);
        numBlocks++;
        totalDim += dim;
    }
    return numBlocks > 0 && totalDim > 0;
}

// Rigorous upper bound: min over block-structured positive D of
// sigma_max(D M D^-1), via MuSynthesisFramework. Falls back to the
// unstructured spectral norm when the structure is unset or does not
// tile n (still a valid, looser upper bound).
double muUpperBound(const std::complex<double>* M, int n,
                    const std::vector<UncertaintyBlock>& blocks) {
    if (n <= 0 || !M) return 0.0;
    MuSynthesisFramework fw;
    int bs[10]; bool rep[10]; int nb = 0, dim = 0;
    if (structureFromBlocks(blocks, bs, rep, nb, dim) && dim == n) {
        fw.setUncertaintyStructure(bs, nb, rep);
    }
    return fw.computeMuUpperBound(M, n);
}

// Lower-bound estimate. For a single full complex block mu = sigma_max
// exactly; otherwise the spectral radius of M is used as a heuristic
// estimate (exact for a lone repeated-scalar block delta*I, and a
// reasonable proxy for mixed structures). The result is clamped to the
// rigorous upper bound so upper >= lower always holds.
double muLowerBound(const std::complex<double>* M, int n,
                    const std::vector<UncertaintyBlock>& blocks) {
    if (n <= 0 || !M) return 0.0;
    const double upper = muUpperBound(M, n, blocks);
    if (n == 1) return std::min(upper, std::abs(M[0]));
    Eigen::Map<const Cmat> mat(M, n, n);
    const auto eig = mat.eigenvalues();
    double rho = 0.0;
    for (int i = 0; i < eig.size(); ++i) rho = std::max(rho, std::abs(eig(i)));
    // A single unstructured (full) block has mu = sigma_max exactly.
    bool singleFull = (blocks.size() == 1 && blocks[0].totalRows() == n &&
                       blocks[0].type == MuBlockType::Full);
    if (singleFull) return upper;
    return std::min(upper, rho);
}

} // namespace

bool MuSynthesisController::synthesize(const SynthesisConfig& config) {
    if (m_n <= 0 || m_n > static_cast<int>(MAX_STATE_DIM) ||
        m_m <= 0 || m_m > static_cast<int>(MAX_CONTROL_DIM) ||
        m_p <= 0 || m_p > static_cast<int>(MAX_OUTPUT_DIM)) {
        return false;
    }
    m_iterations = 0;
    m_synthesized = false;
    m_hasGen = false;

    const int d = m_totalDeltaRows;
    if (m_totalDeltaCols != d || d < 0 || d > static_cast<int>(MAX_OUTPUT_DIM)) {
        return false;
    }

    // ------------------------------------------------------------------
    // 1) Build the weight-augmented generalized plant. HInfinityController
    //    assembles the mixed-sensitivity plant (W1/W2/W3 dynamics appended)
    //    with D11 = D22 = 0; isAchievable() populates it without needing a
    //    successful Riccati solve.
    // ------------------------------------------------------------------
    HInfinityController hinf;
    hinf.setPlant(m_A.data(), m_B.data(), m_C.data(), m_D.data(),
                  m_n, m_m, m_p);
    if (m_hasW1) hinf.setSensitivityWeight(m_w1M, m_w1wB, m_w1eps);
    if (m_hasW2) hinf.setControlWeight(m_w2maxU);
    if (m_hasW3) hinf.setComplementaryWeight(m_w3M, m_w3wT, m_w3eps);
    hinf.isAchievable(1.0);

    int na = 0, nw = 0, nu = 0, nz = 0, ny = 0;
    hinf.getAugmentedDimensions(na, nw, nu, nz, ny);
    if (na <= 0 || nw <= 0 || nu <= 0 || nz <= 0 || ny <= 0) {
        return false;  // e.g. plant has nonzero D feedthrough
    }

    std::vector<double> Pa(static_cast<size_t>(na) * na),
                        B1(static_cast<size_t>(na) * nw),
                        B2(static_cast<size_t>(na) * nu),
                        C1(static_cast<size_t>(nz) * na),
                        C2(static_cast<size_t>(ny) * na),
                        D11(static_cast<size_t>(nz) * nw, 0.0),
                        D12(static_cast<size_t>(nz) * nu, 0.0),
                        D21(static_cast<size_t>(ny) * nw, 0.0),
                        D22(static_cast<size_t>(ny) * nu, 0.0);
    hinf.getAugmentedPlant(Pa.data(), B1.data(), B2.data(), C1.data(), C2.data(),
                           D11.data(), D12.data(), D21.data(), D22.data());

    // ------------------------------------------------------------------
    // 2) No uncertainty blocks: plain weighted H-infinity design.
    // ------------------------------------------------------------------
    if (d == 0) {
        const double gamma = hinf.designOptimal();
        if (!(gamma > 0.0) || na > MAX_CONTROLLER_DIM) return false;
        std::vector<double> Ak(static_cast<size_t>(na) * na),
                            Bk(static_cast<size_t>(na) * ny),
                            Ck(static_cast<size_t>(nu) * na),
                            Dk(static_cast<size_t>(nu) * ny);
        hinf.getControllerMatrices(Ak.data(), Bk.data(), Ck.data(), Dk.data());
        m_nk = na;
        std::memcpy(m_Ak.data(), Ak.data(), Ak.size() * sizeof(double));
        for (int i = 0; i < na * ny; ++i) m_Bk[i] = Bk[i];
        for (int i = 0; i < nu * na; ++i) m_Ck[i] = Ck[i];
        for (int i = 0; i < nu * ny; ++i) m_Dk[i] = Dk[i];
        m_muResult = analyzeMu(config.freqMin, config.freqMax,
                               config.numFrequencies);
        m_iterations = 1;
        m_synthesized = true;
        return true;
    }

    // ------------------------------------------------------------------
    // 3) Extend the generalized plant with trailing Delta channels.
    //    w' = [w_a ; w_delta] (nw + d),  z' = [z_a ; z_delta] (nz + d).
    // ------------------------------------------------------------------
    const int nwE = nw + d, nzE = nz + d;
    // MuSynthesisFramework limits: n,nw <= 24; nu,ny,nz <= 6.
    if (na > 24 || nwE > 24 || nzE > 6 || nu > 6 || ny > 6) return false;

    // Choose Delta placement: explicit interconnection > input > output.
    int place = 0;
    if (m_hasInterconnection) {
        const bool dimsOk = (m_nzI == d && m_nwI == d && m_neI == ny &&
                             m_nuI == nu);
        bool p22Zero = true;
        for (double v : m_P22) if (std::abs(v) > 1e-9) { p22Zero = false; break; }
        if (!dimsOk || !p22Zero) return false;
        place = 3;
    } else if (d == m_m) {
        place = 1;   // input multiplicative: z_delta = u, w_delta enters B
    } else if (d == m_p) {
        place = 2;   // output multiplicative: z_delta = y_p, w_delta leaves y
    } else {
        return false;  // Delta dimension matches neither plant input nor output
    }

    m_gn = na; m_gnw = nwE; m_gnu = nu; m_gnz = nzE; m_gny = ny;
    m_gUncW = d; m_gUncZ = d; m_deltaPlace = place;
    m_gA = Pa;
    m_gB2 = B2;
    m_gC2 = C2;
    m_gD22 = D22;

    // B1' = [B1 | B1_delta]
    m_gB1.assign(static_cast<size_t>(na) * nwE, 0.0);
    for (int i = 0; i < na * nw; ++i) m_gB1[i] = B1[i];
    // C1' = [C1 ; C1_delta]
    m_gC1.assign(static_cast<size_t>(nzE) * na, 0.0);
    for (int i = 0; i < nz * na; ++i) m_gC1[i] = C1[i];
    // D11' = [D11 0 ; D11_dw D11_dd]
    m_gD11.assign(static_cast<size_t>(nzE) * nwE, 0.0);
    for (int i = 0; i < nz * nw; ++i) m_gD11[i] = D11[i];
    // D12' = [D12 ; D12_delta]
    m_gD12.assign(static_cast<size_t>(nzE) * nu, 0.0);
    for (int i = 0; i < nz * nu; ++i) m_gD12[i] = D12[i];
    // D21' = [D21 | D21_delta]
    m_gD21.assign(static_cast<size_t>(ny) * nwE, 0.0);
    for (int i = 0; i < ny * nw; ++i) m_gD21[i] = D21[i];

    auto B1d = [&](int i, int j) -> double& { return m_gB1[i * nwE + nw + j]; };
    auto C1d = [&](int i, int j) -> double& { return m_gC1[(nz + i) * na + j]; };
    auto D11dd = [&](int i, int j) -> double& { return m_gD11[(nz + i) * nwE + nw + j]; };
    auto D12d = [&](int i, int j) -> double& { return m_gD12[(nz + i) * nu + j]; };
    auto D21d = [&](int i, int j) -> double& { return m_gD21[i * nwE + nw + j]; };

    if (place == 1) {
        // Input multiplicative Delta: u_plant = u + w_delta, z_delta = u.
        // w_delta enters the plant input exactly like u does (B column),
        // but only through the plant states - weight states see the
        // commanded u, not the perturbed plant input.
        for (int j = 0; j < d; ++j)
            for (int i = 0; i < m_n; ++i)
                B1d(i, j) = B2[i * nu + j];
        for (int j = 0; j < d; ++j) D12d(j, j) = 1.0;   // z_delta = u
    } else if (place == 2) {
        // Output multiplicative Delta: y = y_p + w_delta, z_delta = y_p.
        // w_delta subtracts from the tracking error e = r - y_p - w_delta,
        // so wherever the reference columns of B1 drive the W1 states,
        // w_delta drives them with the opposite sign.
        for (int j = 0; j < d; ++j)
            for (int i = 0; i < na; ++i)
                B1d(i, j) = -B1[i * nw + j];   // r occupies w columns 0..p-1
        for (int j = 0; j < d; ++j)
            for (int i = 0; i < m_n; ++i)
                C1d(j, i) = m_C[j * m_n + i];  // z_delta = C x_plant
        for (int i = 0; i < ny; ++i)
            for (int j = 0; j < d; ++j)
                D21d(i, j) = (i == j) ? -1.0 : 0.0;  // y_meas -= w_delta
    } else {
        // Custom static interconnection:
        //   z_delta = P11 w_delta + P12 u      (no state dependence)
        //   y_meas += P21 w_delta              (P22 verified ~0 above)
        for (int i = 0; i < d; ++i)
            for (int j = 0; j < d; ++j)
                D11dd(i, j) = m_P11[i * m_nwI + j];
        for (int i = 0; i < d; ++i)
            for (int j = 0; j < nu; ++j)
                D12d(i, j) = m_P12[i * m_nuI + j];
        for (int i = 0; i < ny; ++i)
            for (int j = 0; j < d; ++j)
                D21d(i, j) = m_P21[i * m_nwI + j];
    }

    // ------------------------------------------------------------------
    // 4) Real D-K iteration via MuSynthesisFramework.
    // ------------------------------------------------------------------
    int bs[10]; bool rep[10]; int nb = 0, dsum = 0;
    for (const auto& b : m_uncertaintyBlocks) {
        if (nb >= 10) return false;
        const int dim = b.totalRows();
        if (dim <= 0 || dim != b.totalCols()) return false;
        bs[nb] = dim;
        rep[nb] = (b.type == MuBlockType::RepeatedScalar ||
                   b.type == MuBlockType::RepeatedReal);
        nb++;
        dsum += dim;
    }
    if (nb == 0 || dsum != d) return false;

    MuSynthesisFramework fw;
    fw.setUncertaintyStructure(bs, nb, rep);
    fw.setGeneralizedPlant(m_gA.data(), m_gB1.data(), m_gB2.data(),
                           m_gC1.data(), m_gC2.data(),
                           m_gD11.data(), m_gD12.data(),
                           m_gD21.data(), m_gD22.data(),
                           na, nwE, nu, nzE, ny, d, d);
    fw.setFrequencyGrid(config.freqMin, config.freqMax,
                        std::min(config.numFrequencies, 64));
    const int iters = fw.dkIterate(config.maxIterations,
                                   config.convergenceTol);
    m_iterations = iters;
    if (!fw.hasController()) return false;

    int nk = 0;
    std::vector<double> Ak(static_cast<size_t>(24) * 24),
                        Bk(static_cast<size_t>(24) * 6),
                        Ck(static_cast<size_t>(6) * 24),
                        Dk(static_cast<size_t>(6) * 6);
    fw.getControllerMatrices(Ak.data(), Bk.data(), Ck.data(), Dk.data(), nk);
    if (nk <= 0 || nk > MAX_CONTROLLER_DIM) return false;
    m_nk = nk;
    std::memcpy(m_Ak.data(), Ak.data(), static_cast<size_t>(nk) * nk * sizeof(double));
    for (int i = 0; i < nk * ny; ++i) m_Bk[i] = Bk[i];
    for (int i = 0; i < nu * nk; ++i) m_Ck[i] = Ck[i];
    for (int i = 0; i < nu * ny; ++i) m_Dk[i] = Dk[i];

    m_muResult = analyzeMu(config.freqMin, config.freqMax,
                           config.numFrequencies);
    m_synthesized = true;
    return true;
}

MuAnalysisResult MuSynthesisController::analyzeMu(double freqMin, double freqMax,
                                                   int numPoints) {
    MuAnalysisResult result;
    if (numPoints < 2) numPoints = 2;
    if (!(freqMin > 0.0) || !(freqMax > freqMin)) {
        freqMin = 1e-3; freqMax = 1e3;
    }

    const int d = (m_hasGen && m_deltaPlace != 0) ? m_gUncZ : 0;
    const bool haveMu = (d > 0 && m_nk > 0);
    std::vector<std::complex<double>> Mbuf(static_cast<size_t>(d) * d);

    const double logMin = std::log10(freqMin);
    const double logMax = std::log10(freqMax);

    result.peakMuUpper = 0.0;
    result.peakMuLower = 0.0;

    for (int i = 0; i < numPoints; i++) {
        const double logFreq = logMin + i * (logMax - logMin) / (numPoints - 1);
        const double omega = std::pow(10.0, logFreq);

        result.frequencies.push_back(omega);

        double muUpper, muLower;
        if (haveMu) {
            evalMuMatrix(omega, Mbuf.data());
            muUpper = computeMuUpperBound(Mbuf.data(), d);
            muLower = computeMuLowerBound(Mbuf.data(), d);
        } else {
            // No Delta channel: report the unstructured bound on the
            // weighted complementary sensitivity T(jw) = GK(I+GK)^-1.
            const double sv = evalLoopNorm(omega);
            muUpper = sv;
            muLower = sv;
        }

        result.muUpper.push_back(muUpper);
        result.muLower.push_back(muLower);

        if (muUpper > result.peakMuUpper) {
            result.peakMuUpper = muUpper;
            result.peakFrequency = omega;
        }
        if (muLower > result.peakMuLower) {
            result.peakMuLower = muLower;
        }
    }

    return result;
}

double MuSynthesisController::computeMuUpperBound(const std::complex<double>* M,
                                                   int size) const {
    return muUpperBound(M, size, m_uncertaintyBlocks);
}

double MuSynthesisController::computeMuLowerBound(const std::complex<double>* M,
                                                   int size) const {
    return muLowerBound(M, size, m_uncertaintyBlocks);
}

void MuSynthesisController::getControllerMatrices(double* Ak, double* Bk,
                                                   double* Ck, double* Dk) const {
    for (int i = 0; i < m_nk * m_nk; i++) Ak[i] = m_Ak[i];
    for (int i = 0; i < m_nk * m_p; i++) Bk[i] = m_Bk[i];
    for (int i = 0; i < m_m * m_nk; i++) Ck[i] = m_Ck[i];
    for (int i = 0; i < m_m * m_p; i++) Dk[i] = m_Dk[i];
}

void MuSynthesisController::evalMuMatrix(double omega,
                                          std::complex<double>* M) const {
    const int d = m_gUncZ;
    if (M) std::fill(M, M + static_cast<size_t>(d) * d,
                     std::complex<double>(0.0, 0.0));
    if (!m_hasGen || m_deltaPlace == 0 || m_nk <= 0 || d <= 0 || !M) return;

    using RMat = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                               Eigen::RowMajor>;
    const int n = m_gn, nw = m_gnw, nu = m_gnu, nz = m_gnz, ny = m_gny, nk = m_nk;
    const int nx = n + nk;

    const auto Aa = Eigen::Map<const RMat>(m_gA.data(), n, n);
    const auto B1 = Eigen::Map<const RMat>(m_gB1.data(), n, nw);
    const auto B2 = Eigen::Map<const RMat>(m_gB2.data(), n, nu);
    const auto C1 = Eigen::Map<const RMat>(m_gC1.data(), nz, n);
    const auto C2 = Eigen::Map<const RMat>(m_gC2.data(), ny, n);
    const auto D11 = Eigen::Map<const RMat>(m_gD11.data(), nz, nw);
    const auto D12 = Eigen::Map<const RMat>(m_gD12.data(), nz, nu);
    const auto D21 = Eigen::Map<const RMat>(m_gD21.data(), ny, nw);
    const auto Ak = Eigen::Map<const RMat>(m_Ak.data(), nk, nk);
    const auto Bk = Eigen::Map<const RMat>(m_Bk.data(), nk, ny);
    const auto Ck = Eigen::Map<const RMat>(m_Ck.data(), nu, nk);
    const auto Dk = Eigen::Map<const RMat>(m_Dk.data(), nu, ny);

    // Closed loop with D22 = 0 (enforced at construction):
    //   u = Ck xk + Dk(C2 x + D21 w)
    //   x'  = (Aa + B2 Dk C2) x + B2 Ck xk + (B1 + B2 Dk D21) w
    //   xk' = Bk C2 x + Ak xk + Bk D21 w
    const Cmat AaC = Aa.cast<std::complex<double>>();
    const Cmat B2C = B2.cast<std::complex<double>>();
    const Cmat C2C = C2.cast<std::complex<double>>();
    const Cmat DkC = Dk.cast<std::complex<double>>();
    const Cmat D21C = D21.cast<std::complex<double>>();
    const Cmat B1C = B1.cast<std::complex<double>>();
    const Cmat C1C = C1.cast<std::complex<double>>();
    const Cmat D11C = D11.cast<std::complex<double>>();
    const Cmat D12C = D12.cast<std::complex<double>>();
    const Cmat AkC = Ak.cast<std::complex<double>>();
    const Cmat BkC = Bk.cast<std::complex<double>>();
    const Cmat CkC = Ck.cast<std::complex<double>>();

    Cmat Acl(nx, nx), Bcl(nx, d), Cm(d, nx), Dm(d, d);
    Acl.topLeftCorner(n, n) = AaC + B2C * DkC * C2C;
    Acl.topRightCorner(n, nk) = B2C * CkC;
    Acl.bottomLeftCorner(nk, n) = BkC * C2C;
    Acl.bottomRightCorner(nk, nk) = AkC;

    // Only the trailing w_delta columns of the exogenous input are needed.
    const Cmat B1d = B1C.rightCols(d);
    const Cmat D21d = D21C.rightCols(d);
    Bcl.topRows(n) = B1d + B2C * DkC * D21d;
    Bcl.bottomRows(nk) = BkC * D21d;

    const Cmat Cz_l = C1C + D12C * DkC * C2C;   // nz x n
    const Cmat Cz_r = D12C * CkC;               // nz x nk
    Cm.leftCols(n) = Cz_l.bottomRows(d);
    Cm.rightCols(nk) = Cz_r.bottomRows(d);

    const Cmat Dz = D11C + D12C * DkC * D21C;   // nz x nw
    Dm = Dz.bottomRightCorner(d, d);

    const std::complex<double> jw(0.0, omega);
    const Cmat Mcl = Cm * (jw * Cmat::Identity(nx, nx) - Acl).partialPivLu().solve(Bcl) + Dm;

    for (int i = 0; i < d; ++i)
        for (int j = 0; j < d; ++j)
            M[i * d + j] = Mcl(i, j);
}

// Spectral norm of the weighted complementary sensitivity T(jw) used as
// the unstructured fallback when no Delta channel is defined.
double MuSynthesisController::evalLoopNorm(double omega) const {
    if (m_n <= 0 || m_nk <= 0) return 0.0;
    using RMat = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                               Eigen::RowMajor>;
    const int n = m_n, m = m_m, p = m_p, nk = m_nk;
    const auto A = Eigen::Map<const RMat>(m_A.data(), n, n);
    const auto B = Eigen::Map<const RMat>(m_B.data(), n, m);
    const auto C = Eigen::Map<const RMat>(m_C.data(), p, n);
    const auto D = Eigen::Map<const RMat>(m_D.data(), p, m);
    const auto Ak = Eigen::Map<const RMat>(m_Ak.data(), nk, nk);
    const auto Bk = Eigen::Map<const RMat>(m_Bk.data(), nk, p);
    const auto Ck = Eigen::Map<const RMat>(m_Ck.data(), m, nk);
    const auto Dk = Eigen::Map<const RMat>(m_Dk.data(), m, p);

    const std::complex<double> jw(0.0, omega);
    const Cmat I_n = Cmat::Identity(n, n), I_k = Cmat::Identity(nk, nk);
    const Cmat G = C.cast<std::complex<double>>() *
        (jw * I_n - A.cast<std::complex<double>>()).partialPivLu().solve(
            B.cast<std::complex<double>>()) + D.cast<std::complex<double>>();
    const Cmat K = Ck.cast<std::complex<double>>() *
        (jw * I_k - Ak.cast<std::complex<double>>()).partialPivLu().solve(
            Bk.cast<std::complex<double>>()) + Dk.cast<std::complex<double>>();
    const Cmat L = G * K;                                  // p x p
    const Cmat T = L * (Cmat::Identity(p, p) + L).partialPivLu().solve(
        Cmat::Identity(p, p));
    return T.jacobiSvd().singularValues()(0);
}

ControllerOutput MuSynthesisController::computeImpl(const ControllerInput& input) {
    ControllerOutput output;

    if (!m_synthesized || m_nk == 0) {
        output.control = 0.0;
        return output;
    }

    // Runtime path is SISO (ControllerInput carries scalar reference and
    // measurement); for MIMO plants only the first measurement/control
    // channel is exercised.
    const double dt = input.dt;
    const double e = input.reference - input.measured;
    const int nk = m_nk, ny = m_p;

    // u = Ck xk + Dk e   (first control row / first measurement column)
    double u = m_Dk[0] * e;
    for (int j = 0; j < nk; ++j) u += m_Ck[j] * m_xk[j];

    // xk+ = xk + dt (Ak xk + Bk e)
    for (int i = 0; i < nk; ++i) {
        double s = m_Bk[i * ny] * e;
        for (int j = 0; j < nk; ++j) s += m_Ak[i * nk + j] * m_xk[j];
        m_xk[i] += dt * s;
    }

    output.control = u;
    return output;
}

void MuSynthesisController::resetImpl() {
    for (int i = 0; i < MAX_CONTROLLER_DIM; i++) {
        m_xk[i] = 0.0;
    }
}

// ============================================================================
// MuAnalysis Implementation
// ============================================================================

double MuAnalysis::upperBound(const std::complex<double>* M, int n,
                              const std::vector<UncertaintyBlock>& blocks) {
    return muUpperBound(M, n, blocks);
}

double MuAnalysis::lowerBound(const std::complex<double>* M, int n,
                              const std::vector<UncertaintyBlock>& blocks) {
    return muLowerBound(M, n, blocks);
}

std::vector<std::complex<double>> MuAnalysis::findOptimalDScale(
    const std::complex<double>* M, int n,
    const std::vector<UncertaintyBlock>& blocks) {

    // Minimize sigma_max(D M D^-1) over block-structured positive diagonal
    // D by coordinate descent on the log-scales. Positions inside a
    // non-repeated (full) block share one scale d*I_k; repeated-scalar
    // blocks get one scale per position (delta*I_k structure).
    std::vector<std::complex<double>> D(static_cast<size_t>(n), 1.0);
    if (n <= 0 || !M) return D;

    // Map each matrix position to its D-parameter index.
    std::vector<int> posParam(static_cast<size_t>(n), -1);
    int np = 0;
    int pos = 0;
    for (const auto& b : blocks) {
        const int dim = b.totalRows();
        if (dim <= 0 || pos + dim > n) break;
        const bool rep = (b.type == MuBlockType::RepeatedScalar ||
                          b.type == MuBlockType::RepeatedReal ||
                          b.type == MuBlockType::Scalar);
        if (rep) {
            for (int k = 0; k < dim; ++k) {
                posParam[static_cast<size_t>(pos + k)] = np++;
            }
        } else {
            for (int k = 0; k < dim; ++k) posParam[static_cast<size_t>(pos + k)] = np;
            ++np;
        }
        pos += dim;
    }
    if (np == 0 || pos != n) {
        // Structure does not tile n: unstructured per-position scaling.
        np = n;
        posParam.assign(static_cast<size_t>(n), 0);
        for (int i = 0; i < n; ++i) posParam[static_cast<size_t>(i)] = i;
    }

    std::vector<double> x(static_cast<size_t>(np), 0.0);   // log-scales
    Eigen::Map<const Cmat> Mc(M, n, n);

    auto bound = [&](const std::vector<double>& xv) {
        Cmat S = Cmat::Zero(n, n);
        for (int i = 0; i < n; ++i) S(i, i) = std::exp(xv[static_cast<size_t>(posParam[static_cast<size_t>(i)])]);
        Cmat Ms = S * Mc * S.inverse();
        return Ms.jacobiSvd().singularValues()(0);
    };

    double best = bound(x);
    const double step = 0.5;
    for (int sweep = 0; sweep < 60; ++sweep) {
        bool improved = false;
        for (int k = 0; k < np; ++k) {
            for (double delta : {step, -step}) {
                std::vector<double> xt = x;
                xt[static_cast<size_t>(k)] += delta;
                const double v = bound(xt);
                if (v < best - 1e-12) {
                    x = xt; best = v; improved = true;
                }
            }
        }
        if (!improved) break;
    }

    for (int i = 0; i < n; ++i) {
        const double s = std::exp(x[static_cast<size_t>(posParam[static_cast<size_t>(i)])]);
        D[static_cast<size_t>(i)] = std::complex<double>(s, 0.0);
    }
    return D;
}

std::pair<std::vector<double>, std::vector<double>> MuAnalysis::fitRational(
    const std::vector<double>& frequencies,
    const std::vector<std::complex<double>>& values,
    int order) {

    // Sanathanan-Koerner iteration: fit H(jw) ~ N(jw)/D(jw) with
    // N(s) = sum n_i s^(order-i) and monic D(s) = s^order + sum d_i s^(order-i).
    // Coefficients are returned in descending powers of s; den[0] = 1.
    if (order > 7) order = 7;   // pw[] scratch buffer depth
    if (order < 0) order = 0;
    std::vector<double> num(static_cast<size_t>(order) + 1, 0.0);
    std::vector<double> den(static_cast<size_t>(order) + 1, 0.0);
    den[0] = 1.0;
    if (frequencies.empty() || values.size() != frequencies.size()) {
        if (!values.empty()) num[0] = std::abs(values[values.size() / 2]);
        return {num, den};
    }

    const int nData = static_cast<int>(frequencies.size());
    const int nUnk = 2 * order + 1;
    // Unknown vector: [n_0..n_order, d_1..d_order]
    Eigen::MatrixXd A(static_cast<size_t>(2 * nData), static_cast<size_t>(nUnk));
    Eigen::VectorXd b(static_cast<size_t>(2 * nData));

    std::vector<double> dPrev(static_cast<size_t>(order) + 1, 0.0);
    dPrev[0] = 1.0;
    Eigen::VectorXd x(static_cast<size_t>(nUnk));
    x.setZero();

    for (int iter = 0; iter < 12; ++iter) {
        for (int k = 0; k < nData; ++k) {
            const double w = frequencies[static_cast<size_t>(k)];
            const std::complex<double> s(0.0, w);
            const std::complex<double> H = values[static_cast<size_t>(k)];
            // powers s^(order-i), i = 0..order
            std::complex<double> pw[8];
            pw[0] = std::pow(s, order);
            for (int i = 1; i <= order && i < 8; ++i) pw[i] = pw[i - 1] / s;
            // SK weight 1/|D_prev(jw)|
            std::complex<double> dp = 0.0;
            for (int i = 0; i <= order; ++i) dp += dPrev[static_cast<size_t>(i)] * pw[i];
            const double wgt = (std::abs(dp) > 1e-12) ? 1.0 / std::abs(dp) : 1.0;

            // Row: sum_i n_i pw_i - H * sum_{i>=1} d_i pw_i = H * pw_0
            for (int i = 0; i <= order; ++i) {
                const std::complex<double> v = pw[i] * wgt;
                A(2 * k, i) = v.real();
                A(2 * k + 1, i) = v.imag();
            }
            for (int i = 1; i <= order; ++i) {
                const std::complex<double> v = -H * pw[i] * wgt;
                A(2 * k, order + i) = v.real();
                A(2 * k + 1, order + i) = v.imag();
            }
            const std::complex<double> rhs = H * pw[0] * wgt;
            b(2 * k) = rhs.real();
            b(2 * k + 1) = rhs.imag();
        }
        x = A.colPivHouseholderQr().solve(b);
        for (int i = 0; i <= order; ++i) num[static_cast<size_t>(i)] = x[i];
        for (int i = 1; i <= order; ++i)
            dPrev[static_cast<size_t>(i)] = x[order + i];
    }
    den = dPrev;
    return {num, den};
}

// ============================================================================
// StructuredUncertainModel Implementation
// ============================================================================

void StructuredUncertainModel::setNominal(const double* A, const double* B,
                                          const double* C, const double* D,
                                          int n, int m, int p) {
    m_n = n;
    m_m = m;
    m_p = p;
    
    for (int i = 0; i < n * n; i++) m_A[i] = A[i];
    for (int i = 0; i < n * m; i++) m_B[i] = B[i];
    for (int i = 0; i < p * n; i++) m_C[i] = C[i];
    for (int i = 0; i < p * m; i++) m_D[i] = D[i];
}

void StructuredUncertainModel::addInputMultiplicative(const WeightingFunction& W, 
                                                       double bound) {
    UncertaintyDesc desc;
    desc.type = UncertaintyDesc::InputMult;
    desc.weight = W;
    desc.name = "Input Multiplicative";
    desc.bound = bound;
    m_uncertainties.push_back(desc);
}

void StructuredUncertainModel::addOutputMultiplicative(const WeightingFunction& W,
                                                        double bound) {
    UncertaintyDesc desc;
    desc.type = UncertaintyDesc::OutputMult;
    desc.weight = W;
    desc.name = "Output Multiplicative";
    desc.bound = bound;
    m_uncertainties.push_back(desc);
}

void StructuredUncertainModel::addAdditive(const WeightingFunction& W, double bound) {
    UncertaintyDesc desc;
    desc.type = UncertaintyDesc::Additive;
    desc.weight = W;
    desc.name = "Additive";
    desc.bound = bound;
    m_uncertainties.push_back(desc);
}

void StructuredUncertainModel::addParametric(const std::string& paramName,
                                              const double* A_delta, 
                                              const double* B_delta,
                                              double bound) {
    UncertaintyDesc desc;
    desc.type = UncertaintyDesc::Parametric;
    desc.name = paramName;
    desc.bound = bound;
    
    if (A_delta) {
        desc.sensitivity_A.assign(A_delta, A_delta + m_n * m_n);
    }
    if (B_delta) {
        desc.sensitivity_B.assign(B_delta, B_delta + m_n * m_m);
    }
    
    m_uncertainties.push_back(desc);
}

void StructuredUncertainModel::buildLFT(double* M11, double* M12,
                                        double* M21, double* M22,
                                        std::vector<UncertaintyBlock>& blocks) const {
    // Build LFT representation: G = F_u(M, Δ)
    // M = [M11, M12; M21, M22]
    
    blocks.clear();
    
    // For each uncertainty, add corresponding block structure
    for (const auto& unc : m_uncertainties) {
        UncertaintyBlock block;
        block.name = unc.name;
        block.bound = unc.bound;
        
        switch (unc.type) {
            case UncertaintyDesc::InputMult:
            case UncertaintyDesc::OutputMult:
                block.type = MuBlockType::Full;
                block.rows = m_m;
                block.cols = m_m;
                break;
            case UncertaintyDesc::Additive:
                block.type = MuBlockType::Full;
                block.rows = m_p;
                block.cols = m_m;
                break;
            case UncertaintyDesc::Parametric:
                block.type = MuBlockType::Real;
                block.rows = 1;
                block.cols = 1;
                break;
        }
        
        blocks.push_back(block);
    }
    
    // Build M matrices (simplified: use nominal plant)
    if (M11 && M12 && M21 && M22) {
        for (int i = 0; i < m_n * m_n; i++) M11[i] = m_A[i];
        for (int i = 0; i < m_n * m_m; i++) M12[i] = m_B[i];
        for (int i = 0; i < m_p * m_n; i++) M21[i] = m_C[i];
        for (int i = 0; i < m_p * m_m; i++) M22[i] = m_D[i];
    }
}

void StructuredUncertainModel::sampleUncertainty(double* A, double* B,
                                                  double* C, double* D) const {
    // Sample one realization of the uncertain system
    // Copy nominal
    for (int i = 0; i < m_n * m_n; i++) A[i] = m_A[i];
    for (int i = 0; i < m_n * m_m; i++) B[i] = m_B[i];
    for (int i = 0; i < m_p * m_n; i++) C[i] = m_C[i];
    for (int i = 0; i < m_p * m_m; i++) D[i] = m_D[i];
    
    // Add parametric perturbations
    for (const auto& unc : m_uncertainties) {
        if (unc.type == UncertaintyDesc::Parametric) {
            // Random perturbation within bounds
            double delta = (static_cast<double>(rand()) / RAND_MAX - 0.5) * 2.0 * unc.bound;
            
            for (size_t i = 0; i < unc.sensitivity_A.size() && i < static_cast<size_t>(m_n * m_n); i++) {
                A[i] += delta * unc.sensitivity_A[i];
            }
            for (size_t i = 0; i < unc.sensitivity_B.size() && i < static_cast<size_t>(m_n * m_m); i++) {
                B[i] += delta * unc.sensitivity_B[i];
            }
        }
    }
}

} // namespace tether::control
