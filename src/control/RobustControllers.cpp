/**
 * @file RobustControllers.cpp
 * @brief Implementation of H2 and H∞ Robust Controllers
 */

#include "control/RobustControllers.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

namespace tether::control {

// ============================================================================
// WeightingFunction Implementation
// ============================================================================

WeightingFunction WeightingFunction::firstOrder(double zero, double pole, double gain) {
    WeightingFunction w;
    w.order = 1;
    w.gain = gain;
    w.num[0] = 0;
    w.num[1] = 1;
    w.num[2] = zero;
    w.den[0] = 0;
    w.den[1] = 1;
    w.den[2] = pole;
    return w;
}

WeightingFunction WeightingFunction::integrator(double gain) {
    WeightingFunction w;
    w.order = 1;
    w.gain = gain;
    w.num[0] = 0;
    w.num[1] = 0;
    w.num[2] = 1;
    w.den[0] = 0;
    w.den[1] = 1;
    w.den[2] = 0;
    return w;
}

WeightingFunction WeightingFunction::sensitivity(double M, double omegaB, double epsilon) {
    // W₁(s) = (s/M + ωB) / (s + ωB·ε)
    // = (s + M·ωB) / (M·(s + ωB·ε))
    WeightingFunction w;
    w.order = 1;
    w.gain = 1.0 / M;
    w.num[0] = 0;
    w.num[1] = 1;
    w.num[2] = M * omegaB;
    w.den[0] = 0;
    w.den[1] = 1;
    w.den[2] = omegaB * epsilon;
    return w;
}

WeightingFunction WeightingFunction::complementary(double M, double omegaT, double epsilon) {
    // W₃(s) = (s + ωT/M) / (ε·s + ωT)
    WeightingFunction w;
    w.order = 1;
    w.gain = 1.0 / epsilon;
    w.num[0] = 0;
    w.num[1] = epsilon;
    w.num[2] = omegaT / M * epsilon;
    w.den[0] = 0;
    w.den[1] = epsilon;
    w.den[2] = omegaT;
    return w;
}

double WeightingFunction::magnitude(double omega) const {
    // Evaluate |W(jω)|
    double numReal = num[2] - num[0] * omega * omega;
    double numImag = num[1] * omega;
    double denReal = den[2] - den[0] * omega * omega;
    double denImag = den[1] * omega;
    
    double numMag = std::sqrt(numReal * numReal + numImag * numImag);
    double denMag = std::sqrt(denReal * denReal + denImag * denImag);
    
    return gain * numMag / denMag;
}

void WeightingFunction::toStateSpace(double* A, double* B, double* C, double* D, int& n) const {
    if (order == 0) {
        n = 0;
        D[0] = gain;
        return;
    }
    
    if (order == 1) {
        n = 1;
        // First order: ẋ = -a*x + b*u, y = c*x + d*u
        // For (s+z)/(s+p): ẋ = -p*x + u, y = (z-p)*x + u
        double p = den[2] / den[1];
        double z = num[2] / num[1];
        
        A[0] = -p;
        B[0] = 1.0;
        C[0] = gain * (z - p);
        D[0] = gain;
        return;
    }
    
    // Second order (controllable canonical form)
    n = 2;
    double a0 = den[2] / den[0];
    double a1 = den[1] / den[0];
    double b0 = num[2] / den[0];
    double b1 = num[1] / den[0];
    double b2 = num[0] / den[0];
    
    A[0] = 0;      A[1] = 1;
    A[2] = -a0;    A[3] = -a1;
    
    B[0] = 0;
    B[1] = 1;
    
    C[0] = gain * (b0 - b2 * a0);
    C[1] = gain * (b1 - b2 * a1);
    
    D[0] = gain * b2;
}
namespace {

using SpMat = Eigen::MatrixXd;
using SpCmat = Eigen::Matrix<std::complex<double>, Eigen::Dynamic, Eigen::Dynamic>;
using RM = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

SpMat loadMat(const double* p, int rows, int cols) {
    return SpMat(Eigen::Map<const RM>(p, rows, cols));
}

void storeMat(double* p, int rows, int cols, const SpMat& M) {
    for (int i = 0; i < rows; ++i)
        for (int j = 0; j < cols; ++j)
            p[i * cols + j] = M(i, j);
}

bool safeInverse(const SpMat& M, SpMat& inv) {
    Eigen::JacobiSVD<SpMat> svd(M);
    if (svd.info() != Eigen::Success || M.rows() == 0) return false;
    const auto& sv = svd.singularValues();
    double smax = sv(0), smin = sv(sv.size() - 1);
    if (!(smax > 0) || smin <= 1e-10 * smax) return false;
    inv = M.inverse();
    return true;
}

bool isHurwitz(const SpMat& A) {
    Eigen::EigenSolver<SpMat> es(A);
    if (es.info() != Eigen::Success) return false;
    return es.eigenvalues().real().maxCoeff() < 1e-7;
}

/**
 * Solve the game-type algebraic Riccati equation
 *     M'X + XM + Q + X S X = 0
 * for the stabilizing solution X >= 0 (i.e. M + S X Hurwitz).
 *
 * Written as the standard CARE  A'X + XA - X Pi X + Theta = 0  with
 * A = M, Pi = -S, Theta = Q, its Hamiltonian is  H = [M, S; -Q, -M'].
 * The n-dimensional stable invariant subspace of H yields X = Z2 Z1^{-1}.
 */
bool solveGameCARE(const SpMat& M, const SpMat& S, const SpMat& Q, SpMat& X) {
    const int n = static_cast<int>(M.rows());
    if (n == 0) return false;

    SpMat H(2 * n, 2 * n);
    H << M, S, -Q, -M.transpose();

    Eigen::EigenSolver<SpMat> es(H);
    if (es.info() != Eigen::Success) return false;

    const auto& w = es.eigenvalues();
    const auto& V = es.eigenvectors();
    SpCmat Z(2 * n, n);
    int cnt = 0;
    for (int i = 0; i < 2 * n; ++i) {
        if (w[i].real() < -1e-9) {
            if (cnt >= n) return false;  // Hamiltonian must have exactly n stable eigenvalues
            Z.col(cnt++) = V.col(i);
        }
    }
    if (cnt != n) return false;

    SpCmat Xc = Z.bottomRows(n) * Z.topRows(n).inverse();
    double scale = std::max(1.0, Xc.real().cwiseAbs().maxCoeff());
    if (Xc.imag().cwiseAbs().maxCoeff() > 1e-6 * scale) return false;
    X = Xc.real();

    // relative residual check
    SpMat R = M.transpose() * X + X * M + Q + X * S * X;
    double rnorm = R.cwiseAbs().maxCoeff();
    double rscale = (M.transpose() * X).cwiseAbs().maxCoeff()
                  + (X * M).cwiseAbs().maxCoeff()
                  + Q.cwiseAbs().maxCoeff()
                  + (X * S * X).cwiseAbs().maxCoeff();
    if (!(rnorm <= 1e-6 * std::max(1.0, rscale))) return false;

    // stabilizing + PSD checks
    if (!isHurwitz(M + S * X)) return false;
    Eigen::SelfAdjointEigenSolver<SpMat> ps(X);
    if (ps.info() != Eigen::Success) return false;
    if (ps.eigenvalues().minCoeff() < -1e-7 * std::max(1.0, ps.eigenvalues().maxCoeff()))
        return false;
    return true;
}

/** Symmetric PSD square root: returns S with S*S = M (eigenvalue-clamped). */
SpMat sqrtPSD(const SpMat& M) {
    const int n = static_cast<int>(M.rows());
    Eigen::SelfAdjointEigenSolver<SpMat> es(0.5 * (M + M.transpose()));
    if (es.info() != Eigen::Success) return SpMat::Zero(n, n);
    SpMat S = SpMat::Zero(n, n);
    for (int i = 0; i < n; ++i) {
        double lam = std::max(0.0, es.eigenvalues()[i]);
        SpMat v = es.eigenvectors().col(i);
        S += std::sqrt(lam) * (v * v.transpose());
    }
    return S;
}

/**
 * Build the LFT closed loop  [Acl, Bcl; Cz, Dzw]  for
 *   u = K y  interconnection of generalized plant (A,B1,B2,C1,C2,D*) and
 *   controller (Ak,Bk,Ck,Dk).  Requires (I - Dk*D22) invertible.
 */
bool buildClosedLoop(const SpMat& Aa, const SpMat& B1, const SpMat& B2,
                     const SpMat& C1, const SpMat& C2,
                     const SpMat& D11, const SpMat& D12,
                     const SpMat& D21, const SpMat& D22,
                     const SpMat& Ak, const SpMat& Bk,
                     const SpMat& Ck, const SpMat& Dk,
                     SpMat& Acl, SpMat& Bcl, SpMat& Cz, SpMat& Dzw) {
    const int na = Aa.rows(), nw = B1.cols(), nu = B2.cols();
    const int nz = C1.rows(), ny = C2.rows(), nk = Ak.rows();
    SpMat Muu = SpMat::Identity(nu, nu) - Dk * D22;
    Eigen::FullPivLU<SpMat> lu(Muu);
    if (!lu.isInvertible()) return false;
    SpMat Mi = Muu.inverse();

    SpMat Tu_xa = Mi * Dk * C2;          // nu x na
    SpMat Tu_xk = Mi * Ck;               // nu x nk
    SpMat Tu_w  = Mi * Dk * D21;         // nu x nw

    int nx = na + nk;
    Acl = SpMat::Zero(nx, nx);
    Acl.topLeftCorner(na, na) = Aa + B2 * Tu_xa;
    Acl.topRightCorner(na, nk) = B2 * Tu_xk;
    Acl.bottomLeftCorner(nk, na) = Bk * (C2 + D22 * Tu_xa);
    Acl.bottomRightCorner(nk, nk) = Ak + Bk * (D22 * Tu_xk);

    Bcl = SpMat::Zero(nx, nw);
    Bcl.topRows(na) = B1 + B2 * Tu_w;
    Bcl.bottomRows(nk) = Bk * (D22 * Tu_w + D21);

    Cz = SpMat::Zero(nz, nx);
    Cz.leftCols(na) = C1 + D12 * Tu_xa;
    Cz.rightCols(nk) = D12 * Tu_xk;
    Dzw = D11 + D12 * Tu_w;
    return true;
}

// ============================================================================
// mu-analysis helpers
// ============================================================================

/**
 * D-scaling parametrization for the structured-singular-value upper bound
 *   mu(M) <= min_D sigma_bar(D M D^-1)
 *
 * D must lie in the commutant of the uncertainty structure:
 *  - non-repeated block of size k (full k x k block): only scalar matrices
 *    d*I_k commute with every k x k uncertainty, so all k diagonal
 *    positions share ONE free parameter.
 *  - repeated block of size k (delta*I_k): any matrix commutes with
 *    delta*I_k, so each diagonal position gets its OWN parameter
 *    (diagonal restriction of the general positive-definite scaling —
 *    still a valid, slightly conservative, upper bound).
 *  - size-1 blocks: one parameter each.
 */
struct MuDParams {
    int nPos{0};              // total diagonal positions = sum(blockSizes)
    int nPar{0};              // number of free log-scale parameters
    std::vector<int> posPar;  // position -> parameter index
    std::vector<double> x;    // log-parameters (d_i = exp(x))
};

MuDParams buildMuParams(const int* blockSizes, const bool* repeated, int numBlocks) {
    MuDParams mp;
    mp.posPar.clear();
    mp.x.clear();
    int pos = 0;
    for (int b = 0; b < numBlocks; ++b) {
        const int k = blockSizes[b];
        if (k <= 0) continue;
        if (repeated && repeated[b]) {
            for (int j = 0; j < k; ++j) {
                mp.posPar.push_back(static_cast<int>(mp.x.size()));
                mp.x.push_back(0.0);
            }
        } else {
            const int p = static_cast<int>(mp.x.size());
            mp.x.push_back(0.0);
            for (int j = 0; j < k; ++j) mp.posPar.push_back(p);
        }
        pos += k;
    }
    mp.nPos = pos;
    mp.nPar = static_cast<int>(mp.x.size());
    return mp;
}

/** Apply diagonal D=diag(exp(x[posPar[i]])) : returns D M D^-1 (real). */
double sigmaBarScaledReal(const SpMat& M, const MuDParams& mp) {
    const int n = static_cast<int>(M.rows());
    SpMat S = SpMat::Zero(n, n);
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) {
            const double di = std::exp(mp.x[mp.posPar[i]]);
            const double dj = std::exp(mp.x[mp.posPar[j]]);
            S(i, j) = M(i, j) * di / dj;
        }
    Eigen::JacobiSVD<SpMat> svd(S);
    if (svd.info() != Eigen::Success || svd.singularValues().size() == 0)
        return std::numeric_limits<double>::infinity();
    return svd.singularValues()(0);
}

/** Same for a complex matrix (frequency response M(jw)). */
double sigmaBarScaledComplex(const SpCmat& M, const MuDParams& mp) {
    const int n = static_cast<int>(M.rows());
    SpCmat S(n, n);
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) {
            const double di = std::exp(mp.x[mp.posPar[i]]);
            const double dj = std::exp(mp.x[mp.posPar[j]]);
            S(i, j) = M(i, j) * (di / dj);
        }
    Eigen::JacobiSVD<SpCmat> svd(S);
    if (svd.info() != Eigen::Success || svd.singularValues().size() == 0)
        return std::numeric_limits<double>::infinity();
    return svd.singularValues()(0);
}

/** Unstructured spectral norm (valid, looser mu upper bound). */
double sigmaBarPlain(const SpMat& M) {
    Eigen::JacobiSVD<SpMat> svd(M);
    if (svd.info() != Eigen::Success || svd.singularValues().size() == 0)
        return std::numeric_limits<double>::infinity();
    return svd.singularValues()(0);
}

/** Golden-section minimization of f(t) on [a,b]. */
double goldenMin(const std::function<double(double)>& f, double a, double b, int iters = 24) {
    const double gr = 0.6180339887498949;
    double c = b - gr * (b - a), d = a + gr * (b - a);
    double fc = f(c), fd = f(d);
    for (int i = 0; i < iters; ++i) {
        if (fc < fd) { b = d; d = c; fd = fc; c = b - gr * (b - a); fc = f(c); }
        else         { a = c; c = d; fc = fd; d = a + gr * (b - a); fd = f(d); }
    }
    return 0.5 * (a + b);
}

/**
 * Coordinate-descent optimization of the D-scaling parameters.
 * Objective: minimize sigma_bar(D M D^-1).  The objective is invariant
 * to a common shift of all log-parameters, so the mean is pinned to 0
 * after each sweep for numerical conditioning.
 */
template <typename F>
void optimizeMuScales(MuDParams& mp, F&& eval) {
    if (mp.nPar <= 0) return;
    for (int sweep = 0; sweep < 5; ++sweep) {
        double maxDelta = 0.0;
        for (int p = 0; p < mp.nPar; ++p) {
            const double x0 = mp.x[p];
            auto f = [&](double t) {
                mp.x[p] = t;
                return eval(mp);
            };
            // search a window around the current value (log units)
            const double t = goldenMin(f, x0 - 3.0, x0 + 3.0);
            mp.x[p] = t;
            maxDelta = std::max(maxDelta, std::abs(t - x0));
        }
        // pin mean to zero (objective is shift-invariant)
        double mean = 0.0;
        for (double v : mp.x) mean += v;
        mean /= mp.nPar;
        for (double& v : mp.x) v -= mean;
        if (maxDelta < 1e-4) break;
    }
}

} // namespace

// ============================================================================
// H2 Controller Implementation
// ============================================================================

void H2Controller::setGeneralizedPlant(const double* A, const double* B1, const double* B2,
                                        const double* C1, const double* C2,
                                        const double* D11, const double* D12,
                                        const double* D21, const double* D22,
                                        int n, int nw, int nu, int nz, int ny) {
    m_n = std::min(n, static_cast<int>(MAX_STATE_DIM));
    m_nw = std::min(nw, static_cast<int>(MAX_STATE_DIM));
    m_nu = std::min(nu, static_cast<int>(MAX_CONTROL_DIM));
    m_nz = std::min(nz, static_cast<int>(MAX_OUTPUT_DIM));
    m_ny = std::min(ny, static_cast<int>(MAX_OUTPUT_DIM));
    m_plantValid = (n <= static_cast<int>(MAX_STATE_DIM) &&
                    nw <= static_cast<int>(MAX_STATE_DIM) &&
                    nu <= static_cast<int>(MAX_CONTROL_DIM) &&
                    nz <= static_cast<int>(MAX_OUTPUT_DIM) &&
                    ny <= static_cast<int>(MAX_OUTPUT_DIM));

    std::memset(m_A.data(), 0, m_A.size() * sizeof(double));
    std::memset(m_B1.data(), 0, m_B1.size() * sizeof(double));
    std::memset(m_B2.data(), 0, m_B2.size() * sizeof(double));
    std::memset(m_C1.data(), 0, m_C1.size() * sizeof(double));
    std::memset(m_C2.data(), 0, m_C2.size() * sizeof(double));
    std::memset(m_D12.data(), 0, m_D12.size() * sizeof(double));
    std::memset(m_D21.data(), 0, m_D21.size() * sizeof(double));

    std::memcpy(m_A.data(), A, static_cast<size_t>(m_n) * static_cast<size_t>(m_n) * sizeof(double));
    std::memcpy(m_B1.data(), B1, static_cast<size_t>(m_n) * static_cast<size_t>(m_nw) * sizeof(double));
    std::memcpy(m_B2.data(), B2, static_cast<size_t>(m_n) * static_cast<size_t>(m_nu) * sizeof(double));
    std::memcpy(m_C1.data(), C1, static_cast<size_t>(m_nz) * static_cast<size_t>(m_n) * sizeof(double));
    std::memcpy(m_C2.data(), C2, static_cast<size_t>(m_ny) * static_cast<size_t>(m_n) * sizeof(double));
    std::memcpy(m_D12.data(), D12, static_cast<size_t>(m_nz) * static_cast<size_t>(m_nu) * sizeof(double));
    std::memcpy(m_D21.data(), D21, static_cast<size_t>(m_ny) * static_cast<size_t>(m_nw) * sizeof(double));

    m_d11Max = m_d22Max = 0.0;
    if (D11) for (int i = 0; i < m_nz * m_nw; ++i) m_d11Max = std::max(m_d11Max, std::abs(D11[i]));
    if (D22) for (int i = 0; i < m_ny * m_nu; ++i) m_d22Max = std::max(m_d22Max, std::abs(D22[i]));

    m_designed = false;
}

void H2Controller::setRegulatorProblem(const double* A, const double* B, const double* C,
                                        const double* Q, const double* R,
                                        const double* W, const double* V,
                                        int n, int m, int p) {
    // Build the standard LQG generalized plant for the regulation problem
    //   x' = Ax + B2 u + w0 ,   w0 ~ (0, W)   (process noise, n channels)
    //   z  = [Q^½ x ; R^½ u]                    (weighted state + control)
    //   y  = Cx + v      ,   v  ~ (0, V)   (measurement noise, p channels)
    //
    //   w = [w0 (n) ; v (p)]  -> nw = n + p
    //   B1 = [sqrt(W) | 0]      (n x nw)
    //   C1 = [sqrt(Q) ; 0]      (nz x n,  nz = n + m)
    //   D12 = [0 ; sqrt(R)]     (nz x m)
    //   D21 = [0 | sqrt(V)]     (ny x nw, ny = p)
    //   D11 = D22 = 0
    const int nn = std::min(n, static_cast<int>(MAX_STATE_DIM));
    const int mm = std::min(m, static_cast<int>(MAX_CONTROL_DIM));
    const int pp = std::min(p, static_cast<int>(MAX_OUTPUT_DIM));
    const int nw = nn + pp;
    const int nz = nn + mm;

    m_plantValid = (n <= static_cast<int>(MAX_STATE_DIM) &&
                    m <= static_cast<int>(MAX_CONTROL_DIM) &&
                    p <= static_cast<int>(MAX_OUTPUT_DIM) &&
                    nz <= static_cast<int>(MAX_OUTPUT_DIM) &&
                    nw <= static_cast<int>(MAX_STATE_DIM));
    if (!m_plantValid) { m_designed = false; return; }

    // square roots of the (symmetric PSD) weight matrices
    SpMat Qm = SpMat::Zero(nn, nn), Rm = SpMat::Zero(mm, mm);
    SpMat Wm = SpMat::Zero(nn, nn), Vm = SpMat::Zero(pp, pp);
    for (int i = 0; i < nn; ++i)
        for (int j = 0; j < nn; ++j) Qm(i, j) = Q ? Q[i * n + j] : (i == j ? 1.0 : 0.0);
    for (int i = 0; i < mm; ++i)
        for (int j = 0; j < mm; ++j) Rm(i, j) = R ? R[i * m + j] : (i == j ? 1.0 : 0.0);
    for (int i = 0; i < nn; ++i)
        for (int j = 0; j < nn; ++j) Wm(i, j) = W ? W[i * n + j] : (i == j ? 0.01 : 0.0);
    for (int i = 0; i < pp; ++i)
        for (int j = 0; j < pp; ++j) Vm(i, j) = V ? V[i * p + j] : (i == j ? 0.01 : 0.0);

    SpMat sqQ = sqrtPSD(Qm), sqR = sqrtPSD(Rm);
    SpMat sqW = sqrtPSD(Wm), sqV = sqrtPSD(Vm);

    // pack into flat row-major buffers and delegate to setGeneralizedPlant
    std::vector<double> bA(nn * nn), bB1(nn * nw), bB2(nn * mm),
                    bC1(nz * nn), bC2(pp * nn),
                    bD11(nz * nw, 0.0), bD12(nz * mm, 0.0),
                    bD21(pp * nw, 0.0), bD22(pp * mm, 0.0);

    for (int i = 0; i < nn; ++i)
        for (int j = 0; j < nn; ++j) bA[i * nn + j] = A[i * n + j];
    for (int i = 0; i < nn; ++i) {
        for (int j = 0; j < mm; ++j) bB2[i * mm + j] = B[i * m + j];
        for (int j = 0; j < nn; ++j) bB1[i * nw + j] = sqW(i, j);
        // bB1 noise columns (nn..nw) stay zero
    }
    for (int i = 0; i < pp; ++i)
        for (int j = 0; j < nn; ++j) bC2[i * nn + j] = C[i * n + j];
    for (int i = 0; i < nn; ++i)
        for (int j = 0; j < nn; ++j) bC1[i * nn + j] = sqQ(i, j);
    for (int i = 0; i < mm; ++i)
        for (int j = 0; j < mm; ++j) bD12[(nn + i) * mm + j] = sqR(i, j);
    for (int i = 0; i < pp; ++i)
        for (int j = 0; j < pp; ++j) bD21[i * nw + nn + j] = sqV(i, j);

    setGeneralizedPlant(bA.data(), bB1.data(), bB2.data(), bC1.data(), bC2.data(),
                        bD11.data(), bD12.data(), bD21.data(), bD22.data(),
                        nn, nw, mm, nz, pp);
    m_plantValid = true;
}

bool H2Controller::design() {
    // H2 optimal control = LQG separation principle, i.e. the gamma -> inf
    // limit of the DGKF solution. Two standard algebraic Riccati equations:
    //
    //   Control: (A - B2 R^-1 D12'C1)'X + X(A - B2 R^-1 D12'C1)
    //            + C1'(I - D12 R^-1 D12')C1 - X B2 R^-1 B2' X = 0
    //   Filter:  (A - B1 D21' Rt^-1 C2)'Y + Y(A - B1 D21' Rt^-1 C2)
    //            + B1(I - D21' Rt^-1 D21)B1' - Y C2' Rt^-1 C2 Y = 0
    //
    // with R = D12'D12, Rt = D21 D21'. Assumptions: D11 = D22 = 0,
    // rank(D12) = nu, rank(D21) = ny.
    if (!m_plantValid || m_n <= 0) return false;
    if (m_d11Max > 1e-9 || m_d22Max > 1e-9) return false;

    const int n = m_n, nw = m_nw, nu = m_nu, nz = m_nz, ny = m_ny;
    SpMat A   = loadMat(m_A.data(),  n,  n);
    SpMat B1  = loadMat(m_B1.data(), n,  nw);
    SpMat B2  = loadMat(m_B2.data(), n,  nu);
    SpMat C1  = loadMat(m_C1.data(), nz, n);
    SpMat C2  = loadMat(m_C2.data(), ny, n);
    SpMat D12 = loadMat(m_D12.data(),nz, nu);
    SpMat D21 = loadMat(m_D21.data(),ny, nw);

    SpMat R = D12.transpose() * D12;
    SpMat Rt = D21 * D21.transpose();
    SpMat Rinv, Rtinv;
    if (!safeInverse(R, Rinv) || !safeInverse(Rt, Rtinv)) return false;

    SpMat I_nz = SpMat::Identity(nz, nz);
    SpMat I_nw = SpMat::Identity(nw, nw);

    // control ARE for X (standard CARE, S negative semidefinite)
    SpMat Abar_x = A - B2 * Rinv * (D12.transpose() * C1);
    SpMat Qx = C1.transpose() * (I_nz - D12 * Rinv * D12.transpose()) * C1;
    Qx = 0.5 * (Qx + Qx.transpose());
    SpMat Sx = -B2 * Rinv * B2.transpose();
    Sx = 0.5 * (Sx + Sx.transpose());
    SpMat X;
    if (!solveGameCARE(Abar_x, Sx, Qx, X)) return false;

    // filter ARE for Y (solved in transposed/dual form)
    SpMat Abar_y = A - B1 * (D21.transpose() * Rtinv) * C2;
    SpMat Qy = B1 * (I_nw - D21.transpose() * Rtinv * D21) * B1.transpose();
    Qy = 0.5 * (Qy + Qy.transpose());
    SpMat Sy = -C2.transpose() * Rtinv * C2;
    Sy = 0.5 * (Sy + Sy.transpose());
    SpMat Y;
    if (!solveGameCARE(Abar_y.transpose(), Sy, Qy, Y)) return false;

    // separation controller (gamma -> inf limit of the DGKF central form):
    //   F = -R^-1 (B2'X + D12'C1)      state feedback
    //   L = -(Y C2' + B1 D21') Rt^-1  observer gain
    //   Ak = A + B2 F + L C2 , Bk = L , Ck = -F , Dk = 0
    SpMat F = -Rinv * (B2.transpose() * X + D12.transpose() * C1);
    SpMat L = -(Y * C2.transpose() + B1 * D21.transpose()) * Rtinv;
    SpMat Ak = A + B2 * F + L * C2;
    SpMat Bk = L;
    SpMat Ck = -F;
    SpMat Dk = SpMat::Zero(nu, ny);

    m_nk = n;
    storeMat(m_Ak.data(), n, n, Ak);
    storeMat(m_Bk.data(), n, ny, Bk);
    storeMat(m_Ck.data(), nu, n, Ck);
    storeMat(m_Dk.data(), nu, ny, Dk);

    // achieved H2 norm via the closed-loop controllability Gramian:
    //   Acl P + P Acl' + Bcl Bcl' = 0  ->  ||Tzw||_2^2 = tr(Cz P Cz')
    SpMat D11 = SpMat::Zero(nz, nw), D22 = SpMat::Zero(ny, nu);
    SpMat Acl, Bcl, Cz, Dzw;
    m_h2Norm = 0.0;
    if (buildClosedLoop(A, B1, B2, C1, C2, D11, D12, D21, D22,
                        Ak, Bk, Ck, Dk, Acl, Bcl, Cz, Dzw)) {
        SpMat Qg = Bcl * Bcl.transpose();
        SpMat P;
        if (solveGameCARE(Acl.transpose(), SpMat::Zero(Acl.rows(), Acl.rows()), Qg, P)) {
            SpMat G = Cz * P * Cz.transpose();
            m_h2Norm = std::sqrt(std::max(0.0, G.trace()));
        }
    }

    m_designed = true;
    m_dtLast = -1.0;
    return true;
}

void H2Controller::getControllerMatrices(double* Ak, double* Bk, double* Ck, double* Dk) const {
    if (Ak) std::memcpy(Ak, m_Ak.data(), m_nk * m_nk * sizeof(double));
    if (Bk) std::memcpy(Bk, m_Bk.data(), m_nk * m_ny * sizeof(double));
    if (Ck) std::memcpy(Ck, m_Ck.data(), m_nu * m_nk * sizeof(double));
    if (Dk) std::memcpy(Dk, m_Dk.data(), m_nu * m_ny * sizeof(double));
}

void H2Controller::discretizeController(double dt) {
    // Tustin (bilinear) discretization of the continuous-time controller:
    //   Ad = (I - dt/2 A)^-1 (I + dt/2 A),  Bd = (I - dt/2 A)^-1 dt B
    const int n = m_nk, ny = m_ny;
    SpMat A = loadMat(m_Ak.data(), n, n);
    SpMat B = loadMat(m_Bk.data(), n, ny);
    SpMat T = 0.5 * dt * SpMat::Identity(n, n);
    SpMat M = SpMat::Identity(n, n) - T * A;
    SpMat Ad = M.partialPivLu().solve(SpMat::Identity(n, n) + T * A);
    SpMat Bd = M.partialPivLu().solve(dt * B);
    if (!Ad.allFinite() || !Bd.allFinite()) {
        Ad = SpMat::Identity(n, n) + dt * A;
        Bd = dt * B;
    }
    storeMat(m_Adk.data(), n, n, Ad);
    storeMat(m_Bdk.data(), n, ny, Bd);
}

ControllerOutput H2Controller::computeImpl(const ControllerInput& input) {
    ControllerOutput output;

    if (!m_designed || m_nk <= 0) {
        output.control = 0;
        return output;
    }

    const double dt = input.dt > 0.0 ? input.dt : 1e-3;
    if (dt != m_dtLast) {
        discretizeController(dt);
        m_dtLast = dt;
    }

    // The generalized plant defines y = Cx + v (raw measurement, not error).
    // SISO: controller input is the measured output.
    // MIMO: caller supplies the measurement vector in input.state.
    double y[MAX_OUTPUT_DIM]{};
    if (m_ny == 1) {
        y[0] = input.measured;
    } else {
        for (int i = 0; i < m_ny; ++i)
            y[i] = (i < static_cast<int>(input.stateDim)) ? input.state[i] : 0.0;
    }

    // xk+ = Adk xk + Bdk y
    double xn[MAX_STATE_DIM]{};
    for (int i = 0; i < m_nk; ++i) {
        double s = 0.0;
        for (int j = 0; j < m_nk; ++j) s += m_Adk[i * m_nk + j] * m_xk[j];
        for (int j = 0; j < m_ny; ++j) s += m_Bdk[i * m_ny + j] * y[j];
        xn[i] = s;
    }
    for (int i = 0; i < m_nk; ++i) m_xk[i] = xn[i];

    // u = Ck xk + Dk y
    double u[MAX_CONTROL_DIM]{};
    for (int i = 0; i < m_nu; ++i) {
        double s = 0.0;
        for (int j = 0; j < m_nk; ++j) s += m_Ck[i * m_nk + j] * m_xk[j];
        for (int j = 0; j < m_ny; ++j) s += m_Dk[i * m_ny + j] * y[j];
        u[i] = s;
    }

    output.control = std::clamp(u[0], m_limits.outputMin, m_limits.outputMax);
    for (int i = 0; i < m_nu && i < static_cast<int>(MAX_CONTROL_DIM); ++i)
        output.controlVector[i] = u[i];
    output.error = input.reference - y[0];
    output.saturated = (u[0] != output.control);

    return output;
}

void H2Controller::resetImpl() {
    std::fill(m_xk.begin(), m_xk.end(), 0.0);
    m_dtLast = -1.0;
}

// ============================================================================
// H∞ Controller Implementation
// ============================================================================

void HInfinityController::setPlant(const double* A, const double* B,
                                    const double* C, const double* D,
                                    int n, int m, int p) {
    m_n = std::min(n, static_cast<int>(MAX_STATE_DIM));
    m_m = std::min(m, static_cast<int>(MAX_CONTROL_DIM));
    m_p = std::min(p, static_cast<int>(MAX_OUTPUT_DIM));

    std::memcpy(m_A.data(), A, static_cast<size_t>(m_n) * static_cast<size_t>(m_n) * sizeof(double));
    std::memcpy(m_B.data(), B, static_cast<size_t>(m_n) * static_cast<size_t>(m_m) * sizeof(double));
    std::memcpy(m_C.data(), C, static_cast<size_t>(m_p) * static_cast<size_t>(m_n) * sizeof(double));
    if (D) {
        std::memcpy(m_D.data(), D, static_cast<size_t>(m_p) * static_cast<size_t>(m_m) * sizeof(double));
    }

    m_genMode = false;
    m_designed = false;
}

void HInfinityController::setGeneralizedPlant(const double* A, const double* B1,
                                               const double* B2,
                                               const double* C1, const double* C2,
                                               const double* D11, const double* D12,
                                               const double* D21, const double* D22,
                                               int n, int nw, int nu, int nz, int ny) {
    m_genMode = true;
    m_n = std::min(n, static_cast<int>(MAX_AUG));
    m_nw = std::min(nw, static_cast<int>(MAX_AUG));
    m_nu = std::min(nu, static_cast<int>(MAX_CONTROL_DIM));
    m_nz = std::min(nz, static_cast<int>(MAX_AUG));
    m_ny = std::min(ny, static_cast<int>(MAX_OUTPUT_DIM));
    m_na = m_n;

    std::memset(m_Aa.data(), 0, m_Aa.size() * sizeof(double));
    std::memset(m_B1a.data(), 0, m_B1a.size() * sizeof(double));
    std::memset(m_B2a.data(), 0, m_B2a.size() * sizeof(double));
    std::memset(m_C1a.data(), 0, m_C1a.size() * sizeof(double));
    std::memset(m_C2a.data(), 0, m_C2a.size() * sizeof(double));
    std::memset(m_D11a.data(), 0, m_D11a.size() * sizeof(double));
    std::memset(m_D12a.data(), 0, m_D12a.size() * sizeof(double));
    std::memset(m_D21a.data(), 0, m_D21a.size() * sizeof(double));
    std::memset(m_D22a.data(), 0, m_D22a.size() * sizeof(double));

    std::memcpy(m_Aa.data(), A, static_cast<size_t>(m_na) * static_cast<size_t>(m_na) * sizeof(double));
    std::memcpy(m_B1a.data(), B1, static_cast<size_t>(m_na) * static_cast<size_t>(m_nw) * sizeof(double));
    std::memcpy(m_B2a.data(), B2, static_cast<size_t>(m_na) * static_cast<size_t>(m_nu) * sizeof(double));
    std::memcpy(m_C1a.data(), C1, static_cast<size_t>(m_nz) * static_cast<size_t>(m_na) * sizeof(double));
    std::memcpy(m_C2a.data(), C2, static_cast<size_t>(m_ny) * static_cast<size_t>(m_na) * sizeof(double));
    if (D11) std::memcpy(m_D11a.data(), D11, static_cast<size_t>(m_nz) * static_cast<size_t>(m_nw) * sizeof(double));
    if (D12) std::memcpy(m_D12a.data(), D12, static_cast<size_t>(m_nz) * static_cast<size_t>(m_nu) * sizeof(double));
    if (D21) std::memcpy(m_D21a.data(), D21, static_cast<size_t>(m_ny) * static_cast<size_t>(m_nw) * sizeof(double));
    if (D22) std::memcpy(m_D22a.data(), D22, static_cast<size_t>(m_ny) * static_cast<size_t>(m_nu) * sizeof(double));

    m_designed = false;
}

void HInfinityController::setSensitivityWeight(double M, double omegaB, double epsilon) {
    m_W1 = WeightingFunction::sensitivity(M, omegaB, epsilon);
    m_hasW1 = true;
    m_designed = false;
}

void HInfinityController::setControlWeight(double maxControl) {
    m_W2 = WeightingFunction::firstOrder(0, 1, 1.0 / maxControl);
    m_hasW2 = true;
    m_designed = false;
}

void HInfinityController::setComplementaryWeight(double M, double omegaT, double epsilon) {
    m_W3 = WeightingFunction::complementary(M, omegaT, epsilon);
    m_hasW3 = true;
    m_designed = false;
}

void HInfinityController::setWeights(const WeightingFunction& W1,
                                      const WeightingFunction& W2,
                                      const WeightingFunction& W3) {
    m_W1 = W1;
    m_W2 = W2;
    m_W3 = W3;
    m_hasW1 = m_hasW2 = m_hasW3 = true;
    m_designed = false;
}


void HInfinityController::buildAugmentedPlant() {
    if (m_genMode) {
        // setGeneralizedPlant() already stored the full generalized plant.
        return;
    }

    // ------------------------------------------------------------------
    // Mixed-sensitivity augmentation of a plain LTI plant
    //   xdot = A x + B u ,   y = C x        (D must be ~0: strictly proper)
    //
    // Exogenous input  w = [r (p) ; d (m) ; n (p)]
    //   r : reference,      enters measurement ym = r - y + n and W1 input e = r - y
    //   d : input disturbance, enters plant through B
    //   n : sensor noise,    enters measurement ym only
    // Performance output z = [W1 e ; W2 u ; W3 y]
    // Measurement       ym = r - C x + n   (the tracking error seen by K)
    //
    // Weight direct feedthrough on the w->z path is suppressed (W1's D is
    // forced to zero) so that D11 = 0 and the standard DGKF assumptions hold.
    // ------------------------------------------------------------------
    const int n = m_n, m = m_m, p = m_p;
    m_na = m_nw = m_nu = m_nz = m_ny = 0;
    if (n <= 0 || m <= 0 || p <= 0) return;

    bool dZero = true;
    for (int i = 0; i < p * m; ++i)
        if (std::abs(m_D[i]) > 1e-12) { dZero = false; break; }
    if (!dZero) return;  // D22 != 0 unsupported

    // weight state-space per channel (SISO weights, replicated diagonally)
    struct WSS { int o{0}; double A[4]{}, B[2]{}, C[2]{}, D{0}; };
    auto loadW = [](const WeightingFunction& w, WSS& s) {
        int on = 0;
        double A[4]{}, B[2]{}, C[2]{}, D[1]{};
        w.toStateSpace(A, B, C, &D[0], on);
        s.o = on;
        for (int i = 0; i < 4; ++i) s.A[i] = A[i];
        for (int i = 0; i < 2; ++i) { s.B[i] = B[i]; s.C[i] = C[i]; }
        s.D = D[0];
    };
    WSS w1, w2, w3;
    if (m_hasW1) loadW(m_W1, w1);
    if (m_hasW2) loadW(m_W2, w2);
    if (m_hasW3) loadW(m_W3, w3);
    // D12 must have full column rank for the DGKF solution: every control
    // channel needs a penalty. When the user supplies no W2, fall back to a
    // small static control weight so the problem stays well-posed.
    if (!m_hasW2) { w2.o = 0; w2.D = 0.1; }

    const int o1 = m_hasW1 ? w1.o : 0;
    const int o2 = w2.o;
    const int o3 = m_hasW3 ? w3.o : 0;
    const int n1 = p * o1, n2 = m * o2, n3 = p * o3;
    const int na = n + n1 + n2 + n3;
    const int nw = 2 * p + m;
    const int nz = (m_hasW1 ? p : 0) + m + (m_hasW3 ? p : 0);
    if (na > MAX_AUG || nw > MAX_AUG || nz > MAX_AUG) return;

    m_na = na;
    m_nw = nw; m_nu = m; m_nz = nz; m_ny = p;

    const int off1 = n, off2 = n + n1, off3 = n + n1 + n2;
    int z1 = 0, z2 = m_hasW1 ? p : 0, z3 = z2 + m;

    auto Aa = [&](int i, int j) -> double& { return m_Aa[i * na + j]; };
    auto B1 = [&](int i, int j) -> double& { return m_B1a[i * nw + j]; };
    auto B2 = [&](int i, int j) -> double& { return m_B2a[i * m + j]; };
    auto C1 = [&](int i, int j) -> double& { return m_C1a[i * na + j]; };
    auto C2 = [&](int i, int j) -> double& { return m_C2a[i * na + j]; };
    auto D11 = [&](int i, int j) -> double& { return m_D11a[i * nw + j]; };
    auto D12 = [&](int i, int j) -> double& { return m_D12a[i * m + j]; };
    auto D21 = [&](int i, int j) -> double& { return m_D21a[i * nw + j]; };
    auto D22 = [&](int i, int j) -> double& { return m_D22a[i * m + j]; };
    (void)D11; (void)D22;  // remain zero by construction

    std::fill(m_Aa.begin(), m_Aa.end(), 0.0);
    std::fill(m_B1a.begin(), m_B1a.end(), 0.0);
    std::fill(m_B2a.begin(), m_B2a.end(), 0.0);
    std::fill(m_C1a.begin(), m_C1a.end(), 0.0);
    std::fill(m_C2a.begin(), m_C2a.end(), 0.0);
    std::fill(m_D11a.begin(), m_D11a.end(), 0.0);
    std::fill(m_D12a.begin(), m_D12a.end(), 0.0);
    std::fill(m_D21a.begin(), m_D21a.end(), 0.0);
    std::fill(m_D22a.begin(), m_D22a.end(), 0.0);

    // plant dynamics and I/O
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) Aa(i, j) = m_A[i * m_n + j];
        for (int j = 0; j < m; ++j) {
            B2(i, j) = m_B[i * m_m + j];
            B1(i, p + j) = m_B[i * m_m + j];  // input disturbance d
        }
    }
    for (int i = 0; i < p; ++i) {
        for (int j = 0; j < n; ++j) C2(i, j) = -m_C[i * m_n + j];  // ym = r - y + n
        D21(i, i) = 1.0;                    // r -> ym
        D21(i, p + m + i) = 1.0;            // n -> ym
    }

    // W1: input e_j = r_j - C_j x ; output z1_j = W1 e_j  (Dw1 forced to 0)
    if (m_hasW1) {
        for (int j = 0; j < p; ++j) {
            int base = off1 + j * o1;
            for (int a = 0; a < o1; ++a) {
                for (int b = 0; b < o1; ++b) Aa(base + a, base + b) = w1.A[a * o1 + b];
                B1(base + a, j) = w1.B[a];                    // r_j -> e_j
                for (int i = 0; i < n; ++i)
                    Aa(base + a, i) = -w1.B[a] * m_C[j * m_n + i];  // -C_j x -> e_j
                C1(z1 + j, base + a) = w1.C[a];
            }
        }
    }

    // W2: input u_j ; output z2_j = W2 u_j (always present; default static
    // penalty when the user did not configure one)
    {
        for (int j = 0; j < m; ++j) {
            int base = off2 + j * o2;
            for (int a = 0; a < o2; ++a) {
                for (int b = 0; b < o2; ++b) Aa(base + a, base + b) = w2.A[a * o2 + b];
                B2(base + a, j) = w2.B[a];
                C1(z2 + j, base + a) = w2.C[a];
            }
            D12(z2 + j, j) = w2.D;
        }
    }

    // W3: input y_j = C_j x ; output z3_j = W3 y_j
    if (m_hasW3) {
        for (int j = 0; j < p; ++j) {
            int base = off3 + j * o3;
            for (int a = 0; a < o3; ++a) {
                for (int b = 0; b < o3; ++b) Aa(base + a, base + b) = w3.A[a * o3 + b];
                for (int i = 0; i < n; ++i)
                    Aa(base + a, i) = w3.B[a] * m_C[j * m_n + i];
                C1(z3 + j, base + a) = w3.C[a];
            }
            for (int i = 0; i < n; ++i)
                C1(z3 + j, i) += w3.D * m_C[j * m_n + i];
        }
    }
}

bool HInfinityController::isAchievable(double gamma) {
    buildAugmentedPlant();
    if (m_na <= 0) return false;
    std::array<double, MAX_AUG * MAX_AUG> X{}, Y{};
    return solveHinfRiccati(gamma, X.data(), Y.data());
}

bool HInfinityController::solveHinfRiccati(double gamma, double* X, double* Y) {
    // DGKF two-Riccati solution for the standard H-infinity problem
    // (D11 = D22 = 0, D12 full column rank, D21 full row rank).
    //
    //   xdot = A x + B1 w + B2 u
    //   z    = C1 x + D12 u
    //   y    = C2 x + D21 w
    //
    // With R = D12'D12 and Rt = D21 D21', the two game-type AREs are
    //
    //   (A - B2 R^-1 D12'C1)'X + X(A - B2 R^-1 D12'C1)
    //       + C1'(I - D12 R^-1 D12')C1
    //       + X(g^-2 B1 B1' - B2 R^-1 B2')X = 0                    (control)
    //
    //   (A - B1 D21' Rt^-1 C2)Y + Y(A - B1 D21' Rt^-1 C2)'
    //       + B1(I - D21' Rt^-1 D21)B1'
    //       + Y(g^-2 C1'C1 - C2' Rt^-1 C2)Y = 0                    (filter)
    //
    // Feasibility: both have stabilizing PSD solutions and rho(XY) < g^2.
    const int n = m_na, nw = m_nw, nu = m_nu, nz = m_nz, ny = m_ny;
    if (n <= 0 || nw <= 0 || nu <= 0 || nz <= 0 || ny <= 0 || gamma <= 0.0)
        return false;

    SpMat A   = loadMat(m_Aa.data(), n, n);
    SpMat B1  = loadMat(m_B1a.data(), n, nw);
    SpMat B2  = loadMat(m_B2a.data(), n, nu);
    SpMat C1  = loadMat(m_C1a.data(), nz, n);
    SpMat C2  = loadMat(m_C2a.data(), ny, n);
    SpMat D11 = loadMat(m_D11a.data(), nz, nw);
    SpMat D12 = loadMat(m_D12a.data(), nz, nu);
    SpMat D21 = loadMat(m_D21a.data(), ny, nw);
    SpMat D22 = loadMat(m_D22a.data(), ny, nu);

    // the two-Riccati solution requires the standard-problem assumptions
    double dScale = std::max({D11.cwiseAbs().maxCoeff(), D22.cwiseAbs().maxCoeff(), 1.0});
    if (D11.cwiseAbs().maxCoeff() > 1e-9 * dScale ||
        D22.cwiseAbs().maxCoeff() > 1e-9 * dScale)
        return false;

    SpMat R = D12.transpose() * D12;    // nu x nu
    SpMat Rt = D21 * D21.transpose();   // ny x ny
    SpMat Rinv, Rtinv;
    if (!safeInverse(R, Rinv) || !safeInverse(Rt, Rtinv)) return false;

    const double g2 = gamma * gamma;
    SpMat I_n  = SpMat::Identity(n, n);
    SpMat I_nz = SpMat::Identity(nz, nz);
    SpMat I_nw = SpMat::Identity(nw, nw);

    // ---- control ARE for X ----
    SpMat Abar_x = A - B2 * Rinv * (D12.transpose() * C1);
    SpMat Qx = C1.transpose() * (I_nz - D12 * Rinv * D12.transpose()) * C1;
    Qx = 0.5 * (Qx + Qx.transpose());
    SpMat Sx = B1 * B1.transpose() / g2 - B2 * Rinv * B2.transpose();
    Sx = 0.5 * (Sx + Sx.transpose());
    SpMat Xsol;
    if (!solveGameCARE(Abar_x, Sx, Qx, Xsol)) return false;

    // ---- filter ARE for Y ----
    SpMat Abar_y = A - B1 * (D21.transpose() * Rtinv) * C2;
    SpMat Qy = B1 * (I_nw - D21.transpose() * Rtinv * D21) * B1.transpose();
    Qy = 0.5 * (Qy + Qy.transpose());
    SpMat Sy = C1.transpose() * C1 / g2 - C2.transpose() * Rtinv * C2;
    Sy = 0.5 * (Sy + Sy.transpose());
    SpMat Ysol;
    if (!solveGameCARE(Abar_y.transpose(), Sy, Qy, Ysol)) return false;

    // coupling condition rho(X Y) < gamma^2
    Eigen::EigenSolver<SpMat> xy(Xsol * Ysol);
    if (xy.info() != Eigen::Success) return false;
    double rho = 0.0;
    for (int i = 0; i < n; ++i) rho = std::max(rho, std::abs(xy.eigenvalues()[i]));
    if (!(rho < g2)) return false;

    storeMat(X, n, n, Xsol);
    storeMat(Y, n, n, Ysol);
    return true;
}

void HInfinityController::computeController(double gamma, const double* X, const double* Y) {
    // DGKF central controller (continuous time):
    //   F  = -R^-1 (B2'X + D12'C1)          state-feedback gain
    //   L  = -(Y C2' + B1 D21') Rt^-1      observer gain
    //   Z  = (I - g^-2 Y X)^-1             coupling correction
    //   Ak = A + g^-2 B1 B1' X + B2 F + Z L C2
    //   Bk = Z L ,  Ck = F ,  Dk = 0
    const int n = m_na, nw = m_nw, nu = m_nu, nz = m_nz, ny = m_ny;
    m_nk = n;

    SpMat A   = loadMat(m_Aa.data(), n, n);
    SpMat B1  = loadMat(m_B1a.data(), n, nw);
    SpMat B2  = loadMat(m_B2a.data(), n, nu);
    SpMat C1  = loadMat(m_C1a.data(), nz, n);
    SpMat C2  = loadMat(m_C2a.data(), ny, n);
    SpMat D12 = loadMat(m_D12a.data(), nz, nu);
    SpMat D21 = loadMat(m_D21a.data(), ny, nw);
    SpMat Xm  = loadMat(X, n, n);
    SpMat Ym  = loadMat(Y, n, n);

    SpMat Rinv = (D12.transpose() * D12).inverse();
    SpMat Rtinv = (D21 * D21.transpose()).inverse();
    const double g2 = gamma * gamma;

    SpMat F = -Rinv * (B2.transpose() * Xm + D12.transpose() * C1);
    SpMat L = -(Ym * C2.transpose() + B1 * D21.transpose()) * Rtinv;
    SpMat Z = (SpMat::Identity(n, n) - Ym * Xm / g2).inverse();

    SpMat Ak = A + B1 * B1.transpose() * Xm / g2 + B2 * F + Z * L * C2;
    SpMat Bk = Z * L;
    SpMat Ck = -F;
    SpMat Dk = SpMat::Zero(nu, ny);

    storeMat(m_Ak.data(), n, n, Ak);
    storeMat(m_Bk.data(), n, ny, Bk);
    storeMat(m_Ck.data(), nu, n, Ck);
    storeMat(m_Dk.data(), nu, ny, Dk);
}

bool HInfinityController::design(double gamma) {
    buildAugmentedPlant();
    if (m_na <= 0) return false;

    std::array<double, MAX_AUG * MAX_AUG> X{}, Y{};
    if (!solveHinfRiccati(gamma, X.data(), Y.data())) {
        return false;
    }

    computeController(gamma, X.data(), Y.data());

    m_gamma = gamma;
    m_designed = true;
    m_dtLast = -1.0;

    return true;
}

double HInfinityController::designOptimal(double tol) {
    buildAugmentedPlant();
    if (m_na <= 0) return -1.0;

    // Achievability is monotone: feasible for all gamma > gamma_min.
    // Bracket gamma_min, then bisect.
    double gammaLow = 0.5;
    double gammaHigh = 2.0;

    int guard = 0;
    while (!isAchievable(gammaHigh) && guard++ < 80) gammaHigh *= 2.0;
    if (!isAchievable(gammaHigh)) return -1.0;

    guard = 0;
    while (isAchievable(gammaLow) && guard++ < 80) gammaLow *= 0.5;
    if (gammaLow < 1e-9) gammaLow = 0.0;

    guard = 0;
    while (gammaHigh - gammaLow > tol * std::max(1.0, gammaHigh) && guard++ < 100) {
        double gammaMid = 0.5 * (gammaLow + gammaHigh);
        if (isAchievable(gammaMid)) {
            gammaHigh = gammaMid;
        } else {
            gammaLow = gammaMid;
        }
    }

    if (!design(gammaHigh)) return -1.0;
    return gammaHigh;
}

void HInfinityController::getControllerMatrices(double* Ak, double* Bk, 
                                                 double* Ck, double* Dk) const {
    if (Ak) storeMat(Ak, m_nk, m_nk, loadMat(m_Ak.data(), m_nk, m_nk));
    if (Bk) storeMat(Bk, m_nk, m_ny, loadMat(m_Bk.data(), m_nk, m_ny));
    if (Ck) storeMat(Ck, m_nu, m_nk, loadMat(m_Ck.data(), m_nu, m_nk));
    if (Dk) storeMat(Dk, m_nu, m_ny, loadMat(m_Dk.data(), m_nu, m_ny));
}

void HInfinityController::getAugmentedPlant(double* A, double* B1, double* B2,
                                            double* C1, double* C2, double* D11,
                                            double* D12, double* D21, double* D22) const {
    const int n = m_na, nw = m_nw, nu = m_nu, nz = m_nz, ny = m_ny;
    if (A)   storeMat(A, n, n, loadMat(m_Aa.data(), n, n));
    if (B1)  storeMat(B1, n, nw, loadMat(m_B1a.data(), n, nw));
    if (B2)  storeMat(B2, n, nu, loadMat(m_B2a.data(), n, nu));
    if (C1)  storeMat(C1, nz, n, loadMat(m_C1a.data(), nz, n));
    if (C2)  storeMat(C2, ny, n, loadMat(m_C2a.data(), ny, n));
    if (D11) storeMat(D11, nz, nw, loadMat(m_D11a.data(), nz, nw));
    if (D12) storeMat(D12, nz, nu, loadMat(m_D12a.data(), nz, nu));
    if (D21) storeMat(D21, ny, nw, loadMat(m_D21a.data(), ny, nw));
    if (D22) storeMat(D22, ny, nu, loadMat(m_D22a.data(), ny, nu));
}

void HInfinityController::discretizeController(double dt) {
    // Tustin (bilinear) discretization of the continuous-time controller:
    //   Ad = (I - dt/2 A)^-1 (I + dt/2 A),  Bd = (I - dt/2 A)^-1 dt B
    const int n = m_nk, ny = m_ny;
    SpMat A = loadMat(m_Ak.data(), n, n);
    SpMat B = loadMat(m_Bk.data(), n, ny);
    SpMat T = 0.5 * dt * SpMat::Identity(n, n);
    SpMat M = SpMat::Identity(n, n) - T * A;
    SpMat Ad = M.partialPivLu().solve(SpMat::Identity(n, n) + T * A);
    SpMat Bd = M.partialPivLu().solve(dt * B);
    if (!Ad.allFinite() || !Bd.allFinite()) {
        // fallback: explicit Euler
        Ad = SpMat::Identity(n, n) + dt * A;
        Bd = dt * B;
    }
    storeMat(m_Adk.data(), n, n, Ad);
    storeMat(m_Bdk.data(), n, ny, Bd);
}

ControllerOutput HInfinityController::computeImpl(const ControllerInput& input) {
    ControllerOutput output;

    if (!m_designed || m_nk <= 0) {
        output.control = 0;
        return output;
    }

    const double dt = input.dt > 0.0 ? input.dt : 1e-3;
    if (dt != m_dtLast) {
        discretizeController(dt);
        m_dtLast = dt;
    }

    // controller measurement y: SISO -> tracking error (reference - measured);
    // MIMO -> caller supplies the measurement vector in input.state
    double y[MAX_OUTPUT_DIM]{};
    if (m_ny == 1) {
        y[0] = input.reference - input.measured;
    } else {
        for (int i = 0; i < m_ny; ++i)
            y[i] = (i < static_cast<int>(input.stateDim)) ? input.state[i] : 0.0;
    }

    // xk+ = Adk xk + Bdk y
    double xn[MAX_AUG]{};
    for (int i = 0; i < m_nk; ++i) {
        double s = 0.0;
        for (int j = 0; j < m_nk; ++j) s += m_Adk[i * m_nk + j] * m_xk[j];
        for (int j = 0; j < m_ny; ++j) s += m_Bdk[i * m_ny + j] * y[j];
        xn[i] = s;
    }
    for (int i = 0; i < m_nk; ++i) m_xk[i] = xn[i];

    // u = Ck xk + Dk y
    double u[MAX_CONTROL_DIM]{};
    for (int i = 0; i < m_nu; ++i) {
        double s = 0.0;
        for (int j = 0; j < m_nk; ++j) s += m_Ck[i * m_nk + j] * m_xk[j];
        for (int j = 0; j < m_ny; ++j) s += m_Dk[i * m_ny + j] * y[j];
        u[i] = s;
    }

    output.control = std::clamp(u[0], m_limits.outputMin, m_limits.outputMax);
    for (int i = 0; i < m_nu && i < static_cast<int>(MAX_CONTROL_DIM); ++i)
        output.controlVector[i] = u[i];
    output.error = y[0];
    output.saturated = (u[0] != output.control);

    return output;
}

void HInfinityController::resetImpl() {
    std::fill(m_xk.begin(), m_xk.end(), 0.0);
    m_dtLast = -1.0;
}

// ============================================================================
// μ-Synthesis Framework
// ============================================================================

void MuSynthesisFramework::setUncertaintyStructure(const int* blockSizes, int numBlocks,
                                                    const bool* repeated) {
    m_numBlocks = std::min(numBlocks, 10);
    for (int i = 0; i < m_numBlocks; i++) {
        m_blockSizes[i] = blockSizes[i];
        m_repeated[i] = repeated ? repeated[i] : false;
    }
    for (int i = m_numBlocks; i < 10; ++i) {
        m_blockSizes[i] = 0;
        m_repeated[i] = false;
    }
    m_nDPar = 0;
    m_dX.fill(0.0);
}

double MuSynthesisFramework::computeMuUpperBound(const double* M, int n) const {
    if (n <= 0 || !M) return std::numeric_limits<double>::infinity();
    const SpMat Mm = loadMat(M, n, n);

    MuDParams mp = buildMuParams(m_blockSizes.data(), m_repeated.data(), m_numBlocks);
    if (mp.nPos != n || mp.nPar == 0)
        return sigmaBarPlain(Mm);   // unstructured bound: still >= mu

    optimizeMuScales(mp, [&](const MuDParams& p) {
        return sigmaBarScaledReal(Mm, p);
    });
    return sigmaBarScaledReal(Mm, mp);
}

double MuSynthesisFramework::computeMuUpperBound(const std::complex<double>* M, int n) const {
    if (n <= 0 || !M) return std::numeric_limits<double>::infinity();
    SpCmat Mm(n, n);
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            Mm(i, j) = M[i * n + j];

    MuDParams mp = buildMuParams(m_blockSizes.data(), m_repeated.data(), m_numBlocks);
    if (mp.nPos != n || mp.nPar == 0) {
        Eigen::JacobiSVD<SpCmat> svd(Mm);
        return (svd.info() == Eigen::Success && svd.singularValues().size() > 0)
               ? svd.singularValues()(0)
               : std::numeric_limits<double>::infinity();
    }

    optimizeMuScales(mp, [&](const MuDParams& p) {
        return sigmaBarScaledComplex(Mm, p);
    });
    return sigmaBarScaledComplex(Mm, mp);
}

void MuSynthesisFramework::setGeneralizedPlant(const double* A, const double* B1,
                                                const double* B2, const double* C1,
                                                const double* C2, const double* D11,
                                                const double* D12, const double* D21,
                                                const double* D22,
                                                int n, int nw, int nu, int nz, int ny,
                                                int nUncW, int nUncZ) {
    m_n = std::min(n, MAX_MU_N);
    m_nw = std::min(nw, MAX_MU_N);
    m_nu = std::min(nu, static_cast<int>(MAX_CONTROL_DIM));
    m_nz = std::min(nz, static_cast<int>(MAX_OUTPUT_DIM));
    m_ny = std::min(ny, static_cast<int>(MAX_OUTPUT_DIM));
    m_nUncW = nUncW;
    m_nUncZ = nUncZ;
    m_hasPlant = (n <= MAX_MU_N && nw <= MAX_MU_N &&
                  nu <= static_cast<int>(MAX_CONTROL_DIM) &&
                  nz <= static_cast<int>(MAX_OUTPUT_DIM) &&
                  ny <= static_cast<int>(MAX_OUTPUT_DIM));

    std::memset(m_A.data(), 0, m_A.size() * sizeof(double));
    std::memset(m_B1.data(), 0, m_B1.size() * sizeof(double));
    std::memset(m_B2.data(), 0, m_B2.size() * sizeof(double));
    std::memset(m_C1.data(), 0, m_C1.size() * sizeof(double));
    std::memset(m_C2.data(), 0, m_C2.size() * sizeof(double));
    std::memset(m_D11.data(), 0, m_D11.size() * sizeof(double));
    std::memset(m_D12.data(), 0, m_D12.size() * sizeof(double));
    std::memset(m_D21.data(), 0, m_D21.size() * sizeof(double));
    std::memset(m_D22.data(), 0, m_D22.size() * sizeof(double));

    if (m_hasPlant) {
        std::memcpy(m_A.data(),  A,  static_cast<size_t>(m_n) * m_n * sizeof(double));
        std::memcpy(m_B1.data(), B1, static_cast<size_t>(m_n) * m_nw * sizeof(double));
        std::memcpy(m_B2.data(), B2, static_cast<size_t>(m_n) * m_nu * sizeof(double));
        std::memcpy(m_C1.data(), C1, static_cast<size_t>(m_nz) * m_n * sizeof(double));
        std::memcpy(m_C2.data(), C2, static_cast<size_t>(m_ny) * m_n * sizeof(double));
        if (D11) std::memcpy(m_D11.data(), D11, static_cast<size_t>(m_nz) * m_nw * sizeof(double));
        if (D12) std::memcpy(m_D12.data(), D12, static_cast<size_t>(m_nz) * m_nu * sizeof(double));
        if (D21) std::memcpy(m_D21.data(), D21, static_cast<size_t>(m_ny) * m_nw * sizeof(double));
        if (D22) std::memcpy(m_D22.data(), D22, static_cast<size_t>(m_ny) * m_nu * sizeof(double));
    }
    m_hasK = false;
    m_iter = 0;
    m_bound = 0.0;
    m_converged = false;
    m_nDPar = 0;
    m_dX.fill(0.0);
}

void MuSynthesisFramework::setFrequencyGrid(double wMin, double wMax, int nPoints) {
    m_wMin = std::max(1e-6, wMin);
    m_wMax = std::max(m_wMin * 10.0, wMax);
    m_nGrid = std::min(nPoints, MAX_GRID);
    if (m_nGrid <= 0) { m_nGrid = 0; return; }
    const double lmin = std::log(m_wMin), lmax = std::log(m_wMax);
    for (int i = 0; i < m_nGrid; ++i)
        m_freqs[i] = std::exp(lmin + (lmax - lmin) * i / (m_nGrid - 1));
}

bool MuSynthesisFramework::dkIteration() {
    if (!m_hasPlant || m_n <= 0) return false;
    const int d = m_nUncW;
    if (d <= 0 || d != m_nUncZ || d > static_cast<int>(MAX_OUTPUT_DIM))
        return false;

    // rebuild the D parametrization; restore previous log-scales (the
    // parameter order produced by buildMuParams is deterministic)
    MuDParams mp = buildMuParams(m_blockSizes.data(), m_repeated.data(), m_numBlocks);
    if (mp.nPos != d || mp.nPar == 0 || mp.nPar > MAX_MU_D) return false;
    for (int p = 0; p < mp.nPar; ++p)
        mp.x[p] = (p < m_nDPar) ? m_dX[p] : 0.0;
    m_nDPar = mp.nPar;
    for (int p = 0; p < m_nDPar; ++p) m_dX[p] = mp.x[p];

    const int n = m_n, nw = m_nw, nu = m_nu, nz = m_nz, ny = m_ny;

    // ---------- K-step: H-infinity synthesis on the D-scaled plant ------
    // The scaled plant produces T~ = S_z T S_w on the uncertainty channels,
    // i.e. the w_D -> z_D block becomes D M D^-1, which is what we minimize.
    {
        SpMat A   = loadMat(m_A.data(),  n,  n);
        SpMat B1  = loadMat(m_B1.data(), n,  nw);
        SpMat B2  = loadMat(m_B2.data(), n,  nu);
        SpMat C1  = loadMat(m_C1.data(), nz, n);
        SpMat C2  = loadMat(m_C2.data(), ny, n);
        SpMat D11 = loadMat(m_D11.data(),nz, nw);
        SpMat D12 = loadMat(m_D12.data(),nz, nu);
        SpMat D21 = loadMat(m_D21.data(),ny, nw);
        SpMat D22 = loadMat(m_D22.data(),ny, nu);

        const int w0 = nw - d, z0 = nz - d;
        for (int j = 0; j < d; ++j) {
            const double s  = std::exp(mp.x[mp.posPar[j]]);
            const double si = 1.0 / s;
            // input side (w_D): scale by D^-1
            B1.col(w0 + j)  *= si;
            D11.col(w0 + j) *= si;
            D21.col(w0 + j) *= si;
            // output side (z_D): scale by D
            C1.row(z0 + j)  *= s;
            D11.row(z0 + j) *= s;
            D12.row(z0 + j) *= s;
        }

        std::vector<double> bA(n*n), bB1(n*nw), bB2(n*nu), bC1(nz*n), bC2(ny*n),
                            bD11(nz*nw), bD12(nz*nu), bD21(ny*nw), bD22(ny*nu);
        storeMat(bA.data(),  n,  n,  A);
        storeMat(bB1.data(), n,  nw, B1);
        storeMat(bB2.data(), n,  nu, B2);
        storeMat(bC1.data(), nz, n,  C1);
        storeMat(bC2.data(), ny, n,  C2);
        storeMat(bD11.data(),nz, nw, D11);
        storeMat(bD12.data(),nz, nu, D12);
        storeMat(bD21.data(),ny, nw, D21);
        storeMat(bD22.data(),ny, nu, D22);

        HInfinityController hinf;
        hinf.setGeneralizedPlant(bA.data(), bB1.data(), bB2.data(),
                                 bC1.data(), bC2.data(), bD11.data(),
                                 bD12.data(), bD21.data(), bD22.data(),
                                 n, nw, nu, nz, ny);
        const double gamma = hinf.designOptimal(0.02);
        if (gamma < 0.0) return false;

        std::vector<double> cAk(n*n), cBk(n*ny), cCk(nu*n), cDk(nu*ny);
        hinf.getControllerMatrices(cAk.data(), cBk.data(), cCk.data(), cDk.data());
        m_nk = n;
        storeMat(m_Ak.data(), n, n,  loadMat(cAk.data(), n, n));
        storeMat(m_Bk.data(), n, ny, loadMat(cBk.data(), n, ny));
        storeMat(m_Ck.data(), nu, n, loadMat(cCk.data(), nu, n));
        storeMat(m_Dk.data(), nu, ny, loadMat(cDk.data(), nu, ny));
        m_hasK = true;
    }

    // ---------- D-step: refit static D against M(jw) on the grid --------
    if (m_nGrid == 0) {
        // default grid if the user never configured one
        m_wMin = 1e-1; m_wMax = 1e3;
        setFrequencyGrid(m_wMin, m_wMax, 33);
    }

    // closed loop of the ORIGINAL (unscaled) plant with the new K
    SpMat A   = loadMat(m_A.data(),  n,  n);
    SpMat B1  = loadMat(m_B1.data(), n,  nw);
    SpMat B2  = loadMat(m_B2.data(), n,  nu);
    SpMat C1  = loadMat(m_C1.data(), nz, n);
    SpMat C2  = loadMat(m_C2.data(), ny, n);
    SpMat D11 = loadMat(m_D11.data(),nz, nw);
    SpMat D12 = loadMat(m_D12.data(),nz, nu);
    SpMat D21 = loadMat(m_D21.data(),ny, nw);
    SpMat D22 = loadMat(m_D22.data(),ny, nu);
    SpMat Ak  = loadMat(m_Ak.data(), n, n);
    SpMat Bk  = loadMat(m_Bk.data(), n, ny);
    SpMat Ck  = loadMat(m_Ck.data(), nu, n);
    SpMat Dk  = loadMat(m_Dk.data(), nu, ny);

    SpMat Acl, Bcl, Cz, Dzw;
    if (!buildClosedLoop(A, B1, B2, C1, C2, D11, D12, D21, D22,
                         Ak, Bk, Ck, Dk, Acl, Bcl, Cz, Dzw))
        return false;
    if (!isHurwitz(Acl)) return false;

    // M(jw) = z_D x w_D block of the closed-loop transfer, per grid point
    SpCmat Aclc = Acl.cast<std::complex<double>>();
    SpCmat BclD = Bcl.rightCols(d).cast<std::complex<double>>();
    SpCmat CzD  = Cz.bottomRows(d);
    SpCmat DzwD = Dzw.bottomRightCorner(d, d);
    std::vector<SpCmat> Ms(m_nGrid);
    for (int g = 0; g < m_nGrid; ++g) {
        const double w = m_freqs[g];
        SpCmat T = -Aclc;
        T.diagonal().array() += std::complex<double>(0.0, w);
        SpCmat X = T.partialPivLu().solve(BclD);
        Ms[g] = CzD * X + DzwD;
    }

    // minimize max_w sigma_bar(D M_w D^-1) over the static log-scales
    MuDParams dp;
    dp.nPos = d; dp.nPar = mp.nPar;
    dp.posPar.assign(mp.posPar.begin(), mp.posPar.begin() + d);
    dp.x = mp.x;
    optimizeMuScales(dp, [&](const MuDParams& p) {
        double mx = 0.0;
        for (const SpCmat& Mwj : Ms)
            mx = std::max(mx, sigmaBarScaledComplex(Mwj, p));
        return mx;
    });

    // write back scales and evaluate the achieved bound
    for (int p = 0; p < mp.nPar; ++p) { m_dX[p] = dp.x[p]; mp.x[p] = dp.x[p]; }
    double bound = 0.0;
    for (const SpCmat& Mwj : Ms)
        bound = std::max(bound, sigmaBarScaledComplex(Mwj, mp));
    m_bound = bound;
    ++m_iter;
    return true;
}

int MuSynthesisFramework::dkIterate(int maxIter, double tol) {
    m_iter = 0;
    m_converged = false;
    double prev = std::numeric_limits<double>::infinity();
    for (int i = 0; i < maxIter; ++i) {
        if (!dkIteration()) break;
        if (std::isfinite(prev) &&
            std::abs(prev - m_bound) <= tol * std::max(1.0, m_bound)) {
            m_converged = true;
            break;
        }
        prev = m_bound;
    }
    return m_iter;
}

void MuSynthesisFramework::getControllerMatrices(double* Ak, double* Bk,
                                                  double* Ck, double* Dk,
                                                  int& nk) const {
    nk = m_hasK ? m_nk : 0;
    if (!m_hasK) return;
    if (Ak) std::memcpy(Ak, m_Ak.data(), static_cast<size_t>(m_nk) * m_nk * sizeof(double));
    if (Bk) std::memcpy(Bk, m_Bk.data(), static_cast<size_t>(m_nk) * m_ny * sizeof(double));
    if (Ck) std::memcpy(Ck, m_Ck.data(), static_cast<size_t>(m_nu) * m_nk * sizeof(double));
    if (Dk) std::memcpy(Dk, m_Dk.data(), static_cast<size_t>(m_nu) * m_ny * sizeof(double));
}

} // namespace tether::control
