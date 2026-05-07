/**
 * @file QFT.cpp
 * @brief Implementation of Quantitative Feedback Theory (QFT) Controller
 */

#include "tether/control/autotuning/QFT.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace Control {

// ============================================================================
// NicholsPoint Implementation
// ============================================================================

NicholsPoint NicholsPoint::fromComplex(std::complex<double> z) {
    NicholsPoint pt;
    pt.gain = 20.0 * std::log10(std::abs(z) + 1e-15);
    pt.phase = std::arg(z) * 180.0 / M_PI;
    return pt;
}

std::complex<double> NicholsPoint::toComplex() const {
    double mag = std::pow(10.0, gain / 20.0);
    double phaseRad = phase * M_PI / 180.0;
    return std::polar(mag, phaseRad);
}

// ============================================================================
// PlantTemplate Implementation
// ============================================================================

std::vector<NicholsPoint> PlantTemplate::boundary() const {
    if (points.empty()) return {};
    
    // Simple convex hull using gift wrapping for small point sets
    return TemplateUtils::convexHull(points);
}

bool PlantTemplate::contains(const NicholsPoint& point) const {
    if (points.empty()) return false;
    
    // Point-in-polygon test using winding number
    auto hull = boundary();
    if (hull.size() < 3) return false;
    
    int winding = 0;
    for (size_t i = 0; i < hull.size(); i++) {
        size_t j = (i + 1) % hull.size();
        
        if (hull[i].phase <= point.phase) {
            if (hull[j].phase > point.phase) {
                // Upward crossing
                double cross = (hull[j].gain - hull[i].gain) * (point.phase - hull[i].phase) -
                               (hull[j].phase - hull[i].phase) * (point.gain - hull[i].gain);
                if (cross > 0) winding++;
            }
        } else {
            if (hull[j].phase <= point.phase) {
                // Downward crossing
                double cross = (hull[j].gain - hull[i].gain) * (point.phase - hull[i].phase) -
                               (hull[j].phase - hull[i].phase) * (point.gain - hull[i].gain);
                if (cross < 0) winding--;
            }
        }
    }
    
    return winding != 0;
}

// ============================================================================
// QFTBound Implementation
// ============================================================================

bool QFTBound::isSatisfied(const NicholsPoint& loopPoint) const {
    // The bound defines a forbidden region
    // Loop point satisfies bound if it's NOT inside the forbidden region
    
    if (boundary.empty()) return true;
    
    // Simple point-in-polygon test
    int crossings = 0;
    for (size_t i = 0; i < boundary.size(); i++) {
        size_t j = (i + 1) % boundary.size();
        
        if ((boundary[i].phase <= loopPoint.phase && boundary[j].phase > loopPoint.phase) ||
            (boundary[j].phase <= loopPoint.phase && boundary[i].phase > loopPoint.phase)) {
            double t = (loopPoint.phase - boundary[i].phase) / (boundary[j].phase - boundary[i].phase);
            double intersectGain = boundary[i].gain + t * (boundary[j].gain - boundary[i].gain);
            if (loopPoint.gain < intersectGain) {
                crossings++;
            }
        }
    }
    
    // Point is inside if odd number of crossings
    return (crossings % 2) == 0;
}

// ============================================================================
// TransferFunction Implementation
// ============================================================================

std::complex<double> TransferFunction::evaluate(double omega) const {
    std::complex<double> s(0, omega);
    
    // Evaluate numerator polynomial
    std::complex<double> numVal(0, 0);
    std::complex<double> sn(1, 0);
    for (int i = num.size() - 1; i >= 0; i--) {
        numVal += num[i] * sn;
        sn *= s;
    }
    
    // Evaluate denominator polynomial
    std::complex<double> denVal(0, 0);
    sn = std::complex<double>(1, 0);
    for (int i = den.size() - 1; i >= 0; i--) {
        denVal += den[i] * sn;
        sn *= s;
    }
    
    if (std::abs(denVal) < 1e-15) return std::complex<double>(1e15, 0);
    return numVal / denVal;
}

TransferFunction TransferFunction::fromZPK(const std::vector<std::complex<double>>& zeros,
                                            const std::vector<std::complex<double>>& poles,
                                            double gain) {
    TransferFunction tf;
    
    // For simplicity, handle up to 4th order
    // Numerator from zeros
    tf.num = {gain};
    for (const auto& z : zeros) {
        std::vector<double> newNum(tf.num.size() + 1, 0);
        for (size_t i = 0; i < tf.num.size(); i++) {
            newNum[i] += tf.num[i];
            newNum[i + 1] -= tf.num[i] * z.real();  // Simplified for real zeros
        }
        tf.num = newNum;
    }
    
    // Denominator from poles
    tf.den = {1.0};
    for (const auto& p : poles) {
        std::vector<double> newDen(tf.den.size() + 1, 0);
        for (size_t i = 0; i < tf.den.size(); i++) {
            newDen[i] += tf.den[i];
            newDen[i + 1] -= tf.den[i] * p.real();
        }
        tf.den = newDen;
    }
    
    return tf;
}

// ============================================================================
// QFTController Implementation - Plant Setup
// ============================================================================

void QFTController::setNominalPlant(double K, double tau, double L) {
    // FOPDT: G(s) = K * exp(-Ls) / (τs + 1)
    // Approximate delay with Padé: exp(-Ls) ≈ (1 - Ls/2) / (1 + Ls/2)
    
    m_nominalPlant.num = {K * (1.0 - L/2.0)};
    m_nominalPlant.den = {tau * (1.0 + L/2.0), (1.0 + L/2.0)};
    m_useFOPDT = true;
}

void QFTController::setNominalPlant(double K, double tau1, double tau2, double L) {
    // SOPDT: G(s) = K * exp(-Ls) / ((τ1s+1)(τ2s+1))
    m_nominalPlant.num = {K};
    m_nominalPlant.den = {tau1 * tau2, tau1 + tau2, 1.0};
    m_useFOPDT = false;
}

void QFTController::setNominalPlant(const TransferFunction& tf) {
    m_nominalPlant = tf;
    m_useFOPDT = false;
}

void QFTController::setNominalPlant(const double* A, const double* B,
                                     const double* C, const double* D,
                                     int n, int m, int p) {
    m_n = n;
    m_m = m;
    m_p = p;
    
    for (int i = 0; i < n * n; i++) m_A[i] = A[i];
    for (int i = 0; i < n * m; i++) m_B[i] = B[i];
    for (int i = 0; i < p * n; i++) m_C[i] = C[i];
    for (int i = 0; i < p * m; i++) m_D[i] = D[i];
    
    m_useFOPDT = false;
}

void QFTController::setFOPDTUncertainty(double Kmin, double Kmax,
                                         double tauMin, double tauMax,
                                         double Lmin, double Lmax) {
    m_fopdtBounds.Kmin = Kmin;
    m_fopdtBounds.Kmax = Kmax;
    m_fopdtBounds.tauMin = tauMin;
    m_fopdtBounds.tauMax = tauMax;
    m_fopdtBounds.Lmin = Lmin;
    m_fopdtBounds.Lmax = Lmax;
    m_useFOPDT = true;
}

void QFTController::addParametricUncertainty(int paramIndex, double minValue,
                                              double maxValue, int numSamples) {
    // Store parametric uncertainty for template generation
    // This is a placeholder - full implementation would track these
}

void QFTController::setMultiplicativeUncertainty(const TransferFunction& W) {
    // Store multiplicative uncertainty weight
    // Full implementation would use this for template generation
}

void QFTController::setPlantSamples(const std::vector<TransferFunction>& plants) {
    m_plantSamples = plants;
}

// ============================================================================
// QFTController Implementation - Specifications
// ============================================================================

void QFTController::setTrackingSpec(const std::vector<double>& freqs,
                                     const std::vector<double>& lower,
                                     const std::vector<double>& upper) {
    m_trackingSpec.frequencies = freqs;
    m_trackingSpec.lowerBound = lower;
    m_trackingSpec.upperBound = upper;
}

void QFTController::setTrackingSpec(double bandwidth,
                                     std::pair<double, double> dampingRange) {
    // Generate tracking bounds for second-order system
    m_trackingSpec.frequencies.clear();
    m_trackingSpec.lowerBound.clear();
    m_trackingSpec.upperBound.clear();
    
    for (double omega = bandwidth / 100; omega <= bandwidth * 10; omega *= 1.5) {
        m_trackingSpec.frequencies.push_back(omega);
        
        // Lower bound (slower response)
        double zetaLow = dampingRange.second;
        double magLow = 1.0 / std::sqrt(std::pow(1 - omega*omega/(bandwidth*bandwidth), 2) + 
                                        std::pow(2*zetaLow*omega/bandwidth, 2));
        m_trackingSpec.lowerBound.push_back(20.0 * std::log10(magLow));
        
        // Upper bound (faster response)
        double zetaHigh = dampingRange.first;
        double magHigh = 1.0 / std::sqrt(std::pow(1 - omega*omega/(bandwidth*bandwidth), 2) + 
                                         std::pow(2*zetaHigh*omega/bandwidth, 2));
        m_trackingSpec.upperBound.push_back(20.0 * std::log10(magHigh));
    }
}

void QFTController::setDisturbanceSpec(const std::vector<double>& freqs,
                                        const std::vector<double>& maxSensitivity) {
    m_disturbanceSpec.frequencies = freqs;
    m_disturbanceSpec.maxSensitivity = maxSensitivity;
}

void QFTController::setDisturbanceSpec(double Smax, double crossover) {
    m_disturbanceSpec.frequencies.clear();
    m_disturbanceSpec.maxSensitivity.clear();
    
    for (double omega = crossover / 100; omega <= crossover * 10; omega *= 1.5) {
        m_disturbanceSpec.frequencies.push_back(omega);
        if (omega < crossover) {
            m_disturbanceSpec.maxSensitivity.push_back(Smax);
        } else {
            // Roll off at 20 dB/decade above crossover
            double rolloff = Smax + 20.0 * std::log10(omega / crossover);
            m_disturbanceSpec.maxSensitivity.push_back(std::min(rolloff, 6.0));
        }
    }
}

void QFTController::setStabilityMargins(double gainMargin, double phaseMargin) {
    m_gainMarginSpec = gainMargin;
    m_phaseMarginSpec = phaseMargin;
}

void QFTController::setControlEffortSpec(const TransferFunction& bound) {
    // Store control effort bound for bound computation
}

// ============================================================================
// QFTController Implementation - Template/Bound Computation
// ============================================================================

void QFTController::setDesignFrequencies(const std::vector<double>& freqs) {
    m_designFreqs = freqs;
}

void QFTController::autoSelectFrequencies(int numPoints) {
    m_designFreqs.clear();
    
    // Determine frequency range from plant dynamics
    double omegaMin = 0.01;
    double omegaMax = 100.0;
    
    if (!m_nominalPlant.den.empty()) {
        // Use plant time constant for range
        double tau = m_nominalPlant.den[0];
        if (tau > 0) {
            omegaMin = 0.1 / tau;
            omegaMax = 100.0 / tau;
        }
    }
    
    // Logarithmic spacing
    double ratio = std::pow(omegaMax / omegaMin, 1.0 / (numPoints - 1));
    double omega = omegaMin;
    for (int i = 0; i < numPoints; i++) {
        m_designFreqs.push_back(omega);
        omega *= ratio;
    }
}

void QFTController::generateFOPDTSamples(int numSamples) {
    m_plantSamples.clear();
    
    // Generate grid of plant samples
    int nK = static_cast<int>(std::sqrt(numSamples));
    int nTau = nK;
    
    for (int i = 0; i < nK; i++) {
        double K = m_fopdtBounds.Kmin + 
                   (m_fopdtBounds.Kmax - m_fopdtBounds.Kmin) * i / (nK - 1);
        for (int j = 0; j < nTau; j++) {
            double tau = m_fopdtBounds.tauMin + 
                        (m_fopdtBounds.tauMax - m_fopdtBounds.tauMin) * j / (nTau - 1);
            
            TransferFunction tf;
            tf.num = {K};
            tf.den = {tau, 1.0};
            m_plantSamples.push_back(tf);
        }
    }
}

void QFTController::computeTemplates() {
    if (m_designFreqs.empty()) {
        autoSelectFrequencies();
    }
    
    if (m_useFOPDT && m_plantSamples.empty()) {
        generateFOPDTSamples(25);
    }
    
    m_templates.clear();
    
    for (double omega : m_designFreqs) {
        PlantTemplate templ;
        templ.frequency = omega;
        templ.nominal = NicholsPoint::fromComplex(m_nominalPlant.evaluate(omega));
        
        for (const auto& plant : m_plantSamples) {
            templ.points.push_back(NicholsPoint::fromComplex(plant.evaluate(omega)));
        }
        
        m_templates.push_back(templ);
    }
}

QFTBound QFTController::computeTrackingBound(double omega, const PlantTemplate& templ) const {
    QFTBound bound;
    bound.frequency = omega;
    bound.type = QFTBound::Tracking;
    
    // Find tracking spec at this frequency
    double lower = -20.0, upper = 6.0;
    for (size_t i = 0; i < m_trackingSpec.frequencies.size(); i++) {
        if (std::abs(m_trackingSpec.frequencies[i] - omega) < omega * 0.1) {
            lower = m_trackingSpec.lowerBound[i];
            upper = m_trackingSpec.upperBound[i];
            break;
        }
    }
    
    // Compute forbidden region boundary
    // The tracking bound constrains L such that |T| stays within bounds
    // T = L/(1+L), so for template variations
    
    for (double phase = -360; phase <= 0; phase += 10) {
        // Gain where closed-loop magnitude equals upper bound
        double gainUpper = upper + 6.0;  // Approximate transformation
        // Gain where closed-loop magnitude equals lower bound  
        double gainLower = lower - 6.0;
        
        bound.boundary.push_back(NicholsPoint(gainLower, phase));
    }
    
    return bound;
}

QFTBound QFTController::computeStabilityBound(double omega, const PlantTemplate& templ) const {
    QFTBound bound;
    bound.frequency = omega;
    bound.type = QFTBound::Stability;
    
    // Stability bound: stay away from -1 point
    // This corresponds to M-circle with M = 1/sin(PM/2) approximately
    
    double Mmax = 1.0 / std::sin(m_phaseMarginSpec * M_PI / 360.0);
    double MdB = 20.0 * std::log10(Mmax);
    
    // Forbidden region near (-180°, 0dB)
    for (double gain = -40; gain <= 20; gain += 2) {
        for (double phase = -270; phase <= -90; phase += 5) {
            NicholsPoint pt(gain, phase);
            double clMag = NicholsChart::closedLoopMagnitude(pt);
            if (clMag > MdB) {
                bound.boundary.push_back(pt);
            }
        }
    }
    
    return bound;
}

QFTBound QFTController::computeDisturbanceBound(double omega, const PlantTemplate& templ) const {
    QFTBound bound;
    bound.frequency = omega;
    bound.type = QFTBound::Disturbance;
    
    // Find disturbance spec at this frequency
    double maxS = 6.0;  // Default
    for (size_t i = 0; i < m_disturbanceSpec.frequencies.size(); i++) {
        if (std::abs(m_disturbanceSpec.frequencies[i] - omega) < omega * 0.1) {
            maxS = m_disturbanceSpec.maxSensitivity[i];
            break;
        }
    }
    
    // S = 1/(1+L), |S| < spec means |1+L| > 1/spec
    double minLoopMag = 1.0 / std::pow(10.0, maxS / 20.0);
    double minLoopDb = 20.0 * std::log10(minLoopMag);
    
    // Forbidden region: low loop gain at low frequency
    for (double phase = -270; phase <= -90; phase += 5) {
        bound.boundary.push_back(NicholsPoint(minLoopDb - 6.0, phase));
    }
    
    return bound;
}

QFTBound QFTController::combineBounds(const std::vector<QFTBound>& bounds) const {
    QFTBound combined;
    combined.type = QFTBound::Combined;
    
    if (bounds.empty()) return combined;
    combined.frequency = bounds[0].frequency;
    
    // Combine by taking union of forbidden regions
    // For simplicity, take the most restrictive bound at each phase
    for (double phase = -360; phase <= 0; phase += 5) {
        double maxGain = -100;  // Most restrictive (highest forbidden gain)
        
        for (const auto& bound : bounds) {
            for (const auto& pt : bound.boundary) {
                if (std::abs(pt.phase - phase) < 3) {
                    maxGain = std::max(maxGain, pt.gain);
                }
            }
        }
        
        if (maxGain > -100) {
            combined.boundary.push_back(NicholsPoint(maxGain, phase));
        }
    }
    
    return combined;
}

void QFTController::computeBounds() {
    m_bounds.clear();
    
    for (const auto& templ : m_templates) {
        std::vector<QFTBound> boundsAtFreq;
        
        boundsAtFreq.push_back(computeTrackingBound(templ.frequency, templ));
        boundsAtFreq.push_back(computeStabilityBound(templ.frequency, templ));
        boundsAtFreq.push_back(computeDisturbanceBound(templ.frequency, templ));
        
        m_bounds.push_back(combineBounds(boundsAtFreq));
    }
}

// ============================================================================
// QFTController Implementation - Loop Shaping
// ============================================================================

bool QFTController::autoShapeLoop() {
    return autoShapeLoop(LoopShapingConfig{});
}

bool QFTController::autoShapeLoop(const LoopShapingConfig& config) {
    if (m_templates.empty()) {
        computeTemplates();
    }
    if (m_bounds.empty()) {
        computeBounds();
    }
    
    // Use QFTLoopShaper for optimization
    m_controller = QFTLoopShaper::optimize(m_nominalPlant, m_bounds, m_designFreqs,
        QFTLoopShaper::Config{
            config.maxPoles,
            config.maxZeros,
            0.001, 1000.0, 0.001, 1000.0,
            50, config.optimizationIterations / 5
        });
    
    return checkBounds();
}

void QFTController::setController(const TransferFunction& C) {
    m_controller = C;
}

void QFTController::addControllerPole(double pole) {
    // Add real pole to controller
    std::vector<double> newDen(m_controller.den.size() + 1);
    for (size_t i = 0; i < m_controller.den.size(); i++) {
        newDen[i] += m_controller.den[i];
        newDen[i + 1] -= m_controller.den[i] * pole;
    }
    m_controller.den = newDen;
}

void QFTController::addControllerZero(double zero) {
    std::vector<double> newNum(m_controller.num.size() + 1);
    for (size_t i = 0; i < m_controller.num.size(); i++) {
        newNum[i] += m_controller.num[i];
        newNum[i + 1] -= m_controller.num[i] * zero;
    }
    m_controller.num = newNum;
}

void QFTController::addControllerPole(std::complex<double> pole) {
    if (std::abs(pole.imag()) < 1e-10) {
        addControllerPole(pole.real());
    } else {
        // Add complex conjugate pair
        double a = -2.0 * pole.real();
        double b = std::norm(pole);
        std::vector<double> newDen(m_controller.den.size() + 2);
        for (size_t i = 0; i < m_controller.den.size(); i++) {
            newDen[i] += m_controller.den[i];
            newDen[i + 1] += m_controller.den[i] * a;
            newDen[i + 2] += m_controller.den[i] * b;
        }
        m_controller.den = newDen;
    }
}

void QFTController::addControllerZero(std::complex<double> zero) {
    if (std::abs(zero.imag()) < 1e-10) {
        addControllerZero(zero.real());
    } else {
        double a = -2.0 * zero.real();
        double b = std::norm(zero);
        std::vector<double> newNum(m_controller.num.size() + 2);
        for (size_t i = 0; i < m_controller.num.size(); i++) {
            newNum[i] += m_controller.num[i];
            newNum[i + 1] += m_controller.num[i] * a;
            newNum[i + 2] += m_controller.num[i] * b;
        }
        m_controller.num = newNum;
    }
}

void QFTController::setControllerGain(double gain) {
    for (double& n : m_controller.num) {
        n *= gain;
    }
}

bool QFTController::checkBounds() const {
    for (size_t i = 0; i < m_bounds.size() && i < m_designFreqs.size(); i++) {
        auto L = evaluateLoop(m_designFreqs[i]);
        NicholsPoint loopPt = NicholsPoint::fromComplex(L);
        
        if (!m_bounds[i].isSatisfied(loopPt)) {
            return false;
        }
    }
    return true;
}

std::vector<bool> QFTController::getBoundSatisfaction() const {
    std::vector<bool> satisfaction;
    for (size_t i = 0; i < m_bounds.size() && i < m_designFreqs.size(); i++) {
        auto L = evaluateLoop(m_designFreqs[i]);
        NicholsPoint loopPt = NicholsPoint::fromComplex(L);
        satisfaction.push_back(m_bounds[i].isSatisfied(loopPt));
    }
    return satisfaction;
}

double QFTController::boundViolation(const TransferFunction& C) const {
    double totalViolation = 0;
    
    for (size_t i = 0; i < m_bounds.size() && i < m_designFreqs.size(); i++) {
        auto P = m_nominalPlant.evaluate(m_designFreqs[i]);
        auto Cv = C.evaluate(m_designFreqs[i]);
        auto L = P * Cv;
        NicholsPoint loopPt = NicholsPoint::fromComplex(L);
        
        if (!m_bounds[i].isSatisfied(loopPt)) {
            // Compute distance to allowed region
            double minDist = 1e10;
            for (const auto& bpt : m_bounds[i].boundary) {
                double dist = std::hypot(loopPt.gain - bpt.gain, loopPt.phase - bpt.phase);
                minDist = std::min(minDist, dist);
            }
            totalViolation += minDist;
        }
    }
    
    return totalViolation;
}

// ============================================================================
// QFTController Implementation - Prefilter Design
// ============================================================================

void QFTController::designPrefilter() {
    if (m_trackingSpec.frequencies.empty()) {
        designPrefilter(1.0, 2);
        return;
    }
    
    // Find bandwidth from tracking spec
    double bandwidth = m_trackingSpec.frequencies.back();
    designPrefilter(bandwidth, 2);
}

void QFTController::designPrefilter(double bandwidth, int order) {
    // Design Butterworth prefilter
    m_prefilter.num = {1.0};
    m_prefilter.den.clear();
    
    if (order == 1) {
        m_prefilter.den = {1.0 / bandwidth, 1.0};
    } else if (order == 2) {
        double wn = bandwidth;
        double zeta = 0.707;
        m_prefilter.den = {1.0 / (wn * wn), 2.0 * zeta / wn, 1.0};
    } else {
        // Higher order - use cascaded second-order sections
        m_prefilter.den = {1.0};
        for (int i = 0; i < order / 2; i++) {
            double angle = M_PI * (2 * i + 1) / (2 * order);
            double a = 2.0 * std::cos(angle) / bandwidth;
            double b = 1.0 / (bandwidth * bandwidth);
            std::vector<double> section = {b, a, 1.0};
            
            std::vector<double> newDen(m_prefilter.den.size() + 2, 0);
            for (size_t j = 0; j < m_prefilter.den.size(); j++) {
                newDen[j] += m_prefilter.den[j] * section[0];
                newDen[j + 1] += m_prefilter.den[j] * section[1];
                newDen[j + 2] += m_prefilter.den[j] * section[2];
            }
            m_prefilter.den = newDen;
        }
    }
}

void QFTController::setPrefilter(const TransferFunction& F) {
    m_prefilter = F;
}

// ============================================================================
// QFTController Implementation - Analysis
// ============================================================================

std::complex<double> QFTController::evaluateLoop(double omega) const {
    return m_nominalPlant.evaluate(omega) * m_controller.evaluate(omega);
}

std::complex<double> QFTController::evaluateClosedLoop(double omega) const {
    auto L = evaluateLoop(omega);
    return L / (1.0 + L);
}

std::complex<double> QFTController::evaluateSensitivity(double omega) const {
    auto L = evaluateLoop(omega);
    return 1.0 / (1.0 + L);
}

QFTController::StabilityMargins QFTController::analyzeMargins() const {
    StabilityMargins margins;
    
    // Find phase crossover (gain margin frequency)
    double omegaPc = 0;
    for (double omega = 0.001; omega < 1000; omega *= 1.1) {
        auto L = evaluateLoop(omega);
        double phase = std::arg(L) * 180.0 / M_PI;
        while (phase > 0) phase -= 360;
        while (phase < -360) phase += 360;
        
        if (phase < -180 && omega > 0.001) {
            omegaPc = omega;
            auto Lpc = evaluateLoop(omegaPc);
            margins.gainMargin = -20.0 * std::log10(std::abs(Lpc));
            margins.phaseCrossover = omegaPc;
            break;
        }
    }
    
    // Find gain crossover (phase margin frequency)
    for (double omega = 0.001; omega < 1000; omega *= 1.1) {
        auto L = evaluateLoop(omega);
        double mag = std::abs(L);
        
        if (mag < 1.0 && omega > 0.001) {
            margins.gainCrossover = omega;
            auto Lgc = evaluateLoop(omega);
            double phase = std::arg(Lgc) * 180.0 / M_PI;
            while (phase > 0) phase -= 360;
            margins.phaseMargin = 180.0 + phase;
            break;
        }
    }
    
    return margins;
}

std::pair<std::vector<double>, std::vector<double>>
QFTController::worstCaseResponse(double omega) const {
    std::vector<double> mags, phases;
    
    for (const auto& plant : m_plantSamples) {
        auto P = plant.evaluate(omega);
        auto C = m_controller.evaluate(omega);
        auto L = P * C;
        auto T = L / (1.0 + L);
        
        mags.push_back(20.0 * std::log10(std::abs(T)));
        phases.push_back(std::arg(T) * 180.0 / M_PI);
    }
    
    return {mags, phases};
}

// ============================================================================
// QFTController Implementation - Control Computation
// ============================================================================

ControllerOutput QFTController::computeImpl(const ControllerInput& input) {
    ControllerOutput output;
    
    double dt = input.dt;
    double y = input.measured;
    double r = input.reference;
    
    // Apply prefilter to reference (simple first-order approximation)
    static double rFiltered = 0;
    if (!m_prefilter.den.empty() && m_prefilter.den[0] > 0) {
        double tau = m_prefilter.den[0];
        rFiltered += (r - rFiltered) * dt / (tau + dt);
    } else {
        rFiltered = r;
    }
    
    double e = rFiltered - y;
    
    // Apply controller (simple PID-like implementation)
    static double integral = 0;
    static double prevError = 0;
    
    // Extract PID-like gains from controller transfer function
    double Kp = 1.0, Ki = 0.0, Kd = 0.0;
    if (!m_controller.num.empty()) {
        Kp = m_controller.num[0];
        if (m_controller.num.size() > 1) {
            Kd = m_controller.num[1];
        }
    }
    if (!m_controller.den.empty() && m_controller.den.size() > 1) {
        Ki = Kp / m_controller.den[1];
    }
    
    integral += e * dt;
    double derivative = (e - prevError) / dt;
    prevError = e;
    
    output.control = Kp * e + Ki * integral + Kd * derivative;
    output.error = e;
    
    return output;
}

void QFTController::resetImpl() {
    m_controllerState.clear();
    m_prefilterState.clear();
}

void QFTController::discretizeController(double dt) {
    m_dt = dt;
    // Full discretization would convert continuous controller to discrete
}

// ============================================================================
// NicholsChart Implementation
// ============================================================================

std::vector<NicholsPoint> NicholsChart::mCircle(double M) {
    std::vector<NicholsPoint> points;
    
    // M-circle: |L/(1+L)| = m where m = 10^(M/20)
    double m = std::pow(10.0, M / 20.0);
    
    if (std::abs(m - 1.0) < 0.01) {
        // M = 0 dB is vertical line at -180°
        for (double gain = -40; gain <= 40; gain += 2) {
            points.push_back(NicholsPoint(gain, -180.0));
        }
    } else {
        // Parametric representation
        for (double theta = 0; theta < 2 * M_PI; theta += 0.1) {
            double r = m * m / (m * m - 1);
            double x = r * std::cos(theta) - m * m / (m * m - 1);
            double y = r * std::sin(theta);
            
            std::complex<double> L(x, y);
            if (std::abs(L) > 1e-10) {
                points.push_back(NicholsPoint::fromComplex(L));
            }
        }
    }
    
    return points;
}

std::vector<NicholsPoint> NicholsChart::nCircle(double N) {
    std::vector<NicholsPoint> points;
    
    // N-circle: arg(L/(1+L)) = n where n = N * pi/180
    double n = N * M_PI / 180.0;
    
    if (std::abs(N) < 1 || std::abs(N - 180) < 1) {
        // N = 0 or 180 is horizontal line
        for (double gain = -40; gain <= 40; gain += 2) {
            points.push_back(NicholsPoint(gain, N < 90 ? 0.0 : -180.0));
        }
    } else {
        // Parametric representation
        double r = 0.5 / std::abs(std::sin(n));
        double cx = -0.5;
        double cy = 0.5 / std::tan(n);
        
        for (double theta = 0; theta < 2 * M_PI; theta += 0.1) {
            double x = cx + r * std::cos(theta);
            double y = cy + r * std::sin(theta);
            
            std::complex<double> L(x, y);
            if (std::abs(L) > 1e-10 && x < 0) {  // Only LHP
                points.push_back(NicholsPoint::fromComplex(L));
            }
        }
    }
    
    return points;
}

double NicholsChart::closedLoopMagnitude(const NicholsPoint& point) {
    auto L = point.toComplex();
    auto T = L / (1.0 + L);
    return 20.0 * std::log10(std::abs(T));
}

double NicholsChart::closedLoopPhase(const NicholsPoint& point) {
    auto L = point.toComplex();
    auto T = L / (1.0 + L);
    return std::arg(T) * 180.0 / M_PI;
}

bool NicholsChart::isUnstableRegion(const NicholsPoint& point) {
    // Check if point encircles -1 (simplified check)
    return point.gain > 0 && point.phase > -180 && point.phase < 0;
}

// ============================================================================
// TemplateUtils Implementation
// ============================================================================

std::vector<NicholsPoint> TemplateUtils::convexHull(
    const std::vector<NicholsPoint>& points) {
    
    if (points.size() < 3) return points;
    
    // Gift wrapping algorithm
    std::vector<NicholsPoint> hull;
    
    // Find leftmost point
    size_t leftmost = 0;
    for (size_t i = 1; i < points.size(); i++) {
        if (points[i].phase < points[leftmost].phase) {
            leftmost = i;
        }
    }
    
    size_t current = leftmost;
    do {
        hull.push_back(points[current]);
        size_t next = 0;
        
        for (size_t i = 0; i < points.size(); i++) {
            if (i == current) continue;
            
            if (next == current) {
                next = i;
            } else {
                // Check if i is more counterclockwise than next
                double cross = (points[i].phase - points[current].phase) *
                               (points[next].gain - points[current].gain) -
                               (points[i].gain - points[current].gain) *
                               (points[next].phase - points[current].phase);
                
                if (cross > 0 || (cross == 0 && 
                    std::hypot(points[i].phase - points[current].phase,
                               points[i].gain - points[current].gain) >
                    std::hypot(points[next].phase - points[current].phase,
                               points[next].gain - points[current].gain))) {
                    next = i;
                }
            }
        }
        
        current = next;
    } while (current != leftmost && hull.size() < points.size());
    
    return hull;
}

PlantTemplate TemplateUtils::shiftTemplate(const PlantTemplate& templ,
                                            const NicholsPoint& loopGain) {
    PlantTemplate shifted;
    shifted.frequency = templ.frequency;
    
    for (const auto& pt : templ.points) {
        shifted.points.push_back(NicholsPoint(pt.gain + loopGain.gain,
                                               pt.phase + loopGain.phase));
    }
    
    shifted.nominal = NicholsPoint(templ.nominal.gain + loopGain.gain,
                                    templ.nominal.phase + loopGain.phase);
    
    return shifted;
}

PlantTemplate TemplateUtils::interpolateTemplate(const PlantTemplate& t1,
                                                  const PlantTemplate& t2,
                                                  double omega) {
    PlantTemplate result;
    result.frequency = omega;
    
    double alpha = (omega - t1.frequency) / (t2.frequency - t1.frequency);
    alpha = std::max(0.0, std::min(1.0, alpha));
    
    // Interpolate nominal
    result.nominal.gain = t1.nominal.gain + alpha * (t2.nominal.gain - t1.nominal.gain);
    result.nominal.phase = t1.nominal.phase + alpha * (t2.nominal.phase - t1.nominal.phase);
    
    // Interpolate points (assumes same number)
    size_t n = std::min(t1.points.size(), t2.points.size());
    for (size_t i = 0; i < n; i++) {
        NicholsPoint pt;
        pt.gain = t1.points[i].gain + alpha * (t2.points[i].gain - t1.points[i].gain);
        pt.phase = t1.points[i].phase + alpha * (t2.points[i].phase - t1.points[i].phase);
        result.points.push_back(pt);
    }
    
    return result;
}

// ============================================================================
// QFTLoopShaper Implementation
// ============================================================================

TransferFunction QFTLoopShaper::optimize(
    const TransferFunction& nominalPlant,
    const std::vector<QFTBound>& bounds,
    const std::vector<double>& frequencies,
    const Config& config) {
    
    TransferFunction best;
    best.num = {1.0};
    best.den = {1.0};
    double bestCost = 1e10;
    
    // Simple evolutionary search
    std::vector<TransferFunction> population(config.populationSize);
    
    // Initialize population with random PID-like controllers
    for (int i = 0; i < config.populationSize; i++) {
        double Kp = config.gainMin + (config.gainMax - config.gainMin) * (rand() % 1000) / 1000.0;
        double Ti = config.poleMin + (config.poleMax - config.poleMin) * (rand() % 1000) / 1000.0;
        double Td = config.poleMin + (config.poleMax - config.poleMin) * (rand() % 1000) / 1000.0 / 10;
        
        population[i].num = {Kp * Td, Kp, Kp / Ti};
        population[i].den = {Td / 10, 1.0, 0.0};
    }
    
    // Evolution loop
    for (int gen = 0; gen < config.generations; gen++) {
        // Evaluate fitness
        std::vector<double> fitness(config.populationSize);
        for (int i = 0; i < config.populationSize; i++) {
            double cost = 0;
            
            for (size_t j = 0; j < frequencies.size() && j < bounds.size(); j++) {
                auto P = nominalPlant.evaluate(frequencies[j]);
                auto C = population[i].evaluate(frequencies[j]);
                auto L = P * C;
                NicholsPoint loopPt = NicholsPoint::fromComplex(L);
                
                if (!bounds[j].isSatisfied(loopPt)) {
                    for (const auto& bpt : bounds[j].boundary) {
                        double dist = std::hypot(loopPt.gain - bpt.gain,
                                                 loopPt.phase - bpt.phase);
                        cost += 1.0 / (dist + 0.1);
                    }
                }
            }
            
            // Penalize high-frequency gain
            auto Lhf = nominalPlant.evaluate(frequencies.back() * 10) * 
                       population[i].evaluate(frequencies.back() * 10);
            cost += std::abs(Lhf) * 0.1;
            
            fitness[i] = 1.0 / (cost + 0.001);
            
            if (cost < bestCost) {
                bestCost = cost;
                best = population[i];
            }
        }
        
        // Selection and reproduction
        std::vector<TransferFunction> newPop(config.populationSize);
        for (int i = 0; i < config.populationSize; i++) {
            // Tournament selection
            int a = rand() % config.populationSize;
            int b = rand() % config.populationSize;
            int parent = (fitness[a] > fitness[b]) ? a : b;
            
            newPop[i] = population[parent];
            
            // Mutation
            if (rand() % 100 < 30) {
                int idx = rand() % std::min(static_cast<int>(newPop[i].num.size()), 3);
                newPop[i].num[idx] *= (0.5 + (rand() % 100) / 100.0);
            }
        }
        
        population = newPop;
    }
    
    return best;
}

} // namespace Control
