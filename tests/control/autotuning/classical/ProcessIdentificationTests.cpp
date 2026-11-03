/**
 * @file ProcessIdentificationTests.cpp
 * @brief Coverage for ProcessIdentification static methods in
 *        classical/Common.cpp — tangentMethod, areaMethod,
 *        twoPointMethod, leastSquaresFit, identifySOPDT — driven by
 *        synthetic FOPDT/SOPDT step responses.
 */

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "tether/control/autotuning/classical/Common.hpp"

using tether::control::Autotuning::FOPDTModel;
using tether::control::Autotuning::ProcessIdentification;
using tether::control::Autotuning::SOPDTModel;

namespace {

/// Synthetic FOPDT step response: y = step*K*(1-exp(-(t-L)/tau)) for t>L.
std::pair<std::vector<double>, std::vector<double>>
fopdtResponse(double K, double tau, double L, double stepSize,
              int n = 200, double dt = 0.1) {
    std::vector<double> time, response;
    for (int i = 0; i < n; ++i) {
        double t = i * dt;
        time.push_back(t);
        double y = 0.0;
        if (t > L)
            y = K * stepSize * (1.0 - std::exp(-(t - L) / tau));
        response.push_back(y);
    }
    return {time, response};
}

TEST(ProcessIdentificationTest, TangentMethodRecoversGain) {
    auto [time, response] = fopdtResponse(1.0, 5.0, 1.0, 10.0);
    auto model = ProcessIdentification::tangentMethod(time, response, 10.0);
    EXPECT_NEAR(model.K, 1.0, 0.2);
    EXPECT_GT(model.tau, 0.0);
    EXPECT_GE(model.L, 0.0);
    EXPECT_TRUE(model.isValid());
}

TEST(ProcessIdentificationTest, TangentMethodRejectsShortData) {
    std::vector<double> time{0.0, 0.1}, response{0.0, 0.1};
    auto model = ProcessIdentification::tangentMethod(time, response, 10.0);
    // Too few samples → default model returned.
    EXPECT_TRUE(model.isValid());
}

TEST(ProcessIdentificationTest, AreaMethodRecoversGain) {
    auto [time, response] = fopdtResponse(1.0, 5.0, 1.0, 10.0);
    auto model = ProcessIdentification::areaMethod(time, response, 10.0);
    EXPECT_NEAR(model.K, 1.0, 0.2);
    EXPECT_GT(model.tau, 0.0);
}

TEST(ProcessIdentificationTest, TwoPointMethodRecoversGain) {
    auto [time, response] = fopdtResponse(1.0, 5.0, 1.0, 10.0);
    auto model = ProcessIdentification::twoPointMethod(time, response, 10.0);
    EXPECT_NEAR(model.K, 1.0, 0.2);
    EXPECT_GT(model.tau, 0.0);
    EXPECT_GE(model.L, 0.0);
}

TEST(ProcessIdentificationTest, LeastSquaresFitHandlesNoise) {
    auto [time, response] = fopdtResponse(1.0, 5.0, 1.0, 10.0);
    for (size_t i = 0; i < response.size(); ++i)
        response[i] += 0.1 * std::sin(10.0 * time[i]);  // mild noise
    auto model = ProcessIdentification::leastSquaresFit(time, response, 10.0);
    EXPECT_NEAR(model.K, 1.0, 0.3);
    EXPECT_GT(model.tau, 0.0);
}

TEST(ProcessIdentificationTest, IdentifySOPDTRecoversParameters) {
    // Synthetic SOPDT response (two cascaded first-order lags).
    const double K = 1.0, tau1 = 5.0, tau2 = 2.0, L = 1.0, step = 10.0;
    std::vector<double> time, response;
    for (int i = 0; i < 200; ++i) {
        double t = i * 0.1;
        time.push_back(t);
        double y = 0.0;
        if (t > L) {
            double tEff = t - L;
            y = K * step *
                (1.0 - (tau1 * std::exp(-tEff / tau1) -
                        tau2 * std::exp(-tEff / tau2)) /
                           (tau1 - tau2));
        }
        response.push_back(y);
    }
    auto model = ProcessIdentification::identifySOPDT(time, response, step);
    EXPECT_NEAR(model.K, K, 0.2);
    EXPECT_GT(model.tau1, 0.0);
    EXPECT_GT(model.tau2, 0.0);
}

TEST(ProcessIdentificationTest, EstimateUltimateFromModel) {
    FOPDTModel model;
    model.K = 1.0;
    model.tau = 5.0;
    model.L = 1.0;
    auto [Ku, Pu] = ProcessIdentification::estimateUltimate(model);
    EXPECT_GT(Ku, 0.0);
    EXPECT_GT(Pu, 0.0);
}

TEST(ProcessIdentificationTest, MethodsHandleEmptyInput) {
    std::vector<double> empty;
    // Must not crash on empty/short inputs.
    auto m1 = ProcessIdentification::areaMethod(empty, empty, 10.0);
    auto m2 = ProcessIdentification::twoPointMethod(empty, empty, 10.0);
    auto m3 = ProcessIdentification::leastSquaresFit(empty, empty, 10.0);
    auto m4 = ProcessIdentification::identifySOPDT(empty, empty, 10.0);
    SUCCEED();
}

TEST(ProcessIdentificationTest, AllMethodsAgreeOnCleanData) {
    auto [time, response] = fopdtResponse(1.5, 4.0, 0.8, 8.0);
    auto mArea = ProcessIdentification::areaMethod(time, response, 8.0);
    auto m2pt = ProcessIdentification::twoPointMethod(time, response, 8.0);
    auto mTan = ProcessIdentification::tangentMethod(time, response, 8.0);
    // Independent estimators should land in the same ballpark for K.
    EXPECT_NEAR(mArea.K, 1.5, 0.4);
    EXPECT_NEAR(m2pt.K, 1.5, 0.4);
    EXPECT_NEAR(mTan.K, 1.5, 0.4);
}

}  // namespace
