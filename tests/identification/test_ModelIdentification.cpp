#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include <identification/IdentificationInternal.hpp>
#include <tether/identification/ModelIdentification.hpp>

using namespace Identification;

namespace {

constexpr double kPi = 3.14159265358979323846;

Vector makeStepInput(size_t count, double step_value) {
    Vector input(count, 0.0);
    for (size_t i = count / 8; i < count; ++i) {
        input[i] = step_value;
    }
    return input;
}

Vector simulateArxPlant(const Vector& input, double a1, double b1) {
    Vector output(input.size(), 0.0);
    for (size_t i = 1; i < input.size(); ++i) {
        output[i] = -a1 * output[i - 1] + b1 * input[i - 1];
    }
    return output;
}

Vector makeRichInput(size_t count) {
    Vector input(count, 0.0);
    for (size_t i = 0; i < count; ++i) {
        input[i] = std::sin(2.0 * kPi * static_cast<double>(i) / 19.0) +
            0.4 * std::cos(2.0 * kPi * static_cast<double>(i) / 7.0);
    }
    return input;
}

std::vector<DynamicFrictionSample> makeDynamicSamples() {
    std::vector<DynamicFrictionSample> samples;
    double displacement = 0.0;
    for (size_t i = 0; i < 200; ++i) {
        const double t = 0.01 * static_cast<double>(i);
        const double velocity = std::sin(2.0 * kPi * 0.4 * t);
        displacement += 0.01 * velocity;
        const double force = 1.5 * displacement + 3.0 * std::tanh(4.0 * velocity);
        samples.push_back({displacement, velocity, force, 0.01});
    }
    return samples;
}

Matrix makeMimoInputs(size_t count) {
    Matrix inputs;
    inputs.reserve(count);
    for (size_t k = 0; k < count; ++k) {
        inputs.push_back({
            std::sin(0.05 * static_cast<double>(k)),
            std::cos(0.035 * static_cast<double>(k))
        });
    }
    return inputs;
}

Matrix simulateStateSpaceOutputs(const Matrix& inputs) {
    Matrix outputs;
    outputs.reserve(inputs.size());
    Vector x{0.0, 0.0};
    for (const auto& u : inputs) {
        outputs.push_back({x[0] + 0.2 * u[0], x[1] + 0.1 * u[1]});
        const double next_x0 = 0.85 * x[0] + 0.1 * x[1] + 0.4 * u[0];
        const double next_x1 = -0.05 * x[0] + 0.9 * x[1] + 0.3 * u[1];
        x[0] = next_x0;
        x[1] = next_x1;
    }
    return outputs;
}

} // namespace

TEST(CommonIdentificationTest, SignAndDataBufferCoverThresholdsAndWrap) {
    EXPECT_FLOAT_EQ(sign(0.5f), 1.0f);
    EXPECT_FLOAT_EQ(sign(-0.5f), -1.0f);
    EXPECT_FLOAT_EQ(sign(0.0001f), 0.0f);

    DataBuffer<2, 3> buffer;
    float a[2] = {1.0f, 2.0f};
    float b[2] = {3.0f, 4.0f};
    float c[2] = {5.0f, 6.0f};
    float d[2] = {7.0f, 8.0f};
    buffer.addSample(0.0f, a);
    buffer.addSample(1.0f, b);
    EXPECT_EQ(buffer.count(), 2u);
    EXPECT_EQ(buffer.capacity(), 3u);
    EXPECT_FALSE(buffer.isFull());
    EXPECT_FLOAT_EQ(buffer[1].values[0], 3.0f);

    buffer.addSample(2.0f, c);
    buffer.addSample(3.0f, d);
    EXPECT_EQ(buffer.count(), 3u);
    EXPECT_TRUE(buffer.isFull());
    EXPECT_FLOAT_EQ(buffer[0].timestamp, 1.0f);
    EXPECT_FLOAT_EQ(buffer[2].values[1], 8.0f);

    buffer.clear();
    EXPECT_EQ(buffer.count(), 0u);
}

TEST(StepResponseTest, HandlesInsufficientDataZeroAmplitudeFirstOrderAndSecondOrderCases) {
    StepResponseAnalyzer empty;
    EXPECT_EQ(empty.getSampleCount(), 0u);
    const auto empty_result = empty.analyze();
    EXPECT_FLOAT_EQ(empty_result.gain, 0.0f);

    StepResponseAnalyzer wrapped_buffer;
    wrapped_buffer.setStepInput(0.0f, 1.0f);
    for (int i = 0; i < 4200; ++i) {
        wrapped_buffer.addSample(0.001f * static_cast<float>(i), 0.5f);
    }
    EXPECT_EQ(wrapped_buffer.getSampleCount(), 4096u);
    EXPECT_GE(wrapped_buffer.analyze().steady_state_value, 0.0f);

    StepResponseAnalyzer zero_step;
    zero_step.setStepInput(0.0f, 0.0f);
    for (int i = -5; i < 30; ++i) {
        const float t = 0.05f * static_cast<float>(i);
        const float y = i < 0 ? 0.0f : 1.0f - std::exp(-0.2f * static_cast<float>(i));
        zero_step.addSample(t, y);
    }
    const auto zero_gain = zero_step.analyze();
    EXPECT_FLOAT_EQ(zero_gain.gain, 0.0f);

    StepResponseAnalyzer first_order;
    first_order.setStepInput(0.0f, 1.0f);
    for (int i = -5; i < 60; ++i) {
        const float t = 0.05f * static_cast<float>(i);
        const float y = i < 0 ? 0.0f : 1.0f - std::exp(-0.25f * static_cast<float>(i));
        first_order.addSample(t, y);
    }
    const auto first = first_order.analyze();
    EXPECT_FALSE(first.is_second_order);
    EXPECT_GT(first.gain, 0.8f);
    EXPECT_GT(first.time_constant, 0.0f);
    EXPECT_GT(first.settling_time, 0.0f);

    StepResponseAnalyzer second_order;
    second_order.setStepInput(0.0f, 1.0f);
    for (int i = -5; i < 25; ++i) {
        const float t = 0.05f * static_cast<float>(i);
        float y = 0.0f;
        if (i >= 0) {
            static const float response[] = {
                0.0f, 0.2f, 0.55f, 0.95f, 1.25f, 1.18f, 1.08f, 1.02f, 1.0f, 1.0f,
                1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
                1.0f, 1.0f, 1.0f, 1.0f, 1.0f
            };
            y = response[std::min(i, 24)];
        }
        second_order.addSample(t, y);
    }
    const auto second = second_order.analyze();
    EXPECT_TRUE(second.is_second_order);
    EXPECT_GT(second.overshoot, 1.0f);
    EXPECT_GT(second.natural_frequency, 0.0f);

    StepResponseAnalyzer negative_step;
    negative_step.setStepInput(0.0f, -1.0f);
    for (int i = -5; i < 25; ++i) {
        const float t = 0.05f * static_cast<float>(i);
        float y = 0.0f;
        if (i >= 0) {
            static const float response[] = {
                0.0f, -0.2f, -0.55f, -0.95f, -1.25f, -1.18f, -1.08f, -1.02f, -1.0f, -1.0f,
                -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f,
                -1.0f, -1.0f, -1.0f, -1.0f, -1.0f
            };
            y = response[std::min(i, 24)];
        }
        negative_step.addSample(t, y);
    }
    EXPECT_TRUE(negative_step.analyze().is_second_order);

    StepResponseAnalyzer unsettled;
    unsettled.setStepInput(0.0f, 1.0f);
    for (int i = -2; i < 12; ++i) {
        const float t = 0.1f * static_cast<float>(i);
        const float y = i < 0 ? 0.0f : 0.08f * static_cast<float>(i + 1);
        unsettled.addSample(t, y);
    }
    EXPECT_GT(unsettled.analyze().settling_time, 0.0f);
}

TEST(LeastSquaresTest, RecursiveAndBatchEstimatorsCoverSuccessAndGuardPaths) {
    RecursiveLeastSquares<2> rls(0.99f, 100.0f);
    for (int i = 0; i < 40; ++i) {
        const float phi[2] = {static_cast<float>(i), static_cast<float>(1 + i % 3)};
        const float y = 2.0f * phi[0] + 3.0f * phi[1];
        rls.update(phi, y);
    }
    EXPECT_NEAR(rls.getParameter(0), 2.0f, 1e-2f);
    EXPECT_NEAR(rls.getParameter(1), 3.0f, 1e-1f);
    const float predict_phi[2] = {4.0f, 2.0f};
    EXPECT_NEAR(rls.predict(predict_phi), 14.0f, 0.3f);
    EXPECT_NE(rls.getParameters(), nullptr);
    EXPECT_EQ(rls.getParameter(10), 0.0f);
    EXPECT_EQ(rls.getCovariance(5, 5), 0.0f);
    EXPECT_EQ(rls.getParameterStdDev(4), 0.0f);
    rls.setForgettingFactor(0.97f);
    EXPECT_FLOAT_EQ(rls.getForgettingFactor(), 0.97f);

    RecursiveLeastSquares<1> zero_denom(0.0f, 0.0f);
    const float zero_phi[1] = {0.0f};
    EXPECT_FLOAT_EQ(zero_denom.update(zero_phi, 5.0f), 5.0f);
    EXPECT_EQ(zero_denom.getSampleCount(), 0u);
    zero_denom.reset(10.0f);
    EXPECT_EQ(zero_denom.getSampleCount(), 0u);

    RecursiveLeastSquares<1> scalar_rls(0.95f, 50.0f);
    const float scalar_phi[1] = {2.0f};
    scalar_rls.update(scalar_phi, 8.0f);
    EXPECT_NEAR(scalar_rls.predict(scalar_phi), 8.0f, 0.5f);

    RecursiveLeastSquares<2> zero_denom_two(0.0f, 0.0f);
    const float zero_phi_two[2] = {0.0f, 0.0f};
    EXPECT_FLOAT_EQ(zero_denom_two.update(zero_phi_two, 3.0f), 3.0f);

    RecursiveLeastSquares<3> zero_denom_three(0.0f, 0.0f);
    const float zero_phi_three[3] = {0.0f, 0.0f, 0.0f};
    EXPECT_FLOAT_EQ(zero_denom_three.update(zero_phi_three, 4.0f), 4.0f);

    BatchLeastSquares<2> underdetermined;
    float theta2[2] = {0.0f, 0.0f};
    EXPECT_FALSE(underdetermined.solve(theta2));

    BatchLeastSquares<3> underdetermined3;
    float theta_under3[3] = {0.0f, 0.0f, 0.0f};
    EXPECT_FALSE(underdetermined3.solve(theta_under3));

    BatchLeastSquares<2> batch;
    const float s0[2] = {1.0f, 0.0f};
    const float s1[2] = {0.0f, 1.0f};
    const float s2[2] = {1.0f, 1.0f};
    batch.addSample(s0, 2.0f);
    batch.addSample(s1, 3.0f);
    batch.addSample(s2, 5.0f);
    ASSERT_TRUE(batch.solve(theta2));
    EXPECT_NEAR(theta2[0], 2.0f, 1e-3f);
    EXPECT_NEAR(theta2[1], 3.0f, 1e-3f);
    EXPECT_FLOAT_EQ(batch.computeResidualSS(theta2), 0.0f);

    BatchLeastSquares<1, 1> bounded;
    const float one[1] = {1.0f};
    bounded.addSample(one, 1.0f);
    bounded.addSample(one, 2.0f);
    EXPECT_EQ(bounded.getSampleCount(), 1u);
    bounded.clear();
    EXPECT_EQ(bounded.getSampleCount(), 0u);

    BatchLeastSquares<2, 1> bounded2;
    bounded2.addSample(s0, 2.0f);
    bounded2.addSample(s1, 3.0f);

    BatchLeastSquares<3, 1> bounded3;
    const float bounded3_phi[3] = {1.0f, 0.0f, 0.0f};
    bounded3.addSample(bounded3_phi, 1.0f);
    bounded3.addSample(bounded3_phi, 2.0f);

    BatchLeastSquares<3> batch3;
    const float b0[3] = {1.0f, 0.0f, 0.0f};
    const float b1[3] = {1.0f, 1.0f, 0.0f};
    const float b2[3] = {1.0f, 1.0f, 1.0f};
    batch3.addSample(b0, 1.0f);
    batch3.addSample(b1, 2.0f);
    batch3.addSample(b2, 3.0f);
    float theta3[3] = {0.0f, 0.0f, 0.0f};
    EXPECT_TRUE(batch3.solve(theta3));
}

TEST(MotorIdentifierTest, LearnsTorqueModelAndHandlesInvalidDt) {
    MotorIdentifier identifier;
    identifier.update(0.0f, 0.0f, 0.0f);
    EXPECT_EQ(identifier.getSampleCount(), 0u);

    double prev_velocity = 0.0;
    for (int i = 1; i <= 400; ++i) {
        const double t = 0.01 * static_cast<double>(i);
        const double velocity = std::sin(0.05 * static_cast<double>(i));
        const double acceleration = (velocity - prev_velocity) / 0.01;
        const double torque = 0.4 * acceleration + 0.3 * velocity + 0.2 * sign(static_cast<float>(velocity));
        identifier.update(static_cast<float>(t), static_cast<float>(velocity), static_cast<float>(torque));
        prev_velocity = velocity;
    }

    const auto params = identifier.getParameters();
    EXPECT_GT(identifier.getSampleCount(), 10u);
    EXPECT_NEAR(identifier.predictTorque(1.0f, 2.0f), 1.3f, 0.8f);
    EXPECT_GT(params.inertia, 0.0f);
    EXPECT_GT(params.inertia_std, 0.0f);
    identifier.reset();
    EXPECT_EQ(identifier.getSampleCount(), 0u);
}

TEST(FrictionIdentifierTest, CoversInsufficientLowVelocityDegenerateAndNominalFits) {
    FrictionIdentifier insufficient;
    insufficient.addMeasurement(0.1f, 0.2f);
    EXPECT_FLOAT_EQ(insufficient.identify().fit_error, 0.0f);

    FrictionIdentifier low_velocity_only;
    low_velocity_only.addMeasurement(0.01f, 0.5f);
    low_velocity_only.addMeasurement(-0.02f, -0.55f);
    low_velocity_only.addMeasurement(0.03f, 0.52f);
    low_velocity_only.addMeasurement(-0.04f, -0.53f);
    const auto low = low_velocity_only.identify();
    EXPECT_GT(low.stiction, 0.0f);
    EXPECT_FLOAT_EQ(low.coulomb, 0.0f);

    FrictionIdentifier degenerate_high_velocity;
    degenerate_high_velocity.addMeasurement(1.0f, 1.0f);
    degenerate_high_velocity.addMeasurement(-1.0f, -1.0f);
    degenerate_high_velocity.addMeasurement(0.01f, 1.4f);
    degenerate_high_velocity.addMeasurement(-0.01f, -1.4f);
    const auto degenerate = degenerate_high_velocity.identify();
    EXPECT_GT(degenerate.stiction, 0.0f);

    FrictionIdentifier nominal;
    for (int i = 0; i < 300; ++i) {
        nominal.addMeasurement(0.01f * static_cast<float>(i), 0.02f * static_cast<float>(i));
    }
    for (float v : {-1.5f, -1.0f, -0.5f, -0.05f, 0.05f, 0.5f, 1.0f, 1.5f}) {
        const float friction = 0.8f * sign(v) + 0.4f * v + (std::abs(v) < 0.1f ? 0.4f * sign(v) : 0.0f);
        nominal.addMeasurement(v, friction);
    }
    const auto fit = nominal.identify();
    EXPECT_GT(fit.fit_error, 0.0f);
    EXPECT_GT(fit.stiction, fit.coulomb);
}

TEST(FrequencyIdentificationTest, CoversAnalyzerGeneratorsAndTransferFunctions) {
    FrequencyResponseAnalyzer insufficient;
    EXPECT_FLOAT_EQ(insufficient.computeResponse().magnitude, 0.0f);

    FrequencyResponseAnalyzer no_input;
    no_input.reset(2.0f, 1.0f);
    for (int i = 0; i < 20; ++i) {
        no_input.addSample(0.01f, 0.0f, 0.5f);
    }
    EXPECT_FLOAT_EQ(no_input.computeResponse().magnitude, 0.0f);

    FrequencyResponseAnalyzer analyzer;
    analyzer.reset(2.0f, 1.0f);
    for (int i = 0; i < 4000; ++i) {
        const float t = 0.001f * static_cast<float>(i);
        const float input = std::sin(2.0f * PI * 2.0f * t);
        const float output = 0.5f * std::sin(2.0f * PI * 2.0f * t + static_cast<float>(kPi / 4.0));
        analyzer.addSample(0.001f, input, output);
    }
    const auto bode = analyzer.computeResponse();
    EXPECT_NEAR(bode.magnitude, 0.5f, 0.05f);
    EXPECT_NEAR(bode.phase, static_cast<float>(-kPi / 4.0), 0.1f);
    EXPECT_GT(analyzer.getCyclesRecorded(), 1.0f);
    EXPECT_LT(std::abs(analyzer.generateExcitation()), 0.01f);

    FrequencyResponseAnalyzer wrap_high;
    wrap_high.reset(1.0f, 1.0f);
    for (int i = 0; i < 3000; ++i) {
        const float t = 0.001f * static_cast<float>(i);
        const float input = std::sin(2.0f * PI * t);
        const float output = std::sin(2.0f * PI * t + 3.5f);
        wrap_high.addSample(0.001f, input, output);
    }
    EXPECT_LE(wrap_high.computeResponse().phase, PI);

    FrequencyResponseAnalyzer wrap_low;
    wrap_low.reset(1.0f, 1.0f);
    for (int i = 0; i < 3000; ++i) {
        const float t = 0.001f * static_cast<float>(i);
        const float input = std::sin(2.0f * PI * t);
        const float output = std::sin(2.0f * PI * t - 3.5f);
        wrap_low.addSample(0.001f, input, output);
    }
    EXPECT_GE(wrap_low.computeResponse().phase, -PI);

    FrequencyResponseAnalyzer wrap_positive;
    wrap_positive.reset(1.0f, 1.0f);
    for (int i = 0; i < 3000; ++i) {
        const float t = 0.001f * static_cast<float>(i);
        const float input = std::sin(2.0f * PI * t + 3.5f);
        const float output = std::sin(2.0f * PI * t);
        wrap_positive.addSample(0.001f, input, output);
    }
    EXPECT_LE(wrap_positive.computeResponse().phase, PI);

    ChirpGenerator chirp(1.0f, 10.0f, 0.1f, 2.0f);
    EXPECT_FLOAT_EQ(chirp.generate(0.01f), 0.0f);
    EXPECT_NE(chirp.generate(0.01f), 0.0f);
    EXPECT_GT(chirp.getInstantaneousFrequency(), 1.0f);
    while (!chirp.isComplete()) {
        chirp.generate(0.02f);
    }
    EXPECT_EQ(chirp.generate(0.01f), 0.0f);
    chirp.reset();
    EXPECT_LT(chirp.getProgress(), 0.5f);

    PRBSGenerator prbs_invalid(42, 1.0f, 2);
    EXPECT_EQ(prbs_invalid.getSequenceLength(), 7u);
    const float first_prbs = prbs_invalid.generate();
    prbs_invalid.generate();
    EXPECT_TRUE(first_prbs == 1.0f || first_prbs == -1.0f);
    EXPECT_TRUE(prbs_invalid.getCurrentBit() == 0 || prbs_invalid.getCurrentBit() == 1);
    prbs_invalid.reset();

    PRBSGenerator prbs3(3, 1.0f, 1);
    for (int i = 0; i < 8; ++i) {
        const float value = prbs3.generate();
        EXPECT_TRUE(value == 1.0f || value == -1.0f);
    }

    for (uint8_t order = 4; order <= 10; ++order) {
        PRBSGenerator prbs(order, 1.0f, 1);
        const float value = prbs.generate();
        EXPECT_TRUE(value == 1.0f || value == -1.0f);
    }

    FirstOrderTF first_order{2.0f, 0.5f};
    EXPECT_GT(first_order.magnitude(1.0f), 0.0f);
    EXPECT_LT(first_order.phase(1.0f), 0.0f);

    SecondOrderTF damped{1.0f, 10.0f, 0.4f};
    EXPECT_GT(damped.magnitude(1.0f), 0.0f);
    EXPECT_LT(damped.phase(1.0f), 0.0f);
    EXPECT_GT(damped.resonantFrequency(), 0.0f);
    EXPECT_GT(damped.peakMagnitude(), 1.0f);

    SecondOrderTF overdamped{1.0f, 10.0f, 0.9f};
    EXPECT_FLOAT_EQ(overdamped.resonantFrequency(), 0.0f);
    EXPECT_FLOAT_EQ(overdamped.peakMagnitude(), 1.0f);
}

TEST(DenseLinearAlgebraTest, CoversCoreOperationsAndGuardPaths) {
    const Matrix A = {{1.0, 2.0}, {3.0, 4.0}};
    const Matrix B = {{2.0, 0.0}, {1.0, 2.0}};
    EXPECT_EQ(makeMatrix(2, 3, 1.5).size(), 2u);
    EXPECT_EQ(identityMatrix(2)[1][1], 1.0);
    EXPECT_EQ(transpose(A)[0][1], 3.0);
    EXPECT_EQ(multiply(A, B)[0][0], 4.0);
    EXPECT_EQ(multiply(A, Vector{1.0, 1.0})[1], 7.0);
    EXPECT_EQ(add(A, B)[1][1], 6.0);
    EXPECT_EQ(subtract(A, B)[0][0], -1.0);
    EXPECT_EQ(outerProduct(Vector{1.0, 2.0}, Vector{3.0})[1][0], 6.0);

    EXPECT_TRUE(solveLinearSystem({}, {}).empty());
    EXPECT_TRUE(solveLeastSquares({}, {}).empty());
    EXPECT_EQ(solveLinearSystem({{0.0}}, {1.0})[0], 0.0);
    const Vector solution = solveLinearSystem({{2.0, 1.0}, {1.0, 3.0}}, {1.0, 2.0});
    EXPECT_NEAR(solution[0], 0.2, 1e-6);
    EXPECT_NEAR(solution[1], 0.6, 1e-6);

    const Vector ls = solveLeastSquares({{1.0, 1.0}, {1.0, 2.0}, {1.0, 3.0}}, {2.0, 3.0, 4.0});
    EXPECT_NEAR(ls[0], 1.0, 1e-6);
    EXPECT_NEAR(ls[1], 1.0, 1e-6);

    EXPECT_TRUE(pseudoInverse({}).empty());
    EXPECT_TRUE(transpose({}).empty());
    EXPECT_NEAR(pseudoInverse({{1.0, 0.0}, {0.0, 1.0}})[1][1], 1.0, 1e-6);
    const Matrix wide = pseudoInverse({{1.0, 2.0, 3.0}});
    EXPECT_EQ(wide.size(), 3u);

    EXPECT_TRUE(jacobiEigenDecomposition({}).values.empty());
    const auto eig = jacobiEigenDecomposition({{3.0, 1.0}, {1.0, 3.0}});
    EXPECT_EQ(eig.values.size(), 2u);
    EXPECT_NEAR(dot(Vector{1.0, 2.0}, Vector{3.0, 4.0}), 11.0, 1e-9);
    EXPECT_NEAR(norm(Vector{3.0, 4.0}), 5.0, 1e-9);
    EXPECT_NEAR(frobeniusNorm(A), std::sqrt(30.0), 1e-9);
    EXPECT_EQ(column(A, 1)[0], 2.0);
    Matrix rows;
    appendRow(rows, {1.0, 2.0});
    EXPECT_EQ(rows.size(), 1u);
    EXPECT_FLOAT_EQ(static_cast<float>(conditionNumber({})), 0.0f);
    EXPECT_FLOAT_EQ(static_cast<float>(conditionNumber({{}})), 0.0f);
    EXPECT_GT(conditionNumber(A), 1.0);
}

TEST(PolynomialModelsTest, CoversInvalidAndNominalIdentificationPaths) {
    PolynomialModelOrders orders;
    orders.na = 1;
    orders.nb = 1;
    orders.nc = 1;
    orders.nd = 1;
    orders.nf = 1;
    orders.nk = 1;
    orders.iterations = 6;

    const auto invalid_arx = ARXIdentifier::identify(Vector{1.0}, Vector{}, orders);
    const auto invalid_iv = InstrumentalVariablesIdentifier::identify(Vector{1.0}, Vector{}, orders);
    const auto invalid_armax = ARMAXIdentifier::identify(Vector{1.0}, Vector{}, orders);
    EXPECT_TRUE(invalid_arx.valid());
    EXPECT_TRUE(invalid_iv.valid());
    EXPECT_TRUE(invalid_armax.valid());

    const Vector input = makeRichInput(400);
    const Vector output = simulateArxPlant(input, -0.72, 0.45);

    const auto arx = ARXIdentifier::identify(input, output, orders);
    const auto iv = InstrumentalVariablesIdentifier::identify(input, output, orders);
    const auto armax = ARMAXIdentifier::identify(input, output, orders);
    const auto oe = OEIdentifier::identify(input, output, orders);
    const auto bj = BoxJenkinsIdentifier::identify(input, output, orders);

    EXPECT_TRUE(arx.valid());
    EXPECT_TRUE(iv.valid());
    EXPECT_TRUE(armax.valid());
    EXPECT_TRUE(oe.valid());
    EXPECT_TRUE(bj.valid());
    EXPECT_NEAR(arx.A[1], -0.72, 0.1);
    EXPECT_NEAR(arx.B[0], 0.45, 0.1);
    EXPECT_NEAR(iv.A[1], -0.72, 0.15);
    EXPECT_NEAR(armax.B[0], 0.45, 0.15);
    EXPECT_GE(oe.fit, 0.0);
    EXPECT_GE(bj.fit, 0.0);
    EXPECT_TRUE(std::isfinite(arx.predictOneStep(input, output, 10)));
    EXPECT_TRUE(arx.simulate({}).empty());

    PolynomialModelOrders oe_no_params;
    oe_no_params.nb = 0;
    oe_no_params.nf = 0;
    oe_no_params.nk = 0;
    const auto trivial_oe = OEIdentifier::identify(Vector(16, 0.0), Vector(16, 0.0), oe_no_params);
    EXPECT_TRUE(trivial_oe.valid());

    PolynomialModelOrders refined_orders = orders;
    refined_orders.nb = 2;
    refined_orders.nf = 2;
    refined_orders.iterations = 12;
    const auto refined_oe = OEIdentifier::identify(input, output, refined_orders);
    EXPECT_TRUE(refined_oe.valid());
}

TEST(SubspaceIdentificationTest, CoversEarlyReturnAndNominalMimoFits) {
    EXPECT_FALSE(N4SIDIdentifier::identify({}, {}, 4).valid());

    Matrix empty_channels(16, Vector{});
    EXPECT_FALSE(MOESPIdentifier::identify(empty_channels, empty_channels, 4).valid());

    const Matrix inputs = makeMimoInputs(160);
    const Matrix outputs = simulateStateSpaceOutputs(inputs);

    const auto n4sid = N4SIDIdentifier::identify(inputs, outputs, 6, 0);
    const auto moesp = MOESPIdentifier::identify(inputs, outputs, 6, 2);
    const auto cva = CVAIdentifier::identify(inputs, outputs, 6, 2);

    EXPECT_TRUE(n4sid.valid());
    EXPECT_TRUE(moesp.valid());
    EXPECT_TRUE(cva.valid());
    EXPECT_GE(n4sid.order, 1u);
    EXPECT_EQ(moesp.C.size(), 2u);
    EXPECT_EQ(cva.order, 2u);
    EXPECT_TRUE(std::isfinite(n4sid.fit));
    EXPECT_TRUE(std::isfinite(moesp.fit));
    EXPECT_TRUE(std::isfinite(cva.fit));
    EXPECT_FALSE(n4sid.singular_values.empty());
    EXPECT_EQ(n4sid.propagate(Vector(n4sid.order, 0.0), inputs.front()).size(), n4sid.order);
    EXPECT_EQ(n4sid.output(Vector(n4sid.order, 0.0), inputs.front()).size(), 2u);
}

TEST(AdvancedFrictionModelsTest, CoversGuardPathsAndFiniteDynamicFits) {
    const std::vector<DynamicFrictionSample> small = {{0.0, 0.0, 0.0, 0.01}};
    EXPECT_FLOAT_EQ(LuGreIdentifier::identify(small).fit, 0.0);
    EXPECT_FLOAT_EQ(DahlIdentifier::identify(small).fit, 0.0);
    EXPECT_FLOAT_EQ(BoucWenIdentifier::identify(small).fit, 0.0);
    EXPECT_FLOAT_EQ(PreisachIdentifier::identify(small).fit, 0.0);
    EXPECT_TRUE(PreisachIdentifier::identify(makeDynamicSamples(), 0).weights.empty());

    PreisachModel manual;
    manual.alpha_thresholds = {0.5, 1.0};
    manual.beta_thresholds = {-0.5, -1.0};
    manual.weights = {1.0, 2.0};
    EXPECT_DOUBLE_EQ(manual.evaluate(2.0), 3.0);
    EXPECT_DOUBLE_EQ(manual.evaluate(-2.0), -3.0);
    EXPECT_DOUBLE_EQ(manual.evaluate(0.0), 0.0);
    EXPECT_EQ(PreisachIdentifier::simulate({{{2.0, 0.0, 0.0, 0.01}, {-2.0, 0.0, 0.0, 0.01}}}, manual).size(), 2u);

    const auto samples = makeDynamicSamples();
    const std::vector<DynamicFrictionSample> relay_samples = {
        {1.5, 0.0, 1.0, 0.01},
        {-1.5, 0.0, -1.0, 0.01},
        {1.2, 0.0, 0.8, 0.01},
        {-1.2, 0.0, -0.8, 0.01}
    };
    const auto lugre = LuGreIdentifier::identify(samples);
    const auto dahl = DahlIdentifier::identify(samples);
    const auto bouc_wen = BoucWenIdentifier::identify(samples);
    const auto preisach = PreisachIdentifier::identify(samples, 6);
    const auto relay_preisach = PreisachIdentifier::identify(relay_samples, 2);

    EXPECT_TRUE(std::isfinite(lugre.fit));
    EXPECT_TRUE(std::isfinite(dahl.fit));
    EXPECT_TRUE(std::isfinite(bouc_wen.fit));
    EXPECT_TRUE(std::isfinite(preisach.fit));
    EXPECT_GT(lugre.fit, 20.0);
    EXPECT_GT(dahl.fit, 20.0);
    EXPECT_GT(bouc_wen.fit, 20.0);
    EXPECT_GT(preisach.fit, 20.0);
    EXPECT_FALSE(relay_preisach.weights.empty());
    EXPECT_EQ(LuGreIdentifier::simulate(samples, lugre).size(), samples.size());
    EXPECT_EQ(DahlIdentifier::simulate(samples, dahl).size(), samples.size());
    EXPECT_EQ(BoucWenIdentifier::simulate(samples, bouc_wen).size(), samples.size());
    EXPECT_EQ(PreisachIdentifier::simulate(samples, preisach).size(), samples.size());
}

TEST(RigidBodyIdentificationTest, CoversEstimatorGuardsScoresAndTrajectories) {
    EXPECT_TRUE(BaseParameterEstimator::estimateQR({}, {}).base_parameters.empty());
    EXPECT_TRUE(BaseParameterEstimator::estimateSVD({}, {}).base_parameters.empty());
    EXPECT_FLOAT_EQ(static_cast<float>(ExcitationTrajectoryOptimizer::evaluateInformationScore({})), 0.0f);
    EXPECT_FLOAT_EQ(static_cast<float>(ExcitationTrajectoryOptimizer::evaluateInformationScore({{}})), 0.0f);
    EXPECT_FLOAT_EQ(static_cast<float>(ExcitationTrajectoryOptimizer::evaluateInformationScore({{0.0, 0.0}, {0.0, 0.0}})), 0.0f);

    BSplineExcitationTrajectory empty_spline;
    EXPECT_TRUE(empty_spline.sample(0.1).empty());

    Matrix regressor = {
        {1.0, 0.0, 1.0},
        {0.0, 1.0, 1.0},
        {1.0, 1.0, 2.0},
        {2.0, 1.0, 3.0},
        {1.5, 0.5, 2.0}
    };
    Vector torque = {2.0, 3.0, 5.0, 7.0, 4.0};

    const auto svd = BaseParameterEstimator::estimateSVD(regressor, torque);
    const auto qr = BaseParameterEstimator::estimateQR(regressor, torque);
    const auto fourier = ExcitationTrajectoryOptimizer::optimizeFourier(3, 3, 2.0, regressor);
    const auto spline = ExcitationTrajectoryOptimizer::optimizeBSpline(3, 5, 2.0, regressor);

    EXPECT_GE(svd.rank, 2u);
    EXPECT_GE(qr.rank, 2u);
    EXPECT_FALSE(svd.base_parameters.empty());
    EXPECT_EQ(fourier.sample(0.5).size(), 3u);
    EXPECT_EQ(spline.sample(0.5).size(), 3u);
    EXPECT_GT(ExcitationTrajectoryOptimizer::evaluateInformationScore(regressor), 0.0);
}

TEST(AdaptiveObserversTest, CoversGuardAndNominalObserverUpdates) {
    AugmentedStateEKF ekf_guard(2, 1);
    EXPECT_EQ(ekf_guard.step({1.0}, {0.0}, 0.1).size(), 2u);

    AugmentedStateUKF ukf_guard;
    EXPECT_TRUE(ukf_guard.step({}, {}, 0.1).empty());

    const auto transition = [](const Vector& state, const Vector& input, double dt) {
        return Vector{state[0] + dt * state[1] * input[0], state[1]};
    };
    const auto measurement = [](const Vector& state) {
        return Vector{state[0]};
    };

    AugmentedStateEKF ekf(2, 1);
    ekf.setModel(transition, measurement);
    ekf.setState({0.0, 0.2});
    ekf.setCovariance(identityMatrix(2));
    ekf.setMeasurementNoise(makeMatrix(1, 1, 1e-3));
    ekf.setProcessNoise({{1e-4, 0.0}, {0.0, 1e-5}});

    AugmentedStateUKF ukf(2, 1);
    ukf.setModel(transition, measurement);
    ukf.setState({0.0, 0.2});
    ukf.setCovariance(identityMatrix(2));
    ukf.setMeasurementNoise(makeMatrix(1, 1, 1e-3));
    ukf.setProcessNoise({{1e-4, 0.0}, {0.0, 1e-5}});
    ukf.setScaling(0.2, 2.0, 0.0);

    Vector true_state{0.0, 0.5};
    for (size_t k = 0; k < 40; ++k) {
        const Vector input{1.0};
        true_state = transition(true_state, input, 0.05);
        const Vector y{true_state[0]};
        ekf.step(input, y, 0.05);
        ukf.step(input, y, 0.05);
    }

    MRASIdentifier mras(1, 2.0);
    mras.setEstimate({0.1});
    mras.setAdaptationGain(2.5);
    mras.update(1.0, 0.2, {1.0}, 0.1);

    EXPECT_NEAR(ekf.state()[1], 0.5, 0.35);
    EXPECT_NEAR(ukf.state()[1], 0.5, 0.35);
    EXPECT_GT(mras.estimate()[0], 0.1);
    EXPECT_GT(mras.lastError(), 0.0);
    EXPECT_EQ(ekf.covariance().size(), 2u);
    EXPECT_EQ(ukf.covariance().size(), 2u);
}

TEST(NonlinearIdentificationTest, CoversNonlinearitiesIdentifierAndEtfeGuards) {
    StaticNonlinearity linear;
    EXPECT_DOUBLE_EQ(linear.apply(2.0), 2.0);

    StaticNonlinearity saturation;
    saturation.type = StaticNonlinearityType::Saturation;
    saturation.lower_limit = -1.0;
    saturation.upper_limit = 1.0;
    EXPECT_DOUBLE_EQ(saturation.apply(2.0), 1.0);

    StaticNonlinearity deadzone;
    deadzone.type = StaticNonlinearityType::DeadZone;
    deadzone.deadzone = 0.5;
    EXPECT_DOUBLE_EQ(deadzone.apply(0.25), 0.0);
    EXPECT_DOUBLE_EQ(deadzone.apply(1.0), 0.5);
    EXPECT_DOUBLE_EQ(deadzone.apply(-1.0), -0.5);

    StaticNonlinearity cubic;
    cubic.type = StaticNonlinearityType::Cubic;
    cubic.cubic = 0.1;
    EXPECT_GT(cubic.apply(2.0), 2.0);

    StaticNonlinearity invalid;
    invalid.type = static_cast<StaticNonlinearityType>(999);
    EXPECT_DOUBLE_EQ(invalid.apply(1.25), 1.25);

    PolynomialModelOrders orders;
    orders.na = 1;
    orders.nb = 1;
    orders.nk = 1;

    Vector input(256, 0.0);
    for (size_t i = 0; i < input.size(); ++i) {
        input[i] = 1.5 * std::sin(2.0 * kPi * static_cast<double>(i) / 32.0);
    }

    Vector output(input.size(), 0.0);
    for (size_t i = 1; i < input.size(); ++i) {
        const double saturated = std::clamp(input[i - 1], -1.0, 1.0);
        output[i] = 0.6 * output[i - 1] + 0.8 * saturated;
    }

    const auto sat_model = HammersteinWienerIdentifier::identify(
        input, output, orders, StaticNonlinearityType::Saturation, StaticNonlinearityType::Linear);
    const auto deadzone_model = HammersteinWienerIdentifier::identify(
        input, output, orders, StaticNonlinearityType::DeadZone, StaticNonlinearityType::Saturation);
    const auto cubic_model = HammersteinWienerIdentifier::identify(
        input, output, orders, StaticNonlinearityType::Cubic, StaticNonlinearityType::DeadZone);

    EXPECT_TRUE(sat_model.linear_block.valid());
    EXPECT_FALSE(sat_model.simulate(input).empty());
    EXPECT_TRUE(deadzone_model.linear_block.valid());
    EXPECT_TRUE(cubic_model.linear_block.valid());
    EXPECT_GT(sat_model.linear_block.fit, 40.0);

    HammersteinWienerModel manual_model;
    manual_model.input_non_linearity = saturation;
    manual_model.output_non_linearity = linear;
    manual_model.linear_block = ARXIdentifier::identify(input, output, orders);
    EXPECT_EQ(manual_model.simulate(input).size(), input.size());

    EXPECT_TRUE(ETFEEstimator::estimate(Vector{1.0}, Vector{1.0}, 0.01).frequencies.empty());
    EXPECT_TRUE(ETFEEstimator::estimate(input, output, 0.0).frequencies.empty());

    const auto etfe = ETFEEstimator::estimate(input, output, 0.01);
    EXPECT_FALSE(etfe.frequencies.empty());
    EXPECT_EQ(etfe.frequencies.size(), etfe.magnitude.size());
    EXPECT_EQ(etfe.frequencies.size(), etfe.phase.size());
    EXPECT_EQ(etfe.frequencies.size(), etfe.coherence.size());
}

TEST(IdentificationInternalTest, CoversHelperGuardPaths) {
    using namespace Identification::detail;

    EXPECT_DOUBLE_EQ(percentileAbs({}, 0.5), 0.0);
    EXPECT_TRUE(covariance({}).empty());
    EXPECT_TRUE(inverseSqrtSymmetric({}).empty());
    EXPECT_EQ(concatHorizontal({}, {{1.0}})[0][0], 1.0);
    EXPECT_EQ(concatHorizontal({{2.0}}, {})[0][0], 2.0);
    EXPECT_TRUE(leastSquaresMatrix({}, {{1.0}}).empty());
    EXPECT_EQ(extractWindow({{1.0, 2.0}, {3.0, 4.0}}, 0, 2).size(), 4u);
    EXPECT_DOUBLE_EQ(computeFitPercent({}, {1.0}), 0.0);
    EXPECT_TRUE(choleskyLikeSqrt({}, 1.0).empty());
}