#include <gtest/gtest.h>

#include <cmath>

#include <tether/identification/ModelIdentification.hpp>

using namespace Identification;

namespace {

Vector makeStepInput(size_t count, double step_value) {
    Vector input(count, 0.0);
    for (size_t i = count / 8; i < count; ++i) {
        input[i] = step_value;
    }
    return input;
}

Vector simulateARXPlant(const Vector& input, double a1, double b1) {
    Vector output(input.size(), 0.0);
    for (size_t i = 1; i < input.size(); ++i) {
        output[i] = -a1 * output[i - 1] + b1 * input[i - 1];
    }
    return output;
}

} // namespace

TEST(PolynomialModelsTest, IdentifiersFitStablePlant) {
    const Vector input = makeStepInput(240, 1.0);
    const Vector output = simulateARXPlant(input, -0.72, 0.45);

    PolynomialModelOrders orders;
    orders.na = 1;
    orders.nb = 1;
    orders.nc = 1;
    orders.nd = 1;
    orders.nf = 1;
    orders.nk = 1;
    orders.iterations = 6;

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
    EXPECT_GT(arx.fit, 80.0);
    EXPECT_GT(iv.fit, 70.0);
    EXPECT_GT(armax.fit, 70.0);
    EXPECT_GT(oe.fit, 60.0);
    EXPECT_GT(bj.fit, 60.0);
}

TEST(SubspaceIdentificationTest, IdentifiersReturnStateSpaceModels) {
    Matrix inputs;
    Matrix outputs;
    Vector x{0.0, 0.0};

    for (size_t k = 0; k < 160; ++k) {
        const Vector u{
            std::sin(0.05 * static_cast<double>(k)),
            std::cos(0.035 * static_cast<double>(k))
        };
        const Vector y{
            x[0] + 0.2 * u[0],
            x[1] + 0.1 * u[1]
        };
        inputs.push_back(u);
        outputs.push_back(y);

        const double next_x0 = 0.85 * x[0] + 0.1 * x[1] + 0.4 * u[0];
        const double next_x1 = -0.05 * x[0] + 0.9 * x[1] + 0.3 * u[1];
        x[0] = next_x0;
        x[1] = next_x1;
    }

    const auto n4sid = N4SIDIdentifier::identify(inputs, outputs, 6, 2);
    const auto moesp = MOESPIdentifier::identify(inputs, outputs, 6, 2);
    const auto cva = CVAIdentifier::identify(inputs, outputs, 6, 2);

    EXPECT_TRUE(n4sid.valid());
    EXPECT_TRUE(moesp.valid());
    EXPECT_TRUE(cva.valid());
    EXPECT_EQ(n4sid.B.front().size(), 2u);
    EXPECT_EQ(moesp.C.size(), 2u);
    EXPECT_EQ(cva.order, 2u);
    EXPECT_GT(n4sid.fit, 10.0);
    EXPECT_GT(moesp.fit, 10.0);
    EXPECT_GT(cva.fit, 10.0);
}

TEST(AdvancedFrictionModelsTest, DynamicModelsProduceFiniteFits) {
    std::vector<DynamicFrictionSample> samples;
    double displacement = 0.0;
    for (size_t i = 0; i < 200; ++i) {
        const double t = 0.01 * static_cast<double>(i);
        const double velocity = std::sin(2.0 * M_PI * 0.4 * t);
        displacement += 0.01 * velocity;
        const double force = 1.5 * displacement + 3.0 * std::tanh(4.0 * velocity);
        samples.push_back({displacement, velocity, force, 0.01});
    }

    const auto lugre = LuGreIdentifier::identify(samples);
    const auto dahl = DahlIdentifier::identify(samples);
    const auto bouc_wen = BoucWenIdentifier::identify(samples);
    const auto preisach = PreisachIdentifier::identify(samples, 6);

    EXPECT_TRUE(std::isfinite(lugre.fit));
    EXPECT_TRUE(std::isfinite(dahl.fit));
    EXPECT_TRUE(std::isfinite(bouc_wen.fit));
    EXPECT_TRUE(std::isfinite(preisach.fit));
    EXPECT_GT(lugre.fit, 20.0);
    EXPECT_GT(dahl.fit, 20.0);
    EXPECT_GT(bouc_wen.fit, 20.0);
    EXPECT_GT(preisach.fit, 20.0);
}

TEST(RigidBodyIdentificationTest, BaseParametersAndTrajectoriesAreGenerated) {
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

TEST(AdaptiveObserversTest, EkfUkfAndMrasUpdateEstimates) {
    const auto transition = [](const Vector& state, const Vector& input, double dt) {
        return Vector{
            state[0] + dt * state[1] * input[0],
            state[1]
        };
    };
    const auto measurement = [](const Vector& state) {
        return Vector{state[0]};
    };

    AugmentedStateEKF ekf(2, 1);
    ekf.setModel(transition, measurement);
    ekf.setState({0.0, 0.2});
    ekf.setCovariance(identityMatrix(2));
    ekf.setProcessNoise(makeMatrix(2, 2, 0.0));
    ekf.setMeasurementNoise(makeMatrix(1, 1, 1e-3));
    ekf.setProcessNoise({{1e-4, 0.0}, {0.0, 1e-5}});

    AugmentedStateUKF ukf(2, 1);
    ukf.setModel(transition, measurement);
    ukf.setState({0.0, 0.2});
    ukf.setCovariance(identityMatrix(2));
    ukf.setMeasurementNoise(makeMatrix(1, 1, 1e-3));
    ukf.setProcessNoise({{1e-4, 0.0}, {0.0, 1e-5}});

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
    mras.update(1.0, 0.2, {1.0}, 0.1);

    EXPECT_NEAR(ekf.state()[1], 0.5, 0.35);
    EXPECT_NEAR(ukf.state()[1], 0.5, 0.35);
    EXPECT_GT(mras.estimate()[0], 0.1);
}

TEST(NonlinearIdentificationTest, HammersteinWienerAndEtfeProduceResults) {
    Vector input(256, 0.0);
    for (size_t i = 0; i < input.size(); ++i) {
        input[i] = 1.5 * std::sin(2.0 * M_PI * static_cast<double>(i) / 32.0);
    }

    Vector output(input.size(), 0.0);
    for (size_t i = 1; i < input.size(); ++i) {
        const double saturated = std::clamp(input[i - 1], -1.0, 1.0);
        output[i] = 0.6 * output[i - 1] + 0.8 * saturated;
    }

    PolynomialModelOrders orders;
    orders.na = 1;
    orders.nb = 1;
    orders.nk = 1;

    const auto model = HammersteinWienerIdentifier::identify(
        input, output, orders, StaticNonlinearityType::Saturation, StaticNonlinearityType::Linear);
    const auto etfe = ETFEEstimator::estimate(input, output, 0.01);

    EXPECT_TRUE(model.linear_block.valid());
    EXPECT_FALSE(model.simulate(input).empty());
    EXPECT_FALSE(etfe.frequencies.empty());
    EXPECT_EQ(etfe.frequencies.size(), etfe.magnitude.size());
    EXPECT_GT(model.linear_block.fit, 40.0);
}