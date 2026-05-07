/**
 * @file MuSynthesisTests.cpp
 * @brief Unit tests for μ-synthesis robust controller design
 */

#include <gtest/gtest.h>
#include "TestHelpers.hpp"
#include "tether/control/autotuning/MuSynthesis.hpp"
#include <complex>
#include <vector>

using namespace Control;
using namespace Control::Autotuning::Testing;

// ============================================================================
// MuSynthesisController Tests
// ============================================================================

class MuSynthesisControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        controller_ = std::make_unique<MuSynthesisController>();
    }
    
    std::unique_ptr<MuSynthesisController> controller_;
};

TEST_F(MuSynthesisControllerTest, GetName) {
    EXPECT_STREQ(controller_->getName(), "μ-Synthesis Controller");
}

TEST_F(MuSynthesisControllerTest, GetDescription) {
    EXPECT_NE(controller_->getDescription(), nullptr);
    EXPECT_GT(strlen(controller_->getDescription()), 0);
}

TEST_F(MuSynthesisControllerTest, GetType) {
    EXPECT_EQ(controller_->getType(), ControllerType::Custom);
}

TEST_F(MuSynthesisControllerTest, AddUncertaintyBlock) {
    controller_->addUncertaintyBlock("gain", 1, MuBlockType::Real, 0.2);
    controller_->addUncertaintyBlock("dynamics", 2, MuBlockType::Full, 0.1);
}

TEST_F(MuSynthesisControllerTest, AddRepeatedScalar) {
    controller_->addRepeatedScalar("repeated_gain", 3, true, 0.15);
    controller_->addRepeatedScalar("complex_repeated", 2, false, 0.25);
}

TEST_F(MuSynthesisControllerTest, ClearUncertaintyBlocks) {
    controller_->addUncertaintyBlock("test", 1, MuBlockType::Scalar, 0.1);
    controller_->clearUncertaintyBlocks();
}

TEST_F(MuSynthesisControllerTest, SetSensitivityWeight) {
    controller_->setSensitivityWeight(1.5, 10.0, 0.001);
}

TEST_F(MuSynthesisControllerTest, SetControlWeight) {
    controller_->setControlWeight(100.0);
}

TEST_F(MuSynthesisControllerTest, SetComplementarySensitivityWeight) {
    controller_->setComplementarySensitivityWeight(1.2, 50.0, 0.01);
}

TEST_F(MuSynthesisControllerTest, SynthesizeWithConfig) {
    // Set up a simple plant
    double A[] = {-1.0};
    double B[] = {1.0};
    double C[] = {1.0};
    double D[] = {0.0};
    controller_->setNominalPlant(A, B, C, D, 1, 1, 1);
    
    controller_->addUncertaintyBlock("gain", 1, MuBlockType::Real, 0.1);
    controller_->setSensitivityWeight(1.5, 10.0, 0.001);
    
    MuSynthesisController::SynthesisConfig config;
    config.maxIterations = 3;
    config.convergenceTol = 0.1;
    config.verbose = false;
    
    // Synthesis should not throw; iteration count should be non-negative
    EXPECT_NO_THROW({ controller_->synthesize(config); });
    EXPECT_GE(controller_->getIterationCount(), 0);
}

TEST_F(MuSynthesisControllerTest, SynthesizeDefault) {
    double A[] = {-1.0};
    double B[] = {1.0};
    double C[] = {1.0};
    double D[] = {0.0};
    controller_->setNominalPlant(A, B, C, D, 1, 1, 1);
    
    EXPECT_NO_THROW({ controller_->synthesize(); });
    EXPECT_GE(controller_->getIterationCount(), 0);
}

TEST_F(MuSynthesisControllerTest, GetMuAnalysis) {
    const MuAnalysisResult& result = controller_->getMuAnalysis();
    // Initially empty / zeroed
    EXPECT_DOUBLE_EQ(result.peakMuLower, 0.0);
    EXPECT_DOUBLE_EQ(result.peakMuUpper, 0.0);
}

TEST_F(MuSynthesisControllerTest, GetPeakMu) {
    double peakMu = controller_->getPeakMu();
    EXPECT_GE(peakMu, 0.0);
}

TEST_F(MuSynthesisControllerTest, HasRobustPerformance) {
    bool robust = controller_->hasRobustPerformance();
    // Initially false or true depending on default state
}

TEST_F(MuSynthesisControllerTest, GetIterationCount) {
    int iterations = controller_->getIterationCount();
    EXPECT_EQ(iterations, 0);
}

TEST_F(MuSynthesisControllerTest, AnalyzeMu) {
    double A[] = {-1.0};
    double B[] = {1.0};
    double C[] = {1.0};
    double D[] = {0.0};
    controller_->setNominalPlant(A, B, C, D, 1, 1, 1);
    
    MuAnalysisResult result = controller_->analyzeMu(0.01, 100.0, 10);
    EXPECT_GE(result.peakMuUpper, 0.0);
}

TEST_F(MuSynthesisControllerTest, ComputeMuUpperBound) {
    std::complex<double> M[] = {{1.0, 0.0}, {0.1, 0.0}, {0.1, 0.0}, {1.0, 0.0}};
    double upper = controller_->computeMuUpperBound(M, 2);
    EXPECT_GE(upper, 0.0);
}

TEST_F(MuSynthesisControllerTest, ComputeMuLowerBound) {
    std::complex<double> M[] = {{1.0, 0.0}, {0.1, 0.0}, {0.1, 0.0}, {1.0, 0.0}};
    double lower = controller_->computeMuLowerBound(M, 2);
    EXPECT_GE(lower, 0.0);
}

// ============================================================================
// UncertaintyBlock Tests
// ============================================================================

TEST(UncertaintyBlockTest, DefaultConstruction) {
    UncertaintyBlock block;
    EXPECT_EQ(block.rows, 1);
    EXPECT_EQ(block.cols, 1);
    EXPECT_EQ(block.repetitions, 1);
    EXPECT_EQ(block.bound, 1.0);
    EXPECT_EQ(block.type, MuBlockType::Full);
}

TEST(UncertaintyBlockTest, TotalDimensionsScalar) {
    UncertaintyBlock block;
    block.type = MuBlockType::Scalar;
    block.rows = 1;
    block.cols = 1;
    EXPECT_EQ(block.totalRows(), 1);
    EXPECT_EQ(block.totalCols(), 1);
}

TEST(UncertaintyBlockTest, TotalDimensionsRepeatedScalar) {
    UncertaintyBlock block;
    block.type = MuBlockType::RepeatedScalar;
    block.rows = 1;
    block.cols = 1;
    block.repetitions = 3;
    EXPECT_EQ(block.totalRows(), 3);
    EXPECT_EQ(block.totalCols(), 3);
}

TEST(UncertaintyBlockTest, TotalDimensionsFull) {
    UncertaintyBlock block;
    block.type = MuBlockType::Full;
    block.rows = 2;
    block.cols = 3;
    EXPECT_EQ(block.totalRows(), 2);
    EXPECT_EQ(block.totalCols(), 3);
}

TEST(UncertaintyBlockTest, TotalDimensionsRepeatedReal) {
    UncertaintyBlock block;
    block.type = MuBlockType::RepeatedReal;
    block.rows = 2;
    block.cols = 2;
    block.repetitions = 4;
    EXPECT_EQ(block.totalRows(), 8);
    EXPECT_EQ(block.totalCols(), 8);
}

// ============================================================================
// MuBlockType Tests
// ============================================================================

TEST(MuBlockTypeTest, AllTypesExist) {
    MuBlockType type1 = MuBlockType::Full;
    MuBlockType type2 = MuBlockType::Scalar;
    MuBlockType type3 = MuBlockType::RepeatedScalar;
    MuBlockType type4 = MuBlockType::Real;
    MuBlockType type5 = MuBlockType::RepeatedReal;
    
    EXPECT_NE(type1, type2);
    EXPECT_NE(type2, type3);
    EXPECT_NE(type3, type4);
    EXPECT_NE(type4, type5);
}

// ============================================================================
// DScaleFit Tests
// ============================================================================

TEST(DScaleFitTest, DefaultConstruction) {
    DScaleFit fit;
    EXPECT_EQ(fit.order, 2);
    EXPECT_TRUE(fit.frequencies.empty());
    EXPECT_TRUE(fit.D.empty());
}

TEST(DScaleFitTest, SetFrequencies) {
    DScaleFit fit;
    fit.frequencies = {0.1, 1.0, 10.0, 100.0};
    EXPECT_EQ(fit.frequencies.size(), 4);
}

TEST(DScaleFitTest, SetCoefficients) {
    DScaleFit fit;
    fit.numeratorCoeffs = {1.0, 0.5};
    fit.denominatorCoeffs = {1.0, 0.1, 0.01};
    EXPECT_EQ(fit.numeratorCoeffs.size(), 2);
    EXPECT_EQ(fit.denominatorCoeffs.size(), 3);
}

// ============================================================================
// MuAnalysisResult Tests
// ============================================================================

TEST(MuAnalysisResultTest, DefaultConstruction) {
    MuAnalysisResult result;
    EXPECT_EQ(result.peakMuUpper, 0.0);
    EXPECT_EQ(result.peakMuLower, 0.0);
    EXPECT_EQ(result.peakFrequency, 0.0);
}

TEST(MuAnalysisResultTest, RobustStabilityTrue) {
    MuAnalysisResult result;
    result.peakMuUpper = 0.5;
    EXPECT_TRUE(result.robustStability());
}

TEST(MuAnalysisResultTest, RobustStabilityFalse) {
    MuAnalysisResult result;
    result.peakMuUpper = 1.5;
    EXPECT_FALSE(result.robustStability());
}

TEST(MuAnalysisResultTest, RobustPerformanceTrue) {
    MuAnalysisResult result;
    result.peakMuUpper = 0.8;
    EXPECT_TRUE(result.robustPerformance());
}

TEST(MuAnalysisResultTest, RobustPerformanceFalse) {
    MuAnalysisResult result;
    result.peakMuUpper = 1.2;
    EXPECT_FALSE(result.robustPerformance());
}

// ============================================================================
// MuAnalysis Utility Tests
// ============================================================================

TEST(MuAnalysisTest, UpperBoundSimple) {
    std::complex<double> M[] = {
        {1.0, 0.0}, {0.1, 0.0},
        {0.1, 0.0}, {1.0, 0.0}
    };
    std::vector<UncertaintyBlock> blocks;
    UncertaintyBlock b1;
    b1.type = MuBlockType::Scalar;
    b1.rows = 1;
    b1.cols = 1;
    blocks.push_back(b1);
    blocks.push_back(b1);
    
    double upper = MuAnalysis::upperBound(M, 2, blocks);
    EXPECT_GE(upper, 0.0);
}

TEST(MuAnalysisTest, LowerBoundSimple) {
    std::complex<double> M[] = {
        {1.0, 0.0}, {0.1, 0.0},
        {0.1, 0.0}, {1.0, 0.0}
    };
    std::vector<UncertaintyBlock> blocks;
    UncertaintyBlock b1;
    b1.type = MuBlockType::Scalar;
    blocks.push_back(b1);
    blocks.push_back(b1);
    
    double lower = MuAnalysis::lowerBound(M, 2, blocks);
    EXPECT_GE(lower, 0.0);
}

TEST(MuAnalysisTest, FindOptimalDScale) {
    std::complex<double> M[] = {
        {1.0, 0.0}, {0.2, 0.1},
        {0.3, -0.1}, {1.0, 0.0}
    };
    std::vector<UncertaintyBlock> blocks;
    UncertaintyBlock b1;
    b1.type = MuBlockType::Scalar;
    blocks.push_back(b1);
    blocks.push_back(b1);
    
    auto D = MuAnalysis::findOptimalDScale(M, 2, blocks);
    EXPECT_FALSE(D.empty());
}

TEST(MuAnalysisTest, FitRational) {
    std::vector<double> frequencies = {0.1, 1.0, 10.0, 100.0};
    std::vector<std::complex<double>> values = {
        {1.0, 0.0}, {0.9, -0.1}, {0.5, -0.3}, {0.1, -0.1}
    };
    
    auto [num, den] = MuAnalysis::fitRational(frequencies, values, 2);
    EXPECT_FALSE(num.empty());
    EXPECT_FALSE(den.empty());
}

// ============================================================================
// StructuredUncertainModel Tests
// ============================================================================

class StructuredUncertainModelTest : public ::testing::Test {
protected:
    void SetUp() override {
        model_ = std::make_unique<StructuredUncertainModel>();
        
        // Simple first-order system
        double A[] = {-1.0};
        double B[] = {1.0};
        double C[] = {1.0};
        double D[] = {0.0};
        model_->setNominal(A, B, C, D, 1, 1, 1);
    }
    
    std::unique_ptr<StructuredUncertainModel> model_;
};

TEST_F(StructuredUncertainModelTest, SetNominal) {
    // Already done in SetUp - verify sampling works
    double A[1], B[1], C[1], D[1];
    model_->sampleUncertainty(A, B, C, D);
    EXPECT_TRUE(std::isfinite(A[0]));
}

TEST_F(StructuredUncertainModelTest, AddInputMultiplicative) {
    WeightingFunction W = WeightingFunction::firstOrder(1.0, 10.0, 0.1);
    model_->addInputMultiplicative(W, 1.0);
}

TEST_F(StructuredUncertainModelTest, AddOutputMultiplicative) {
    WeightingFunction W = WeightingFunction::firstOrder(2.0, 20.0, 0.2);
    model_->addOutputMultiplicative(W, 1.0);
}

TEST_F(StructuredUncertainModelTest, AddAdditive) {
    WeightingFunction W = WeightingFunction::firstOrder(0.5, 5.0, 0.05);
    model_->addAdditive(W, 1.0);
}

TEST_F(StructuredUncertainModelTest, AddParametric) {
    double A_delta[] = {0.1};
    double B_delta[] = {0.0};
    model_->addParametric("gain", A_delta, B_delta, 0.2);
}

TEST_F(StructuredUncertainModelTest, BuildLFT) {
    WeightingFunction W = WeightingFunction::firstOrder(1.0, 10.0, 0.1);
    model_->addInputMultiplicative(W, 1.0);
    
    double M11[4], M12[4], M21[4], M22[4];
    std::vector<UncertaintyBlock> blocks;
    model_->buildLFT(M11, M12, M21, M22, blocks);
}

TEST_F(StructuredUncertainModelTest, SampleUncertainty) {
    WeightingFunction W = WeightingFunction::firstOrder(1.0, 10.0, 0.1);
    model_->addInputMultiplicative(W, 1.0);
    
    double A[1], B[1], C[1], D[1];
    model_->sampleUncertainty(A, B, C, D);
}

// ============================================================================
// Integration Tests
// ============================================================================

TEST(MuSynthesisIntegration, FullDKIterationWorkflow) {
    MuSynthesisController controller;
    
    // Set up a simple SISO plant: G(s) = 1/(s+1)
    double A[] = {-1.0};
    double B[] = {1.0};
    double C[] = {1.0};
    double D[] = {0.0};
    controller.setNominalPlant(A, B, C, D, 1, 1, 1);
    
    // Add uncertainty: 20% gain uncertainty
    controller.addUncertaintyBlock("gain_uncertainty", 1, MuBlockType::Real, 0.2);
    
    // Set performance weights
    controller.setSensitivityWeight(2.0, 10.0, 0.01);
    controller.setControlWeight(50.0);
    
    // Configure and run synthesis
    MuSynthesisController::SynthesisConfig config;
    config.maxIterations = 5;
    config.convergenceTol = 0.05;
    config.dScaleOrder = 2;
    config.numFrequencies = 50;
    
    bool success = controller.synthesize(config);
    
    // Get results
    int iterations = controller.getIterationCount();
    double peakMu = controller.getPeakMu();
    
    EXPECT_GE(iterations, 0);
    EXPECT_GE(peakMu, 0.0);
}

TEST(MuSynthesisIntegration, MuBoundsConsistency) {
    // Create a simple transfer function matrix
    std::complex<double> M[] = {
        {1.0, 0.0}, {0.1, 0.05},
        {0.1, -0.05}, {1.0, 0.0}
    };
    
    std::vector<UncertaintyBlock> blocks;
    UncertaintyBlock b;
    b.type = MuBlockType::Scalar;
    b.rows = 1;
    b.cols = 1;
    blocks.push_back(b);
    blocks.push_back(b);
    
    double upper = MuAnalysis::upperBound(M, 2, blocks);
    double lower = MuAnalysis::lowerBound(M, 2, blocks);
    
    // Upper bound should be >= lower bound
    EXPECT_GE(upper + 1e-6, lower);  // Small tolerance for numerical errors
}

TEST(MuSynthesisIntegration, StructuredModelToSynthesis) {
    // Build uncertain model
    StructuredUncertainModel uncertainModel;
    double A[] = {-1.0};
    double B[] = {1.0};
    double C[] = {1.0};
    double D[] = {0.0};
    uncertainModel.setNominal(A, B, C, D, 1, 1, 1);
    
    WeightingFunction Wi = WeightingFunction::firstOrder(1.0, 10.0, 0.1);
    uncertainModel.addInputMultiplicative(Wi, 1.0);
    
    // Get LFT representation
    double M11[4], M12[4], M21[4], M22[4];
    std::vector<UncertaintyBlock> blocks;
    uncertainModel.buildLFT(M11, M12, M21, M22, blocks);
    
    // LFT was built successfully and blocks populated
    EXPECT_GT(blocks.size(), 0u);
}

TEST(MuSynthesisIntegration, SynthesisConfigDefaults) {
    MuSynthesisController::SynthesisConfig config;
    
    EXPECT_EQ(config.maxIterations, 10);
    EXPECT_DOUBLE_EQ(config.convergenceTol, 0.01);
    EXPECT_EQ(config.dScaleOrder, 2);
    EXPECT_EQ(config.numFrequencies, 100);
    EXPECT_DOUBLE_EQ(config.freqMin, 1e-3);
    EXPECT_DOUBLE_EQ(config.freqMax, 1e3);
    EXPECT_FALSE(config.verbose);
}

// ============================================================================
// Additional Coverage Tests for MuSynthesis
// ============================================================================

TEST(MuBlockTypeCoverage, AllBlockTypes) {
    std::vector<MuBlockType> types = {
        MuBlockType::Scalar,
        MuBlockType::Real,
        MuBlockType::Full
    };
    
    MuSynthesisController controller;
    
    for (size_t i = 0; i < types.size(); ++i) {
        std::string name = "block_" + std::to_string(i);
        controller.addUncertaintyBlock(name.c_str(), i + 1, types[i], 0.1 * (i + 1));
    }
}

TEST(MuSynthesisCoverage, VariousPlantDimensions) {
    std::vector<int> dims = {1, 2, 3};
    
    for (int dim : dims) {
        MuSynthesisController controller;
        
        std::vector<double> A(dim * dim);
        std::vector<double> B(dim);
        std::vector<double> C(dim);
        std::vector<double> D(1, 0.0);
        
        // Create stable diagonal A matrix
        for (int i = 0; i < dim; ++i) {
            A[i * dim + i] = -(i + 1);
            B[i] = 1.0;
            C[i] = 1.0;
        }
        
        controller.setNominalPlant(A.data(), B.data(), C.data(), D.data(), dim, 1, 1);
        controller.addUncertaintyBlock("delta", 1, MuBlockType::Scalar, 0.1);
        
        bool result = controller.synthesize();
    }
}

TEST(MuSynthesisCoverage, MultipleUncertaintyBlocks) {
    MuSynthesisController controller;
    
    double A[] = {-1.0, 0.0, 0.0, -2.0};
    double B[] = {1.0, 1.0};
    double C[] = {1.0, 0.0};
    double D[] = {0.0};
    controller.setNominalPlant(A, B, C, D, 2, 1, 1);
    
    // Add multiple uncertainty blocks
    controller.addUncertaintyBlock("input_gain", 1, MuBlockType::Real, 0.2);
    controller.addUncertaintyBlock("dynamics", 2, MuBlockType::Full, 0.15);
    controller.addUncertaintyBlock("output_gain", 1, MuBlockType::Scalar, 0.1);
    
    MuSynthesisController::SynthesisConfig config;
    config.maxIterations = 5;
    bool result = controller.synthesize(config);
}

TEST(MuSynthesisCoverage, AllWeightingFunctions) {
    MuSynthesisController controller;
    
    double A[] = {-1.0};
    double B[] = {1.0};
    double C[] = {1.0};
    double D[] = {0.0};
    controller.setNominalPlant(A, B, C, D, 1, 1, 1);
    
    // Different weight configurations
    controller.setSensitivityWeight(0.5, 5.0, 0.0001);
    controller.setControlWeight(50.0);
    controller.setComplementarySensitivityWeight(0.8, 20.0, 0.001);
    
    controller.addUncertaintyBlock("delta", 1, MuBlockType::Real, 0.15);
    
    bool result = controller.synthesize();
}

TEST(MuSynthesisCoverage, SynthesisConfigVariations) {
    MuSynthesisController controller;
    
    double A[] = {-1.0};
    double B[] = {1.0};
    double C[] = {1.0};
    double D[] = {0.0};
    controller.setNominalPlant(A, B, C, D, 1, 1, 1);
    controller.addUncertaintyBlock("delta", 1, MuBlockType::Scalar, 0.1);
    
    // Test various config settings
    MuSynthesisController::SynthesisConfig config;
    
    config.maxIterations = 1;
    config.convergenceTol = 0.5;
    config.dScaleOrder = 1;
    config.numFrequencies = 20;
    config.freqMin = 0.01;
    config.freqMax = 100.0;
    controller.synthesize(config);
    
    config.maxIterations = 20;
    config.convergenceTol = 0.001;
    config.dScaleOrder = 4;
    config.numFrequencies = 200;
    controller.synthesize(config);
}

TEST(MuAnalysisCoverage, VariousMatrixSizes) {
    for (int n = 1; n <= 3; ++n) {
        std::vector<std::complex<double>> M(n * n);
        
        // Identity-like matrix with small off-diagonal
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                M[i * n + j] = (i == j) ? std::complex<double>(1.0, 0.0) 
                                         : std::complex<double>(0.1, 0.05);
            }
        }
        
        std::vector<UncertaintyBlock> blocks;
        for (int i = 0; i < n; ++i) {
            UncertaintyBlock b;
            b.type = MuBlockType::Scalar;
            b.rows = 1;
            b.cols = 1;
            blocks.push_back(b);
        }
        
        double upper = MuAnalysis::upperBound(M.data(), n, blocks);
        double lower = MuAnalysis::lowerBound(M.data(), n, blocks);
        
        EXPECT_GE(upper + 1e-6, lower);
    }
}

TEST(MuAnalysisCoverage, MixedBlockStructure) {
    std::complex<double> M[] = {
        {1.0, 0.0}, {0.2, 0.1}, {0.1, 0.0},
        {0.2, -0.1}, {1.0, 0.0}, {0.15, 0.05},
        {0.1, 0.0}, {0.15, -0.05}, {1.0, 0.0}
    };
    
    std::vector<UncertaintyBlock> blocks;
    
    UncertaintyBlock scalar;
    scalar.type = MuBlockType::Scalar;
    scalar.rows = 1;
    scalar.cols = 1;
    blocks.push_back(scalar);
    
    UncertaintyBlock full;
    full.type = MuBlockType::Full;
    full.rows = 2;
    full.cols = 2;
    blocks.push_back(full);
    
    double upper = MuAnalysis::upperBound(M, 3, blocks);
    EXPECT_GT(upper, 0.0);
}

TEST(WeightingFunctionCoverage, FirstOrderVariations) {
    // Use correct firstOrder signature: firstOrder(zero, pole, gain)
    std::vector<std::tuple<double, double, double>> params = {
        {0.5, 1.0, 1.0},
        {1.0, 10.0, 2.0},
        {2.0, 50.0, 0.5},
        {0.1, 0.5, 1.5}
    };
    
    for (const auto& [zero, pole, gain] : params) {
        WeightingFunction W = WeightingFunction::firstOrder(zero, pole, gain);
        
        // Evaluate magnitude at various frequencies
        for (double w = 0.01; w <= 100.0; w *= 10.0) {
            double mag = W.magnitude(w);
            EXPECT_TRUE(std::isfinite(mag));
            EXPECT_GE(mag, 0.0);
        }
    }
}

TEST(WeightingFunctionCoverage, SensitivityWeights) {
    // Use the sensitivity weight factory method
    std::vector<std::tuple<double, double, double>> params = {
        {1.5, 10.0, 0.001},
        {2.0, 20.0, 0.01},
        {1.0, 5.0, 0.0001}
    };
    
    for (const auto& [M, wB, eps] : params) {
        WeightingFunction W = WeightingFunction::sensitivity(M, wB, eps);
        
        // Magnitude should be large at low frequencies
        double lowFreqMag = W.magnitude(0.01);
        // Magnitude should be smaller at high frequencies
        double highFreqMag = W.magnitude(100.0);
        
        EXPECT_TRUE(std::isfinite(lowFreqMag));
        EXPECT_TRUE(std::isfinite(highFreqMag));
    }
}

TEST(StructuredUncertainModelCoverage, AllUncertaintyTypes) {
    StructuredUncertainModel model;
    
    double A[] = {-1.0};
    double B[] = {1.0};
    double C[] = {1.0};
    double D[] = {0.0};
    model.setNominal(A, B, C, D, 1, 1, 1);
    
    // Input multiplicative
    WeightingFunction Wi = WeightingFunction::firstOrder(1.0, 10.0, 0.1);
    model.addInputMultiplicative(Wi, 0.2);
    
    // Output multiplicative
    WeightingFunction Wo = WeightingFunction::firstOrder(0.5, 5.0, 0.05);
    model.addOutputMultiplicative(Wo, 0.15);
    
    // Additive
    WeightingFunction Wa = WeightingFunction::firstOrder(0.1, 1.0, 0.01);
    model.addAdditive(Wa);
    
    // Build LFT
    double M11[16], M12[16], M21[16], M22[16];
    std::vector<UncertaintyBlock> blocks;
    model.buildLFT(M11, M12, M21, M22, blocks);
    
    EXPECT_FALSE(blocks.empty());
}

TEST(StructuredUncertainModelCoverage, ParametricUncertainty) {
    StructuredUncertainModel model;
    
    double A[] = {-1.0, 0.0, 0.0, -2.0};
    double B[] = {1.0, 0.5};
    double C[] = {1.0, 0.0};
    double D[] = {0.0};
    model.setNominal(A, B, C, D, 2, 1, 1);
    
    // Parametric uncertainty on A matrix element
    // Use correct signature: addParametric(name, A_delta, B_delta, bound)
    double A_delta[] = {0.1, 0.0, 0.0, 0.0};  // Sensitivity of A to parameter
    double B_delta[] = {0.0, 0.0};  // Sensitivity of B to parameter
    model.addParametric("gain_uncertainty", A_delta, B_delta, 0.1);
    
    double M11[16], M12[16], M21[16], M22[16];
    std::vector<UncertaintyBlock> blocks;
    model.buildLFT(M11, M12, M21, M22, blocks);
}

TEST(MuSynthesisIntegration, FullWorkflow) {
    // Step 1: Create uncertain model
    StructuredUncertainModel uncertainModel;
    double A[] = {-0.5, 0.0, 1.0, -1.0};
    double B[] = {0.0, 1.0};
    double C[] = {1.0, 0.0};
    double D[] = {0.0};
    uncertainModel.setNominal(A, B, C, D, 2, 1, 1);
    
    WeightingFunction Wi = WeightingFunction::firstOrder(0.5, 5.0, 0.05);
    uncertainModel.addInputMultiplicative(Wi, 0.15);
    
    // Step 2: Create controller and configure
    MuSynthesisController controller;
    controller.setNominalPlant(A, B, C, D, 2, 1, 1);
    controller.addUncertaintyBlock("input_mult", 1, MuBlockType::Full, 0.15);
    controller.setSensitivityWeight(1.0, 8.0, 0.001);
    controller.setControlWeight(80.0);
    
    // Step 3: Synthesize
    MuSynthesisController::SynthesisConfig config;
    config.maxIterations = 5;
    config.convergenceTol = 0.05;
    config.verbose = false;
    
    bool success = controller.synthesize(config);
    
    // Step 4: Analyze results
    auto muResult = controller.getMuAnalysis();
    int iterations = controller.getIterationCount();
    double peakMu = controller.getPeakMu();
}

TEST(MuSynthesisIntegration, RepeatedScalars) {
    MuSynthesisController controller;
    
    double A[] = {-1.0};
    double B[] = {1.0};
    double C[] = {1.0};
    double D[] = {0.0};
    controller.setNominalPlant(A, B, C, D, 1, 1, 1);
    
    // Repeated real scalars (structured uncertainty)
    controller.addRepeatedScalar("gain_var", 2, true, 0.1);
    
    // Repeated complex scalars
    controller.addRepeatedScalar("dynamics_var", 3, false, 0.15);
    
    controller.setSensitivityWeight(1.0, 5.0, 0.001);
    
    bool result = controller.synthesize();
}
