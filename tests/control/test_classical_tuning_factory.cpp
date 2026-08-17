#include <gtest/gtest.h>

#include "tether/control/autotuning/ClassicalTuningMethods.hpp"

using tether::control::Autotuning::ClassicalTuningFactory;

TEST(ClassicalTuningFactory, AvailableMethodsAreStableAndNamesAreNonEmpty) {
    auto methods = ClassicalTuningFactory::getAvailableMethods();
    ASSERT_FALSE(methods.empty());

    for (auto m : methods) {
        auto name = ClassicalTuningFactory::getMethodName(m);
        EXPECT_FALSE(name.empty());
        auto tuner = ClassicalTuningFactory::create(m);
        EXPECT_NE(tuner, nullptr);
    }
}

TEST(ClassicalTuningFactory, UnknownMethodReturnsNullAndUnknownName) {
    auto unknown = static_cast<ClassicalTuningFactory::Method>(999);
    EXPECT_EQ(ClassicalTuningFactory::create(unknown), nullptr);
    EXPECT_EQ(ClassicalTuningFactory::getMethodName(unknown), "Unknown");
}
