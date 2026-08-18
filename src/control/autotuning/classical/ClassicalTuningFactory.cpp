#include "tether/control/autotuning/ClassicalTuningMethods.hpp"
#include "tether/control/autotuning/classical/ZieglerNicholsStepResponse.hpp"
#include "tether/control/autotuning/classical/ZieglerNicholsUltimateCycle.hpp"
#include "tether/control/autotuning/classical/TyreusLuyben.hpp"
#include "tether/control/autotuning/classical/CohenCoon.hpp"
#include "tether/control/autotuning/classical/ChienHronesReswick.hpp"
#include "tether/control/autotuning/classical/LopezMethod.hpp"
#include "tether/control/autotuning/classical/LambdaTuning.hpp"
#include "tether/control/autotuning/classical/SIMCMethod.hpp"
#include "tether/control/autotuning/classical/AMIGOMethod.hpp"
#include "tether/control/autotuning/classical/AstromHagglundRelay.hpp"

namespace tether::control {
namespace Autotuning {

std::unique_ptr<AutotunerBase> ClassicalTuningFactory::create(Method method) {
    switch (method) {
        case Method::ZieglerNicholsStep: return std::make_unique<ZieglerNicholsStepResponse>();
        case Method::ZieglerNicholsUltimate: return std::make_unique<ZieglerNicholsUltimateCycle>();
        case Method::TyreusLuyben: return std::make_unique<TyreusLuyben>();
        case Method::CohenCoon: return std::make_unique<CohenCoon>();
        case Method::CHR_SetpointNoOS: {
            auto tuner = std::make_unique<ChienHronesReswick>(); tuner->setTuningMode(ChienHronesReswick::Mode::SetpointNoOvershoot); return tuner; }
        case Method::CHR_Setpoint20OS: {
            auto tuner = std::make_unique<ChienHronesReswick>(); tuner->setTuningMode(ChienHronesReswick::Mode::Setpoint20Overshoot); return tuner; }
        case Method::CHR_RegulatorNoOS: {
            auto tuner = std::make_unique<ChienHronesReswick>(); tuner->setTuningMode(ChienHronesReswick::Mode::RegulatorNoOvershoot); return tuner; }
        case Method::CHR_Regulator20OS: {
            auto tuner = std::make_unique<ChienHronesReswick>(); tuner->setTuningMode(ChienHronesReswick::Mode::Regulator20Overshoot); return tuner; }
        case Method::LopezITAE: { auto tuner = std::make_unique<LopezMethod>(); tuner->setCriterion(LopezMethod::Criterion::ITAE); return tuner; }
        case Method::LopezIAE: { auto tuner = std::make_unique<LopezMethod>(); tuner->setCriterion(LopezMethod::Criterion::IAE); return tuner; }
        case Method::LopezISE: { auto tuner = std::make_unique<LopezMethod>(); tuner->setCriterion(LopezMethod::Criterion::ISE); return tuner; }
        case Method::Lambda: return std::make_unique<LambdaTuning>();
        case Method::SIMC: return std::make_unique<SIMCMethod>();
        case Method::AMIGO: return std::make_unique<AMIGOMethod>();
        case Method::RelayFeedback: return std::make_unique<AstromHagglundRelay>();
        default: return nullptr;
    }
}

std::vector<ClassicalTuningFactory::Method> ClassicalTuningFactory::getAvailableMethods() {
    return { Method::ZieglerNicholsStep, Method::ZieglerNicholsUltimate, Method::TyreusLuyben, Method::CohenCoon, Method::CHR_SetpointNoOS, Method::CHR_Setpoint20OS, Method::CHR_RegulatorNoOS, Method::CHR_Regulator20OS, Method::LopezITAE, Method::LopezIAE, Method::LopezISE, Method::Lambda, Method::SIMC, Method::AMIGO, Method::RelayFeedback };
}

std::string ClassicalTuningFactory::getMethodName(Method method) {
    switch(method) {
        case Method::ZieglerNicholsStep: return "Ziegler-Nichols Step Response";
        case Method::ZieglerNicholsUltimate: return "Ziegler-Nichols Ultimate Cycle";
        case Method::TyreusLuyben: return "Tyreus-Luyben";
        case Method::CohenCoon: return "Cohen-Coon";
        case Method::CHR_SetpointNoOS: return "CHR Setpoint 0% OS";
        case Method::CHR_Setpoint20OS: return "CHR Setpoint 20% OS";
        case Method::CHR_RegulatorNoOS: return "CHR Regulator 0% OS";
        case Method::CHR_Regulator20OS: return "CHR Regulator 20% OS";
        case Method::LopezITAE: return "Lopez ITAE";
        case Method::LopezIAE: return "Lopez IAE";
        case Method::LopezISE: return "Lopez ISE";
        case Method::Lambda: return "Lambda/IMC";
        case Method::SIMC: return "SIMC (Skogestad)";
        case Method::AMIGO: return "AMIGO";
        case Method::RelayFeedback: return "Relay Feedback";
        default: return "Unknown";
    }
}

} // namespace Autotuning
} // namespace tether::control
