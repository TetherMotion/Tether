#include "tether/control/autotuning/classical/AstromHagglundRelay.hpp"
#include "tether/control/autotuning/classical/ZieglerNicholsUltimateCycle.hpp"
#include "tether/control/autotuning/classical/TyreusLuyben.hpp"
#include <numeric>
#include <cmath>

namespace Control {
namespace Autotuning {

std::string AstromHagglundRelay::getName() const { return "Åström-Hägglund Relay"; }
std::string AstromHagglundRelay::getDescription() const { return "Online autotuning using relay feedback."; }
bool AstromHagglundRelay::isCompatible(const TunableController& controller) const { return !controller.getParameterDescriptors().empty(); }

TuningResult AstromHagglundRelay::tune(TunableController& controller, const ProcessModel* model) {
    TuningResult result;
    if (m_state != State::Complete) { result.success = false; result.message = "Relay experiment not complete. Run online update first."; return result; }
    result.parameters = {m_gains.Kp, m_gains.Ki, m_gains.Kd};
    result.success = controller.setParameters(result.parameters);
    result.message = result.success ? "Relay feedback tuning successful" : "Failed to set parameters";
    return result;
}

double AstromHagglundRelay::update(double measured, double reference, double control, double dt) {
    if (m_state != State::Running) return 0.0;
    m_elapsed += dt;
    double error = reference - measured;
    if (error > m_config.hysteresis) m_relayOutput = m_config.relayAmplitude;
    else if (error < -m_config.hysteresis) m_relayOutput = -m_config.relayAmplitude;
    if (m_lastError * error < 0) {
        if (error > 0 && !m_peakTimes.empty()) m_valleyValues.push_back(measured);
        else if (error < 0 && !m_peakTimes.empty()) { m_peakValues.push_back(measured); m_peakTimes.push_back(m_elapsed); m_cycles++; }
        else if (m_peakTimes.empty()) m_peakTimes.push_back(m_elapsed);
    }
    m_lastError = error;

    if (m_cycles >= m_config.minCycles) {
        if (m_peakValues.size() >= 2 && m_valleyValues.size() >= 1) {
            double avgPeak = std::accumulate(m_peakValues.begin(), m_peakValues.end(), 0.0) / m_peakValues.size();
            double avgValley = std::accumulate(m_valleyValues.begin(), m_valleyValues.end(), 0.0) / m_valleyValues.size();
            double amplitude = (avgPeak - avgValley) / 2.0;
            if (std::abs(amplitude) < 1e-10) {
                m_state = State::Failed;
                return m_relayOutput;
            }
            double totalPeriod = 0;
            for (size_t i = 1; i < m_peakTimes.size(); ++i) totalPeriod += m_peakTimes[i] - m_peakTimes[i - 1];
            m_Tu = totalPeriod / (m_peakTimes.size() - 1);
            double d = m_config.relayAmplitude;
            m_Ku = 4.0 * d / (M_PI * amplitude);
            switch (m_rule) {
                case TuningRule::ZieglerNichols:
                    m_gains = ZieglerNicholsUltimateCycle::calculateGains(m_Ku, m_Tu, PIDForm::Parallel);
                    break;
                case TuningRule::TyreusLuyben:
                    m_gains = TyreusLuyben::calculateGains(m_Ku, m_Tu, false);
                    break;
                case TuningRule::AMIGO:
                    m_gains.Kp = 0.35 * m_Ku;
                    m_gains.Ti = 0.85 * m_Tu;
                    m_gains.Td = 0.15 * m_Tu;
                    m_gains.Ki = m_gains.Kp / m_gains.Ti;
                    m_gains.Kd = m_gains.Kp * m_gains.Td;
                    break;
            }
            m_state = State::Complete;
        }
    }
    if (m_cycles >= m_config.maxCycles) m_state = State::Failed;
    return m_relayOutput;
}

bool AstromHagglundRelay::isComplete() const { return m_state == State::Complete || m_state == State::Failed; }
void AstromHagglundRelay::start() { m_state = State::Running; m_setpoint = 0.0; m_relayOutput = m_config.relayAmplitude; m_lastError = 0.0; m_peakTimes.clear(); m_peakValues.clear(); m_valleyValues.clear(); m_elapsed = 0.0; m_cycles = 0; m_Ku = 0.0; m_Tu = 0.0; }
void AstromHagglundRelay::stop() { m_state = State::Idle; }
TuningResult AstromHagglundRelay::getIntermediateResult() const { TuningResult result; result.success = (m_state == State::Complete); result.parameters = {m_gains.Kp, m_gains.Ki, m_gains.Kd}; result.iterations = m_cycles; result.message = result.success ? "Relay identification complete" : "Relay identification in progress"; return result; }

} // namespace Autotuning
} // namespace Control
