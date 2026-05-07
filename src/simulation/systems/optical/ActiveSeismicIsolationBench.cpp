#include "tether/simulation/systems/optical/ActiveSeismicIsolationBench.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

ActiveSeismicIsolationBench::ActiveSeismicIsolationBench() {
    initParam("m1", 120.0);
    initParam("m2", 80.0);
    initParam("m3", 40.0);
    initParam("k1", 4.5e4);
    initParam("k12", 2.8e4);
    initParam("k23", 1.8e4);
    initParam("c1", 220.0);
    initParam("c12", 150.0);
    initParam("c23", 90.0);
    initParam("ground_low_amp", 0.0005);
    initParam("ground_low_hz", 0.8);
    initParam("ground_high_amp", 2.0e-5);
    initParam("ground_high_hz", 18.0);
}

StateVector ActiveSeismicIsolationBench::dynamics(double t, const StateVector& s, const StateVector& u) const {
    const double m1 = params_.at("m1");
    const double m2 = params_.at("m2");
    const double m3 = params_.at("m3");
    const double k1 = params_.at("k1");
    const double k12 = params_.at("k12");
    const double k23 = params_.at("k23");
    const double c1 = params_.at("c1");
    const double c12 = params_.at("c12");
    const double c23 = params_.at("c23");
    const double ground_low_amp = params_.at("ground_low_amp");
    const double ground_low_hz = params_.at("ground_low_hz");
    const double ground_high_amp = params_.at("ground_high_amp");
    const double ground_high_hz = params_.at("ground_high_hz");

    const double f1 = u.empty() ? 0.0 : u[0];
    const double f2 = u.size() < 2 ? 0.0 : u[1];
    const double f3 = u.size() < 3 ? 0.0 : u[2];

    const double x1 = s[0];
    const double v1 = s[1];
    const double x2 = s[2];
    const double v2 = s[3];
    const double x3 = s[4];
    const double v3 = s[5];

    const double wg1 = 2.0 * M_PI * ground_low_hz;
    const double wg2 = 2.0 * M_PI * ground_high_hz;
    const double ground = ground_low_amp * std::sin(wg1 * t) + ground_high_amp * std::sin(wg2 * t + 0.6);
    const double ground_v = ground_low_amp * wg1 * std::cos(wg1 * t) + ground_high_amp * wg2 * std::cos(wg2 * t + 0.6);

    const double dx1 = v1;
    const double dv1 = (-k1 * (x1 - ground) - c1 * (v1 - ground_v) - k12 * (x1 - x2) - c12 * (v1 - v2) + f1) / m1;
    const double dx2 = v2;
    const double dv2 = (k12 * (x1 - x2) + c12 * (v1 - v2) - k23 * (x2 - x3) - c23 * (v2 - v3) + f2) / m2;
    const double dx3 = v3;
    const double dv3 = (k23 * (x2 - x3) + c23 * (v2 - v3) + f3) / m3;

    return {dx1, dv1, dx2, dv2, dx3, dv3};
}

StateVector ActiveSeismicIsolationBench::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const {
    return {s[4]};
}

StateVector ActiveSeismicIsolationBench::defaultInitialState() const {
    return {0.0003, 0.0, 0.0002, 0.0, 0.0001, 0.0};
}

std::vector<ParamDescriptor> ActiveSeismicIsolationBench::parameterDescriptors() const {
    return {{"m1","kg","Stage 1 mass",120.0,1.0,1000.0,1.0},
            {"m2","kg","Stage 2 mass",80.0,1.0,1000.0,1.0},
            {"m3","kg","Bench mass",40.0,1.0,1000.0,1.0},
            {"k1","N/m","Ground-stage stiffness",4.5e4,1e3,1e6,100.0},
            {"k12","N/m","Stage 1-2 stiffness",2.8e4,1e3,1e6,100.0},
            {"k23","N/m","Stage 2-3 stiffness",1.8e4,1e3,1e6,100.0},
            {"c1","N·s/m","Ground-stage damping",220.0,0.0,1e4,1.0},
            {"c12","N·s/m","Stage 1-2 damping",150.0,0.0,1e4,1.0},
            {"c23","N·s/m","Stage 2-3 damping",90.0,0.0,1e4,1.0},
            {"ground_low_amp","m","Low-frequency ground amplitude",0.0005,0.0,0.01,1e-5},
            {"ground_low_hz","Hz","Low-frequency ground content",0.8,0.01,20.0,0.01},
            {"ground_high_amp","m","High-frequency ground amplitude",2.0e-5,0.0,0.01,1e-6},
            {"ground_high_hz","Hz","High-frequency ground content",18.0,0.1,200.0,0.1}};
}

std::vector<Preset> ActiveSeismicIsolationBench::presets() const {
    return {{"Optical table","Moderate stage coupling",{{"m1",120.0},{"m2",80.0},{"m3",40.0},{"k1",4.5e4},{"k12",2.8e4},{"k23",1.8e4},{"c1",220.0},{"c12",150.0},{"c23",90.0},{"ground_low_amp",0.0005},{"ground_low_hz",0.8},{"ground_high_amp",2.0e-5},{"ground_high_hz",18.0}}},
            {"Ultra quiet","Softer support with lower seismic floor",{{"m1",150.0},{"m2",100.0},{"m3",50.0},{"k1",2.5e4},{"k12",1.8e4},{"k23",1.2e4},{"c1",180.0},{"c12",110.0},{"c23",70.0},{"ground_low_amp",0.0002},{"ground_low_hz",0.5},{"ground_high_amp",8.0e-6},{"ground_high_hz",12.0}}}};
}

void ActiveSeismicIsolationBench::applyPreset(int index) {
    auto preset_list = presets();
    if (index >= 0 && index < static_cast<int>(preset_list.size())) {
        setParameters(preset_list[index].params);
    }
}

std::vector<std::string> ActiveSeismicIsolationBench::stateNames() const {
    return {"x1","v1","x2","v2","x3","v3"};
}

std::vector<std::string> ActiveSeismicIsolationBench::outputNames() const {
    return {"Bench displacement"};
}

std::vector<std::string> ActiveSeismicIsolationBench::inputNames() const {
    return {"Actuator 1","Actuator 2","Actuator 3"};
}

}  // namespace Simulation
