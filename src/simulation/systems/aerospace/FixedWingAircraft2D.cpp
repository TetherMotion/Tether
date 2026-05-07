#include "tether/simulation/systems/aerospace/FixedWingAircraft2D.hpp"
#include <cmath>
#include <stdexcept>

namespace Simulation {

FixedWingAircraft2D::FixedWingAircraft2D() {
    initParam("m_fw", 10.0); initParam("S", 0.5); initParam("c_bar", 0.2);
    initParam("Iyy", 1.0); initParam("rho_air", 1.225); initParam("g", 9.81);
    initParam("CL0", 0.3); initParam("CLa", 5.0); initParam("CLde", 0.5);
    initParam("CD0", 0.03); initParam("CDa", 0.1);
    initParam("Cm0", 0.0); initParam("Cma", -0.5); initParam("Cmde", -1.0); initParam("Cmq", -10.0);
}

StateVector FixedWingAircraft2D::dynamics(double /*t*/, const StateVector& s, const StateVector& u) const {
    double m = params_.at("m_fw"), S = params_.at("S"), c = params_.at("c_bar");
    double Iy = params_.at("Iyy"), rho = params_.at("rho_air"), g = params_.at("g");
    double CL0 = params_.at("CL0"), CLa = params_.at("CLa"), CLde = params_.at("CLde");
    double CD0 = params_.at("CD0"), CDa = params_.at("CDa");
    double Cm0 = params_.at("Cm0"), Cma = params_.at("Cma"), Cmde = params_.at("Cmde"), Cmq = params_.at("Cmq");

    double de = u.size()>0 ? u[0] : 0.0; // elevator [rad]
    double T = u.size()>1 ? u[1] : m*g*0.1; // thrust [N]

    double V_a = std::max(s[0], 1.0); // airspeed
    double gamma = s[1]; // flight path angle
    double alpha = s[2]; // angle of attack
    double q_rate = s[3]; // pitch rate
    double theta = gamma + alpha; // pitch angle
    double h = s[5]; // altitude
    (void)h;

    double qbar = 0.5 * rho * V_a * V_a;
    double CL = CL0 + CLa * alpha + CLde * de;
    double CD = CD0 + CDa * alpha * alpha;
    double Cm = Cm0 + Cma * alpha + Cmde * de + Cmq * c * q_rate / (2.0*V_a);

    double L = qbar * S * CL;
    double D = qbar * S * CD;
    double M_pitch = qbar * S * c * Cm;

    double dV = (T*std::cos(alpha) - D)/m - g*std::sin(gamma);
    double dgamma = (L + T*std::sin(alpha))/(m*V_a) - g*std::cos(gamma)/V_a;
    double dalpha = q_rate - dgamma;
    double dq = M_pitch / Iy;
    (void)theta;
    double dtheta = q_rate;
    double dh = V_a * std::sin(gamma);

    return {dV, dgamma, dalpha, dq, dtheta, dh};
}

StateVector FixedWingAircraft2D::output(double /*t*/, const StateVector& s, const StateVector& /*u*/) const { return {s[5], s[0]}; }
StateVector FixedWingAircraft2D::defaultInitialState() const { return {20.0, 0.0, 0.05, 0.0, 0.05, 100.0}; }

std::vector<ParamDescriptor> FixedWingAircraft2D::parameterDescriptors() const {
    return {{"m_fw","kg","Mass",10.0,0.5,100.0,0.1},{"S","m²","Wing area",0.5,0.01,10.0,0.01},
            {"c_bar","m","Mean chord",0.2,0.01,2.0,0.01},{"Iyy","kg·m²","Pitch inertia",1.0,0.01,100.0,0.01},
            {"rho_air","kg/m³","Air density",1.225,0.5,1.5,0.001},{"g","m/s²","Gravity",9.81,0.0,20.0,0.01},
            {"CL0","","CL0",0.3,0.0,1.0,0.01},{"CLa","1/rad","CLα",5.0,1.0,10.0,0.1},
            {"CD0","","CD0",0.03,0.001,0.2,0.001},{"Cma","1/rad","Cmα",-0.5,-5.0,0.0,0.01}};
}

std::vector<Preset> FixedWingAircraft2D::presets() const {
    return {{"UAV","Small fixed-wing",{{"m_fw",10.0},{"S",0.5},{"c_bar",0.2},{"Iyy",1.0},{"rho_air",1.225},{"g",9.81}, // LCOV_EXCL_LINE // LCOV_EXCL_LINE
              {"CL0",0.3},{"CLa",5.0},{"CLde",0.5},{"CD0",0.03},{"CDa",0.1},{"Cm0",0.0},{"Cma",-0.5},{"Cmde",-1.0},{"Cmq",-10.0}}}};
}
void FixedWingAircraft2D::applyPreset(int i) { auto p=presets(); if(i>=0&&i<(int)p.size()) setParameters(p[i].params); }
std::vector<std::string> FixedWingAircraft2D::stateNames() const { return {"V","γ","α","q","θ","h"}; }
std::vector<std::string> FixedWingAircraft2D::outputNames() const { return {"Altitude","Airspeed"}; }
std::vector<std::string> FixedWingAircraft2D::inputNames() const { return {"Elevator","Thrust"}; }

}  // namespace Simulation
