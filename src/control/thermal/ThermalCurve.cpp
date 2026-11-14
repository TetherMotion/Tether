#include "tether/control/thermal/ThermalCurve.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace tether::control::thermal {

namespace {

// Time [s] inside one linear segment where T crosses `level`, assuming
// the segment spans it monotonically. Returns {tEnter, tExit} clipped
// to [t0,t1]; both equal when no crossing.
std::pair<double, double> bandTime(const TemperatureProfile::Segment& s,
                                   double lo, double hi, double t0, double t1) {
    // Parameterize T(t) = T0 + m*(t-t0); find t range where lo <= T <= hi.
    const double m = (s.t1 > s.t0) ? (s.T1 - s.T0) / (s.t1 - s.t0) : 0.0;
    double ta = s.t0, tb = s.t1;
    if (std::abs(m) < 1e-12) {
        const bool inside = (s.T0 >= lo && s.T0 <= hi);
        return inside ? std::make_pair(ta, tb) : std::make_pair(t0, t0);
    }
    double tLo = (lo - s.T0) / m + s.t0;
    double tHi = (hi - s.T0) / m + s.t0;
    if (tLo > tHi) std::swap(tLo, tHi);
    ta = std::max(ta, tLo);
    tb = std::min(tb, tHi);
    if (tb < ta) return {t0, t0};
    return {std::max(ta, t0), std::min(tb, t1)};
}

// Fraction of [t0,t1] where linear T(t) > level (for TAL).
double aboveTime(const TemperatureProfile::Segment& s, double level,
                 double t0, double t1) {
    const double m = (s.t1 > s.t0) ? (s.T1 - s.T0) / (s.t1 - s.t0) : 0.0;
    if (std::abs(m) < 1e-12)
        return (s.T0 > level) ? (t1 - t0) : 0.0;
    const double tc = (level - s.T0) / m + s.t0;   // crossing time
    if (m > 0) {  // rising: above level for t > tc
        const double a = std::max(tc, t0), b = t1;
        return b > a ? b - a : 0.0;
    }
    // falling: above level for t < tc
    const double a = t0, b = std::min(tc, t1);
    return b > a ? b - a : 0.0;
}

void addViolation(ThermalCurveReport& r, const char* fmt, double v) {
    char buf[160];
    std::snprintf(buf, sizeof buf, fmt, v);
    r.violations.emplace_back(buf);
}

void checkCommon(const ThermalCurveReport& m,
                 const ThermalCurveSpec& s, ThermalCurveReport& out) {
    if (s.peakMinC > 0.0 && m.peakC < s.peakMinC)
        addViolation(out, "peak %.1fC below min %.1fC", m.peakC);
    if (s.peakMaxC > 0.0 && m.peakC > s.peakMaxC)
        addViolation(out, "peak %.1fC above max %.1fC", m.peakC);
    if (s.talMinS > 0.0 && m.talS < s.talMinS)
        addViolation(out, "TAL %.1fs below min %.1fs", m.talS);
    if (s.talMaxS > 0.0 && m.talS > s.talMaxS)
        addViolation(out, "TAL %.1fs above max %.1fs", m.talS);
    if (s.soakMinS > 0.0 && s.soakMaxC > s.soakMinC &&
        m.soakS < s.soakMinS)
        addViolation(out, "soak %.1fs below min %.1fs", m.soakS);
    if (s.maxHeatRampCPerS > 0.0 &&
        m.heatRampMax > s.maxHeatRampCPerS)
        addViolation(out, "heat ramp %.2fC/s exceeds %.2f",
                     m.heatRampMax);
    if (s.maxCoolRampCPerS > 0.0 &&
        m.coolRampMax > s.maxCoolRampCPerS)
        addViolation(out, "cool ramp %.2fC/s exceeds %.2f",
                     m.coolRampMax);
}

} // namespace

const char* thermalCurvePresetName(ThermalCurvePreset p) {
    switch (p) {
    case ThermalCurvePreset::ReflowSac305:    return "Reflow SAC305";
    case ThermalCurvePreset::ReflowSnPb:      return "Reflow SnPb";
    case ThermalCurvePreset::WaveSolder:      return "Wave solder";
    case ThermalCurvePreset::SelectiveSolder: return "Selective solder";
    case ThermalCurvePreset::Brazing:         return "Silver braze";
    case ThermalCurvePreset::Anneal:          return "Anneal";
    }
    return "?";
}

TemperatureProfile presetProfile(ThermalCurvePreset p) {
    switch (p) {
    case ThermalCurvePreset::ReflowSac305: {
        // Lead-free SAC305: preheat 150C soak, 170C soak, ramp through
        // 217C liquidus to ~245C peak, TAL ~60-120s, forced cool.
        TemperatureProfile prof(25.0);
        prof.addRampSoak(150.0, 1.2, 90.0);    // preheat + soak @150
        prof.addRampSoak(175.0, 0.5, 60.0);    // soak band 150-180
        prof.addRampTo(245.0, 1.0);            // reflow ramp
        prof.addSoak(40.0);                    // peak dwell
        prof.addRampTo(60.0, -3.0);            // cooling
        return prof;
    }
    case ThermalCurvePreset::ReflowSnPb: {
        // Sn63Pb37: lower liquidus 183C, peak ~215C, shorter TAL.
        TemperatureProfile prof(25.0);
        prof.addRampSoak(130.0, 1.5, 60.0);
        prof.addRampSoak(160.0, 0.6, 45.0);
        prof.addRampTo(215.0, 1.2);
        prof.addSoak(25.0);
        prof.addRampTo(60.0, -3.5);
        return prof;
    }
    case ThermalCurvePreset::WaveSolder: {
        // Wave: preheat soak ~105C, fast rise through liquidus, short
        // dwell near pot temp, cool.
        TemperatureProfile prof(25.0);
        prof.addRampSoak(105.0, 2.0, 90.0);
        prof.addRampTo(250.0, 3.0);
        prof.addSoak(8.0);                     // wave contact dwell
        prof.addRampTo(80.0, -4.0);
        return prof;
    }
    case ThermalCurvePreset::SelectiveSolder: {
        // Selective: longer gentle preheat, dip dwell longer than wave.
        TemperatureProfile prof(25.0);
        prof.addRampSoak(110.0, 1.0, 150.0);
        prof.addRampTo(255.0, 2.5);
        prof.addSoak(15.0);
        prof.addRampTo(80.0, -3.5);
        return prof;
    }
    case ThermalCurvePreset::Brazing: {
        // Silver braze ~620C: slow ramp, soak, controlled cool.
        TemperatureProfile prof(25.0);
        prof.addRampSoak(400.0, 2.0, 120.0);
        prof.addRampTo(620.0, 1.5);
        prof.addSoak(60.0);
        prof.addRampTo(200.0, -1.5);
        prof.addSoak(300.0);
        return prof;
    }
    case ThermalCurvePreset::Anneal: {
        // Stress-relief: moderate ramp, long soak, very slow cool.
        TemperatureProfile prof(25.0);
        prof.addRampTo(370.0, 1.5);
        prof.addSoak(1800.0);
        prof.addRampTo(80.0, -0.5);
        return prof;
    }
    }
    return TemperatureProfile{};
}

ThermalCurveSpec specForPreset(ThermalCurvePreset p) {
    ThermalCurveSpec s;
    switch (p) {
    case ThermalCurvePreset::ReflowSac305:
        s.liquidusC = 217.0;
        s.soakMinC = 150.0; s.soakMaxC = 180.0; s.soakMinS = 60.0;
        s.peakMinC = 230.0; s.peakMaxC = 260.0;
        s.talMinS = 60.0;  s.talMaxS = 150.0;
        s.maxHeatRampCPerS = 3.0; s.maxCoolRampCPerS = 6.0;
        break;
    case ThermalCurvePreset::ReflowSnPb:
        s.liquidusC = 183.0;
        s.soakMinC = 140.0; s.soakMaxC = 170.0; s.soakMinS = 45.0;
        s.peakMinC = 200.0; s.peakMaxC = 230.0;
        s.talMinS = 30.0;  s.talMaxS = 90.0;
        s.maxHeatRampCPerS = 3.0; s.maxCoolRampCPerS = 6.0;
        break;
    case ThermalCurvePreset::WaveSolder:
        s.liquidusC = 183.0;
        s.soakMinC = 90.0;  s.soakMaxC = 120.0; s.soakMinS = 30.0;
        s.peakMinC = 230.0; s.peakMaxC = 270.0;
        // TAL here is ramp-dominated (the wave dwell itself is ~8 s);
        // the window bounds total exposure above liquidus, not the
        // reflow-style TAL metric.
        s.talMinS = 20.0;  s.talMaxS = 90.0;
        s.maxHeatRampCPerS = 4.0; s.maxCoolRampCPerS = 8.0;
        break;
    case ThermalCurvePreset::SelectiveSolder:
        s.liquidusC = 183.0;
        s.soakMinC = 95.0;  s.soakMaxC = 130.0; s.soakMinS = 60.0;
        s.peakMinC = 235.0; s.peakMaxC = 270.0;
        s.talMinS = 30.0;  s.talMaxS = 120.0;
        s.maxHeatRampCPerS = 4.0; s.maxCoolRampCPerS = 8.0;
        break;
    case ThermalCurvePreset::Brazing:
        s.liquidusC = 0.0;
        s.peakMinC = 590.0; s.peakMaxC = 650.0;
        s.maxHeatRampCPerS = 5.0; s.maxCoolRampCPerS = 4.0;
        break;
    case ThermalCurvePreset::Anneal:
        s.liquidusC = 0.0;
        s.peakMinC = 350.0; s.peakMaxC = 390.0;
        s.maxHeatRampCPerS = 3.0; s.maxCoolRampCPerS = 2.0;
        break;
    }
    return s;
}

ThermalCurveReport analyzeProfile(const TemperatureProfile& profile,
                                  const ThermalCurveSpec& spec) {
    ThermalCurveReport r;
    if (profile.empty()) {
        r.violations.emplace_back("empty profile");
        return r;
    }
    const auto segs = profile.segments();
    r.durationS = profile.duration();

    for (const auto& s : segs) {
        r.peakC = std::max(r.peakC, std::max(s.T0, s.T1));
        const double slope = (s.t1 > s.t0)
            ? (s.T1 - s.T0) / (s.t1 - s.t0) : 0.0;
        if (slope > 0) r.heatRampMax = std::max(r.heatRampMax, slope);
        if (slope < 0) r.coolRampMax = std::max(r.coolRampMax, -slope);
        if (spec.liquidusC > 0.0)
            r.talS += aboveTime(s, spec.liquidusC, s.t0, s.t1);
        if (spec.soakMaxC > spec.soakMinC && spec.soakMinC >= 0.0) {
            auto [a, b] = bandTime(s, spec.soakMinC, spec.soakMaxC,
                                   s.t0, s.t1);
            r.soakS += b - a;
        }
    }
    checkCommon(r, spec, r);
    return r;
}

ThermalCurveReport analyzeSamples(const std::vector<double>& times,
                                  const std::vector<double>& temps,
                                  const ThermalCurveSpec& spec) {
    ThermalCurveReport r;
    if (times.size() < 2 || times.size() != temps.size()) {
        r.violations.emplace_back("need >=2 equal-size samples");
        return r;
    }
    for (size_t i = 1; i < times.size(); ++i) {
        if (times[i] <= times[i - 1]) {
            r.violations.emplace_back("times not increasing");
            return r;
        }
    }
    r.durationS = times.back() - times.front();

    // Trapezoidal metrics over consecutive samples.
    for (size_t i = 0; i < temps.size(); ++i)
        r.peakC = std::max(r.peakC, temps[i]);
    for (size_t i = 1; i < times.size(); ++i) {
        const double dt = times[i] - times[i - 1];
        const double dT = temps[i] - temps[i - 1];
        const double slope = dT / dt;
        if (slope > 0) r.heatRampMax = std::max(r.heatRampMax, slope);
        if (slope < 0) r.coolRampMax = std::max(r.coolRampMax, -slope);
        if (dt <= 0.0) continue;
        // TAL: fraction of interval above liquidus (linear interp).
        if (spec.liquidusC > 0.0) {
            const double t0 = temps[i - 1], t1 = temps[i];
            if (t0 > spec.liquidusC && t1 > spec.liquidusC)
                r.talS += dt;
            else if (t0 > spec.liquidusC || t1 > spec.liquidusC) {
                const double f = std::abs(t1 - t0) > 1e-12
                    ? std::abs((spec.liquidusC - t0) / (t1 - t0)) : 0.0;
                r.talS += dt * std::clamp(f, 0.0, 1.0);
            }
        }
        // Soak band: linear segment clipped to [soakMinC, soakMaxC].
        if (spec.soakMaxC > spec.soakMinC && spec.soakMinC >= 0.0) {
            TemperatureProfile::Segment s{times[i - 1], times[i],
                                          temps[i - 1], temps[i]};
            auto [a, b] = bandTime(s, spec.soakMinC, spec.soakMaxC,
                                   times[i - 1], times[i]);
            r.soakS += b - a;
        }
    }
    checkCommon(r, spec, r);
    return r;
}

} // namespace tether::control::thermal
