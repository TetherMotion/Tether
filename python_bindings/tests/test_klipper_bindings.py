"""
Tests for the tether.klipper Python bindings.

These tests verify that the newly-bound classes are importable and that
their core methods work as expected.  They are intentionally lightweight —
the heavy behavioural validation lives in the C++ gtest suite — and focus
on the Python-facing API surface.
"""

import math
import os
import sys
import unittest

# Ensure the built module is importable when running from the build tree.
_BUILD_LIB = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "lib",
)
if _BUILD_LIB not in sys.path:
    sys.path.insert(0, _BUILD_LIB)

import tether.klipper as klipper  # noqa: E402


class TestDeltaPrinter(unittest.TestCase):
    """DeltaPrinter kinematics round-trip."""

    def test_default_geometry(self):
        dp = klipper.DeltaPrinter()
        geo = dp.geometry
        self.assertAlmostEqual(geo.arm_length, 250.0, places=1)
        self.assertAlmostEqual(geo.delta_radius, 125.0, places=1)

    def test_set_geometry(self):
        dp = klipper.DeltaPrinter()
        geo = klipper.DeltaGeometry()
        geo.arm_length = 300.0
        geo.delta_radius = 150.0
        dp.set_geometry(geo)
        g2 = dp.geometry
        self.assertAlmostEqual(g2.arm_length, 300.0, places=1)
        self.assertAlmostEqual(g2.delta_radius, 150.0, places=1)

    def test_kinematics_roundtrip(self):
        dp = klipper.DeltaPrinter()
        x, y, z = 50.0, -30.0, 200.0
        tower = dp.forward_actuator_kinematics(x, y, z)
        self.assertEqual(len(tower), 3)
        back = dp.inverse_actuator_kinematics(tower[0], tower[1], tower[2])
        self.assertAlmostEqual(back[0], x, places=2)
        self.assertAlmostEqual(back[1], y, places=2)
        self.assertAlmostEqual(back[2], z, places=2)


class TestTmcDriverConfig(unittest.TestCase):
    """TMC driver configuration."""

    def test_create_and_set_params(self):
        cfg = klipper.TmcDriverConfig()
        cfg.set_run_current("x", 1.2)
        cfg.set_hold_current("x", 0.8)
        p = cfg.params("x")
        self.assertAlmostEqual(p.run_current, 1.2, places=2)
        self.assertAlmostEqual(p.hold_current, 0.8, places=2)


class TestFilamentLoader(unittest.TestCase):
    """FilamentLoader M701-M708 commands."""

    def test_load_unload_callbacks(self):
        fl = klipper.FilamentLoader()
        loaded = {"value": False}
        unloaded = {"value": False}

        def on_load(speed, length, extra):
            loaded["value"] = True

        def on_unload(speed, length, extra):
            unloaded["value"] = True

        fl.set_load_callback(on_load)
        fl.set_unload_callback(on_unload)
        fl.load_filament(100)  # load 100mm
        fl.unload_filament(100)  # unload 100mm
        self.assertTrue(loaded["value"])
        self.assertTrue(unloaded["value"])


class TestMultiMcuManager(unittest.TestCase):
    """Multi-MCU coordination."""

    def test_default_no_mcus(self):
        mgr = klipper.MultiMcuManager()
        self.assertEqual(len(mgr.mcu_ids()), 0)


class TestSkewCorrection(unittest.TestCase):
    """Skew correction."""

    def test_default_params(self):
        sc = klipper.SkewCorrection()
        p = sc.params
        self.assertAlmostEqual(p.xy, 0.0, places=4)
        self.assertAlmostEqual(p.xz, 0.0, places=4)
        self.assertAlmostEqual(p.yz, 0.0, places=4)

    def test_set_params(self):
        sc = klipper.SkewCorrection()
        p = klipper.SkewParams()
        p.xy = 0.01
        p.xz = 0.02
        p.yz = 0.03
        sc.set_params(p)
        p2 = sc.params
        self.assertAlmostEqual(p2.xy, 0.01, places=4)
        self.assertAlmostEqual(p2.xz, 0.02, places=4)
        self.assertAlmostEqual(p2.yz, 0.03, places=4)


class TestCaseLight(unittest.TestCase):
    """CaseLight peripheral."""

    def test_default_state(self):
        cl = klipper.CaseLight()
        self.assertFalse(cl.is_on)

    def test_set_state(self):
        cl = klipper.CaseLight()
        cl.set_state(True, 0.5)
        self.assertTrue(cl.is_on)


class TestConfigValidator(unittest.TestCase):
    """Config validation."""

    def test_valid_config(self):
        cp = klipper.ConfigParser()
        cp.parse("[printer]\nkinematics: cartesian\nmax_velocity: 500\nmax_accel: 3000\n")
        cv = klipper.ConfigValidator()
        results = cv.validate(cp)
        self.assertIsInstance(results, list)
        # Should have at least one result for the printer section
        self.assertTrue(len(results) > 0)

    def test_result_has_errors_list(self):
        cp = klipper.ConfigParser()
        cp.parse("[stepper_x]\nstep_pin: PA0\n")
        cv = klipper.ConfigValidator()
        results = cv.validate(cp)
        for r in results:
            self.assertIsInstance(r.errors, list)


class TestDigitalOut(unittest.TestCase):
    """DigitalOut peripheral."""

    def test_default_state(self):
        dout = klipper.DigitalOut(0)
        self.assertFalse(dout.value)

    def test_set_value(self):
        dout = klipper.DigitalOut(0)
        dout.set_value(True)
        self.assertTrue(dout.value)


class TestPWMOut(unittest.TestCase):
    """PWMOut peripheral."""

    def test_default_duty(self):
        pwm = klipper.PWMOut(0)
        self.assertEqual(pwm.duty, 0)

    def test_set_duty(self):
        pwm = klipper.PWMOut(0)
        pwm.set_duty(0.75)
        # duty_double is a property returning the float duty cycle
        self.assertAlmostEqual(pwm.duty_double, 0.75, places=2)


class TestAnalogIn(unittest.TestCase):
    """AnalogIn peripheral."""

    def test_read_value(self):
        ain = klipper.AnalogIn(0, lambda: 500)
        ain.update()
        self.assertGreaterEqual(ain.read(), 0)


class TestEndstop(unittest.TestCase):
    """Endstop peripheral."""

    def test_triggered_state(self):
        es = klipper.Endstop(0, lambda: True)
        # triggered() should return a bool
        result = es.triggered()
        self.assertIsInstance(result, bool)


class TestTrsync(unittest.TestCase):
    """Trsync state machine."""

    def test_default_state(self):
        ts = klipper.Trsync(0)
        self.assertEqual(ts.state, klipper.TrsyncState.Idle)


class TestHallFilamentSensor(unittest.TestCase):
    """Hall filament sensor."""

    def test_diameter(self):
        sensor = klipper.HallFilamentSensor(0, lambda: 1.75)
        d = sensor.diameter()
        self.assertIsInstance(d, (int, float))


class TestJsonValue(unittest.TestCase):
    """JsonValue class."""

    def test_json_value_int(self):
        v = klipper.JsonValue(42)
        self.assertTrue(v.is_int())
        self.assertEqual(v.as_int(), 42)

    def test_json_value_double(self):
        v = klipper.JsonValue(3.14)
        self.assertTrue(v.is_double())
        self.assertAlmostEqual(v.as_double(), 3.14, places=2)

    def test_json_value_string(self):
        v = klipper.JsonValue("hello")
        self.assertTrue(v.is_string())
        self.assertEqual(v.as_string(), "hello")

    def test_json_value_bool(self):
        v = klipper.JsonValue(True)
        self.assertTrue(v.is_bool())
        self.assertTrue(v.as_bool())

    def test_json_value_null(self):
        v = klipper.JsonValue()
        self.assertTrue(v.is_null())

    def test_json_value_dump(self):
        v = klipper.JsonValue("test")
        s = v.dump()
        self.assertIn("test", s)


class TestFirmwareRetraction(unittest.TestCase):
    """FirmwareRetraction params."""

    def test_set_and_retract(self):
        params = klipper.FirmwareRetractionParams()
        params.retract_length = 3.0
        params.retract_speed = 35.0
        params.unretract_length = 0.0
        params.unretract_speed = 15.0
        params.z_hop = 0.4
        fr = klipper.FirmwareRetraction(params)
        self.assertFalse(fr.is_retracted())
        fr.retract()
        self.assertTrue(fr.is_retracted())
        fr.unretract()
        self.assertFalse(fr.is_retracted())


class TestThermistor(unittest.TestCase):
    """Thermistor sensor."""

    def test_create(self):
        params = klipper.ThermistorParams()
        params.beta = 3950.0
        params.resistance_at_25c = 100000.0
        params.pullup_resistor = 4700.0
        params.reference_voltage = 3.3
        params.adc_max = 1023
        params.min_temp = 0
        params.max_temp = 300
        t = klipper.Thermistor(0, params, lambda: 500)
        self.assertIsNotNone(t)


class TestParseGcodeLine(unittest.TestCase):
    """G-code line parsing utility."""

    def test_parse_simple(self):
        result = klipper.parse_gcode_line("G1 X10 Y20 F600")
        self.assertIsNotNone(result)
        self.assertEqual(result.code, "G1")

    def test_parse_mcode(self):
        result = klipper.parse_gcode_line("M104 S200")
        self.assertIsNotNone(result)
        self.assertEqual(result.code, "M104")


if __name__ == "__main__":
    unittest.main()
