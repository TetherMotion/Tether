#!/usr/bin/env python3
"""
================================================================================
rotor_gen.py — Pressure-advance / ooze / extrusion-dynamics TESTCASE generator
================================================================================

WHAT THIS IS
------------
This script generates raw, slicer-independent G-code for a TORTURE-TEST PRINT
designed to characterise extrusion dynamics: pressure advance (linear
advance), ooze/stringing, and extruder step behaviour over a wide range of
speeds and accelerations. It is NOT a functional part — it is a diagnostic
instrument that happens to be shaped like a rotor.

Why a rotor-like shape? Because the test must look *somewhat* like a real
object: concentric rings, radial ribs, and notches are features you would
genuinely find in impellers, wheels, gears, and fan rotors. That makes the
results meaningful — artefacts appear in the same places they would on a
real print — while the exact geometry is chosen purely to stress the
extrusion system in specific, measurable ways.

WHY DIRECT G-CODE INSTEAD OF CAD + SLICER
-----------------------------------------
  * Full control of feed rate per segment: the test needs speeds a slicer
    would never choose (e.g. constant-omega scaling, 10% cutout speed,
    a 25%-per-ring speed ladder).
  * No slicer "helpfully" adding retracts, wipes, or pressure advance of
    its own: the G-code must contain ONLY the motion, so that whatever
    ooze or pressure artefact you see is caused by YOUR controller
    settings, not the slicer.
  * Exact control of stop/start points: the hard stops at vane tips and
    the duty-cycle gaps must be where you put them, not where a path
    planner merges them.

THE THREE TEST REGIONS AND WHAT EACH ONE STRESSES
-------------------------------------------------

1. ISLAND RINGS (--rings, --island-length, --omega)
   A set of concentric rings, each broken into disconnected arc "islands"
   with a 50% duty cycle (equal printed/gap length). Unconnected to the
   rotor body.

   WHY: This is the OOZING and RESTART test, repeated dozens of times per
   layer at many radii. Every island is: travel move -> start of extrusion
   -> short arc -> end of extrusion -> travel. The gaps mean there is a
   hard start and hard stop every few millimetres. If your pressure
   advance is wrong, you see it as:
     * blobs/doglegs at island STARTS  (too little PA / ooze on travel)
     * under-extruded tails at island ENDS (too much PA / insufficient
       pressure at deceleration)
     * strings across the GAPS         (ooze during travel moves)

   WHY CONSTANT OMEGA (--omega): linear feed = omega x radius, so the
   SAME angular speed gives a different LINEAR speed on every ring. One
   layer therefore sweeps extrusion speed continuously from very slow
   (innermost ring) to fast (outermost ring) — a built-in speed sweep.
   PA behaviour is speed-dependent (melt pressure, viscoelasticity), and
   this lets you find the speed at which your settings break down, ring
   by ring, in a single print. The innermost ring is printed VERY slowly,
   probing the low-speed regime where ooze dominates over pressure.

   WHY FIXED ISLAND LENGTH (--island-length), not fixed count: island
   arc length stays constant with radius while the gap/count adapts, so
   every island presents the same extrusion-time signature and you can
   compare start/stop artefacts across rings directly, with only the
   speed changing. The innermost ring degenerates to 2 printed + 2 empty
   sections (the shortest ring that still has 50% duty cycle).

   WHY INNERMOST FIRST: printing order inner->outer means the speed ramps
   monotonically UP within a layer; you never slow back down, so there is
   no ambiguity about which artefact belongs to which speed.

2. VANES (--vanes, --vane-lines, --vane-inner-radius)
   Eight thin radial ribs, each printed as 4 parallel lines, OUTSIDE-IN,
   each ending in a HARD STOP (extruder and X/Y halt simultaneously, with
   a dwell) at the inner tip.

   WHY OUTSIDE-IN with a HARD STOP: this is the DECELERATION-TO-ZERO and
   PRESSURE-DECAY test. The line runs inward at speed and then everything
   — X, Y, and E — stops dead at the tip. Melt pressure doesn't vanish
   instantly, so with insufficient PA you get a fat blob at every vane
   tip; with too much PA you get a starved, hollow tip. Eight vanes x 4
   lines = 32 identical hard stops per layer, all at the same radius, so
   the tips form a ring of blobs whose size directly visualises residual
   nozzle pressure. Comparing tip-to-tip consistency also reveals
   hysteresis and step-skipping.

   WHY 4 PARALLEL LINES: the adjacent lines share walls, so PA errors also
   show up as ridge/valley banding BETWEEN lines — a second observable
   from the same feature. Four lines is enough to show the effect without
   making the vane a solid block.

3. OUTER ROTOR + SEMICIRCULAR CUTOUTS
   (--outer-layers, --rotor-outer-radius, --outer-base-feed,
    --outer-speed-step, --cutout-depth, --cutout-protrusion,
    --cutout-speed-factor)
   The outer body is N concentric circle layers — long, continuous
   extrusion that establishes the STEADY-STATE reference pressure (what
   "good" extrusion looks like at speed, against which you compare the
   islands and vane tips).

   WHY EACH RING 25% FASTER THAN THE LAST (--outer-speed-step): the
   concentric circles form a discrete SPEED LADDER. Ring 0 runs at
   --outer-base-feed, ring 1 at 1.25x, ring 2 at 1.25^2 x, and so on —
   with the default 10 rings the outermost circle runs ~9.3x faster than
   the innermost. Because the rings are adjacent and identical in shape,
   any speed-dependent artefact (under-extrusion at high flow, ringing,
   PA lag) shows up as a visible step change from one ring to the next,
   and you can read off exactly at which feed rate your extrusion system
   starts to degrade. The geometric series covers a wide speed range in
   few rings while keeping equal relative steps — each step is the same
   *proportional* challenge to the pressure controller.

   WHY THE CUTOUTS AT 10% SPEED (--cutout-speed-factor): they are the
   SUDDEN SPEED-CHANGE test. The nozzle goes from full circle speed to
   10% speed in one move. The pressure built up at high speed must be
   bled off by your PA before the slow arc, or the cutout comes out fat
   and overflowing. Immediately after, it must re-prime for the return
   to full speed. The 50% protrusion (--cutout-protrusion, half the
   semicircle outside the nominal circle) means the cutout edge is partly
   printed in free air — an overhang-like condition that amplifies any
   over-extrusion into visible curling and droop, making small pressure
   errors easy to see.

HOW TO USE IT
-------------
  1. Print with pressure advance DISABLED -> keep this as your reference
     for "what the geometry wants to do wrong": blobs at vane tips and
     island starts, starved island ends, fat cutouts, strings in gaps.
  2. Re-print sweeping your PA value (e.g. M572 D... / M900 K...) and
     compare, region by region:
       island starts/ends  -> tune PA for accel/decel transients
       vane tips           -> tune PA for full stops
       cutouts             -> tune PA for speed changes
       outer ring ladder   -> find the feed rate where extrusion degrades
       gap strings         -> tune retraction, not PA
  3. Because the island speed sweep is radial, note WHICH RING artefacts
     first appear/disappear on — that tells you the speed range over
     which your settings are valid.

PARAMETERS
----------
Everything is argparse-configurable; defaults: 8 island rings, 8 vanes,
4 lines/vane, 10 outer circle layers with a 1.25x speed step, 50% cutout
protrusion, 10% cutout speed factor, 0.4 mm nozzle, 0.2 mm layers.
Run --help for the full list.

CAVEATS
-------
  * This emits only motion: add your own start/end G-code, temps,
    retraction and PA settings so they are IDENTICAL between comparison
    prints except for the parameter under test.
  * No brim/skirt/priming is generated.
  * Preview before printing (e.g. PrusaSlicer G-code viewer).
================================================================================
"""

import argparse
import math


def parse_args():
    p = argparse.ArgumentParser(
        description="Pressure-advance / ooze testcase: island-ring rotor.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    # Machine / filament
    p.add_argument("--nozzle", type=float, default=0.4,
                   help="Nozzle diameter / line width (mm)")
    p.add_argument("--layer-height", type=float, default=0.2,
                   help="Layer height (mm)")
    p.add_argument("--filament-dia", type=float, default=1.75,
                   help="Filament diameter (mm)")
    p.add_argument("--layers", type=int, default=1,
                   help="Number of Z layers to print")
    p.add_argument("--feed", type=float, default=2400.0,
                   help="Nominal extrusion feed rate (mm/min)")
    p.add_argument("--travel-feed", type=float, default=6000.0,
                   help="Travel feed rate (mm/min)")
    p.add_argument("--max-feed", type=float, default=12000.0,
                   help="Clamp for all computed extrusion feeds (mm/min)")
    p.add_argument("--temp", type=int, default=210, help="Hotend temp (C)")
    p.add_argument("--bed-temp", type=int, default=60, help="Bed temp (C)")

    # Island rings
    p.add_argument("--rings", type=int, default=8,
                   help="Number of island rings")
    p.add_argument("--inner-radius", type=float, default=10.0,
                   help="Radius of innermost island ring (mm)")
    p.add_argument("--ring-spacing", type=float, default=3.0,
                   help="Radial spacing between island rings (mm)")
    p.add_argument("--island-length", type=float, default=10.0,
                   help="Approximate arc length of each island (mm)")
    p.add_argument("--omega", type=float, default=1.0,
                   help="Constant angular speed for island rings (rad/s)")

    # Vanes
    p.add_argument("--vanes", type=int, default=8, help="Number of vanes")
    p.add_argument("--vane-lines", type=int, default=4,
                   help="Parallel lines per vane")
    p.add_argument("--vane-inner-radius", type=float, default=12.0,
                   help="Radius where vane tips (hard stops) end (mm)")

    # Outer rotor
    p.add_argument("--rotor-outer-radius", type=float, default=45.0,
                   help="Outer radius of the rotor (mm)")
    p.add_argument("--outer-layers", type=int, default=10,
                   help="Number of concentric outer circle layers")
    p.add_argument("--outer-base-feed", type=float, default=600.0,
                   help="Feed rate of the innermost outer circle (mm/min)")
    p.add_argument("--outer-speed-step", type=float, default=1.25,
                   help="Speed multiplier per outer ring (1.25 = +25%%)")
    p.add_argument("--cutout-depth", type=float, default=4.0,
                   help="Full depth (diameter) of semicircular cutouts (mm)")
    p.add_argument("--cutout-protrusion", type=float, default=0.5,
                   help="Fraction of cutout depth protruding past outer "
                   "radius (0.5 = 50%%)")
    p.add_argument("--cutout-speed-factor", type=float, default=0.1,
                   help="Feed multiplier while printing cutouts (0.1 = 10%%)")

    p.add_argument("-o", "--output", default="rotor_test.gcode",
                   help="Output G-code file")
    return p.parse_args()


class GCode:
    """Minimal G-code emitter with absolute-XY / absolute-E tracking."""

    def __init__(self, a):
        self.a = a
        self.lines = []
        self.e = 0.0
        self.x = None
        self.y = None
        # mm of filament per mm of extruded path
        area_fil = math.pi * (a.filament_dia / 2.0) ** 2
        self.e_per_mm = (a.nozzle * a.layer_height) / area_fil

    def emit(self, s):
        self.lines.append(s)

    def travel(self, x, y):
        self.emit(f"G0 X{x:.3f} Y{y:.3f} F{self.a.travel_feed:.0f}")
        self.x, self.y = x, y

    def extrude(self, x, y, feed):
        feed = min(feed, self.a.max_feed)
        if self.x is None:
            raise RuntimeError("extrude before travel")
        dist = math.hypot(x - self.x, y - self.y)
        self.e += dist * self.e_per_mm
        self.emit(f"G1 X{x:.3f} Y{y:.3f} E{self.e:.5f} F{feed:.0f}")
        self.x, self.y = x, y

    def arc(self, cx, cy, r, a0, a1, feed, step_deg=2.0):
        """Extrude along a circular arc from angle a0 to a1 (degrees)."""
        span = a1 - a0
        n = max(1, int(abs(span) / step_deg))
        for i in range(1, n + 1):
            ang = math.radians(a0 + span * i / n)
            self.extrude(cx + r * math.cos(ang), cy + r * math.sin(ang), feed)

    def hard_stop(self, dwell_ms=150):
        """Deliberate full stop: dwell, so X/Y AND E all stop dead."""
        self.emit(f"G4 P{dwell_ms} ; HARD STOP (pressure-decay probe)")


# ----------------------------------------------------------------------------
# Region 1: island rings
# ----------------------------------------------------------------------------
def island_rings(g, a):
    g.emit("\n; === REGION 1: ISLAND RINGS (ooze/restart + radial speed sweep)")
    for ring in range(a.rings):
        r = a.inner_radius + ring * a.ring_spacing
        circ = 2.0 * math.pi * r
        # 50% duty cycle: period = 2 * island_length. Round to an integer
        # number of islands, minimum 2 (innermost: 2 printed + 2 empty).
        n_islands = max(2, int(round(circ / (2.0 * a.island_length))))
        period_deg = 360.0 / n_islands
        island_deg = period_deg * 0.5
        # constant omega -> linear feed scales with radius (speed sweep!)
        feed = a.omega * r * 60.0  # rad/s * mm = mm/s -> mm/min
        g.emit(f"; ring {ring}: r={r:.2f}mm islands={n_islands} "
               f"feed={min(feed, a.max_feed):.0f}mm/min")
        for k in range(n_islands):
            a0 = k * period_deg
            x0 = r * math.cos(math.radians(a0))
            y0 = r * math.sin(math.radians(a0))
            g.travel(x0, y0)
            g.arc(0, 0, r, a0, a0 + island_deg, feed)


# ----------------------------------------------------------------------------
# Region 2: vanes (outside-in, hard stop at tip)
# ----------------------------------------------------------------------------
def vanes(g, a):
    g.emit("\n; === REGION 2: VANES (hard-stop pressure-decay probe) ===")
    r_out = a.rotor_outer_radius - a.nozzle  # stick to inside of rotor
    for v in range(a.vanes):
        ang = math.radians(v * 360.0 / a.vanes)
        # perpendicular direction for the parallel line offsets
        px, py = -math.sin(ang), math.cos(ang)
        dx, dy = math.cos(ang), math.sin(ang)
        for line in range(a.vane_lines):
            off = (line - (a.vane_lines - 1) / 2.0) * a.nozzle
            x_out = r_out * dx + off * px
            y_out = r_out * dy + off * py
            x_in = a.vane_inner_radius * dx + off * px
            y_in = a.vane_inner_radius * dy + off * py
            g.travel(x_out, y_out)            # start OUTSIDE
            g.extrude(x_in, y_in, a.feed)     # extrude INWARD
            g.hard_stop()                     # dead stop at the tip


# ----------------------------------------------------------------------------
# Region 3: outer rotor circle layers + semicircular cutouts
# ----------------------------------------------------------------------------
def outer_rotor(g, a):
    g.emit("\n; === REGION 3: OUTER ROTOR (speed ladder + cutouts) ===")
    rc = a.cutout_depth / 2.0  # semicircle radius
    # centre of each cutout semicircle: 'protrusion' fraction of the depth
    # lies OUTSIDE the nominal outer radius.
    r_centre = a.rotor_outer_radius + a.cutout_protrusion * a.cutout_depth - rc
    # half-angle (on the outer circle) subtended by each cutout, from the
    # circle-circle intersection (law of cosines)
    cos_d = (a.rotor_outer_radius**2 + r_centre**2 - rc**2) / \
            (2.0 * a.rotor_outer_radius * r_centre)
    cos_d = max(-1.0, min(1.0, cos_d))
    half_ang = math.degrees(math.acos(cos_d))
    cutout_centres = [(k + 0.5) * 360.0 / a.vanes for k in range(a.vanes)]

    for i in range(a.outer_layers):
        r = a.rotor_outer_radius - i * a.nozzle
        feed = a.outer_base_feed * (a.outer_speed_step ** i)
        g.emit(f"; outer ring {i}: r={r:.2f}mm "
               f"feed={min(feed, a.max_feed):.0f}mm/min "
               f"({a.outer_speed_step**i:.2f}x base)")
        g.travel(r, 0)
        # Walk the full circle; when entering a cutout's angular window,
        # detour along the semicircle at the slow cutout feed.
        slow = feed * a.cutout_speed_factor
        deg = 0.0
        step = 2.0
        while deg < 360.0:
            nxt = deg + step
            hit = None
            for c in cutout_centres:
                d = (c - deg) % 360.0
                if d <= step + 1e-9 and d > 1e-9:
                # entering a cutout window at c - half_ang .. c + half_ang
                    pass
            # simpler: check if next sample crosses (c - half_ang)
            for c in cutout_centres:
                start = (c - half_ang) % 360.0
                if deg <= start < nxt or (start < deg and nxt >= 360.0):
                    hit = c
                    break
            if hit is None:
                g.arc(0, 0, r, deg, nxt, feed, step_deg=step)
            else:
                # finish circle up to cutout entry
                entry = (hit - half_ang) % 360.0
                g.arc(0, 0, r, deg, entry, feed, step_deg=step)
                # semicircle: centre on the cutout angle at r_centre
                ca = math.radians(hit)
                cx, cy = r_centre * math.cos(ca), r_centre * math.sin(ca)
                # entry/exit points relative to cutout centre
                ex = r * math.cos(math.radians(entry)) - cx
                ey = r * math.sin(math.radians(entry)) - cy
                a_entry = math.degrees(math.atan2(ey, ex))
                # sweep the long way around (through the innermost point,
                # which lies opposite the cutout centre direction)
                g.arc(cx, cy, rc, a_entry, a_entry + 180.0, slow,
                      step_deg=step)
                deg = (hit + half_ang) % 360.0
                continue
            deg = nxt


# ----------------------------------------------------------------------------
# main
# ----------------------------------------------------------------------------
def main():
    a = parse_args()
    g = GCode(a)

    g.emit("; rotor_gen.py — pressure-advance / ooze TESTCASE")
    g.emit("; NOT a functional part. See header docs for what to look for.")
    g.emit("G21 ; mm")
    g.emit("G90 ; absolute XY")
    g.emit("M83 ; relative E" if False else "M82 ; absolute E")
    g.emit("G92 E0")
    g.emit(f"M104 S{a.temp}")
    g.emit(f"M140 S{a.bed_temp}")
    g.emit(f"M109 S{a.temp}")
    g.emit(f"M190 S{a.bed_temp}")
    g.emit("G28 ; home")
    g.emit("; NOTE: no priming/retraction/PA here by design — set those in")
    g.emit("; your printer start code, IDENTICALLY across comparison prints.")

    for layer in range(a.layers):
        z = a.layer_height * (layer + 1)
        g.emit(f"\n; ===== LAYER {layer + 1}/{a.layers}  Z={z:.2f} =====")
        g.emit(f"G0 Z{z:.3f} F{a.travel_feed:.0f}")
        island_rings(g, a)   # islands first, innermost (slowest) first
        vanes(g, a)
        outer_rotor(g, a)

    g.emit("\n; === END ===")
    g.emit("M104 S0")
    g.emit("M140 S0")
    g.emit("G91")
    g.emit("G0 Z10 F600")
    g.emit("G90")
    g.emit("M84")

    with open(a.output, "w") as f:
        f.write("\n".join(g.lines) + "\n")
    print(f"Wrote {a.output} ({len(g.lines)} lines)")


if __name__ == "__main__":
    main()
