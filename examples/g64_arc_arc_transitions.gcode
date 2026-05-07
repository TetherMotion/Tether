G21 ; mm mode
G90 ; absolute
G17 ; XY plane

G64 P3.0

; === Arc-to-arc transitions ===

; --- Smooth: two CCW quarter-circles sharing the same center ---
; Together they form a smooth semicircle.
G0 X20 Y0 Z5
G3 X0 Y20 I-20 J0 F500
G3 X-20 Y0 I0 J-20 F500
G1 X-40 Y0 F500

; --- Non-smooth: two arcs meeting at a point with ~90° tangent change ---
G0 X0 Y-30 Z5
G2 X10 Y-40 I0 J-10 F500
G3 X10 Y-50 I5 J0 F500
G1 X-10 Y-50 F500

G0 Z5
M30
