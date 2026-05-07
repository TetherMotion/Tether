G21 ; mm mode
G90 ; absolute
G17 ; XY plane

G64 P5.0

; === Smooth line-arc-line: tangent-continuous transitions ===
; The arc is tangent to both lines — no G64 blending should occur.
G0 Z5
G1 X50 Y0 F500
G2 X50 Y-30 I0 J-15 F500
G1 X0 Y-30 F500
G0 Z5

M30
