G21 ; mm mode
G90 ; absolute
G17 ; XY plane

G64 P3.0

; === Line-to-arc transitions at various angles ===
; Each pair demonstrates a different angular mismatch between line and arc.

; --- 0° (smooth): line along +X, arc tangent to it ---
G0 X0 Y0 Z5
G1 X30 Y0 F500
G2 X30 Y-20 I0 J-10 F500
G1 X0 Y-20 F500

; --- ~45° mismatch ---
; Line along +X ending at (30, -40)
; Arc with center offset to create 45° tangent mismatch
G0 X0 Y-40 Z5
G1 X30 Y-40 F500
; CW arc, center placed so entry tangent is 45° from +X
; Center = (30 + r*sin(45°), -40 - r*cos(45°)) where r=10
; = (30 + 7.07, -40 - 7.07) = (37.07, -47.07)
G2 X44.14 Y-54.14 I7.071 J-7.071 F500
G1 X20 Y-54.14 F500

; --- ~90° mismatch ---
; Line along +X, then arc whose tangent is perpendicular (downward)
G0 X0 Y-70 Z5
G1 X30 Y-70 F500
; CW arc with center at (40, -70), r=10 → entry tangent is (0, -1)
G2 X40 Y-80 I10 J0 F500
G1 X10 Y-80 F500

G0 Z5
M30
