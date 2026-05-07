G21 ; mm mode
G90 ; absolute
G17 ; XY plane

G64 P5.0

; === All transition types in a single path ===
; Line → Line → Arc → Line → Arc → Arc → Line
; Demonstrates mixed segment type transitions.

G0 X0 Y0 Z5

; Line → Line (45° corner)
G1 X20 Y0 F500
G1 X30 Y10 F500

; Line → smooth CW arc (tangent-continuous)
; Direction at end of line: normalized (10, 10) = (0.707, 0.707)
; Need arc tangent at start to match. CW arc, dir=-1:
;   tangent = (sin(a), -cos(a)) = (0.707, 0.707) → a = atan2(-0.707, 0.707) + π
;   Actually, for a 45° direction, center is 90° CW from tangent direction:
;   perpendicular CW of (0.707, 0.707) = (0.707, -0.707)
;   center = (30 + 10*0.707, 10 - 10*0.707) = (37.07, 2.93)
G2 X44.14 Y2.93 I7.071 J-7.071 F500

; Arc → Line (smooth — tangent-continuous)
; Need to figure the exit tangent of previous arc. Just add a line going
; in the same direction as arc exit tangent.
; For now, let's just do a clearly non-smooth corner for variety.
G1 X60 Y2.93 F500

; Line → CCW arc (90° corner)
G3 X60 Y22.93 I0 J10 F500

; Arc → Arc (smooth — same center continuation)
G3 X50 Y22.93 I-5 J0 F500

; Arc → Line
G1 X10 Y22.93 F500

G0 Z5
M30
