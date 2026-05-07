G21 ; mm mode
G90 ; absolute
G17 ; XY plane

G64 P3.0

; === Overlapping blends: two 90° corners very close together ===
; The short segment (3mm) between corners is shorter than the
; blend tangent distance, so blends must handle overlap.

; --- Case 1: Two 90° left turns close together (U-turn) ---
G0 X0 Y0 Z5
G1 X30 Y0 F500       ; long line going right
G1 X30 Y3 F500        ; short line going up (90° left turn)
G1 X0 Y3 F500         ; long line going left (90° left turn)

; --- Case 2: Two 90° right turns close together ---
G0 X0 Y-15 Z5
G1 X30 Y-15 F500      ; long line going right
G1 X30 Y-18 F500      ; short line going down (90° right turn)
G1 X0 Y-18 F500       ; long line going left (90° right turn)

; --- Case 3: Three corners very close (zig-zag) ---
G0 X0 Y-35 Z5
G1 X20 Y-35 F500
G1 X20 Y-37 F500      ; 90° left, 2mm segment
G1 X22 Y-37 F500      ; 90° right, 2mm segment
G1 X22 Y-39 F500      ; 90° left
G1 X0 Y-39 F500

; --- Case 4: Overlapping with arcs —- line, short arc, line ---
G0 X0 Y-55 Z5
G1 X20 Y-55 F500
; Short 90° CW arc, r=2mm
G2 X22 Y-57 I2 J0 F500
G1 X22 Y-70 F500

G0 Z5
M30
