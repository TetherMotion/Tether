G21 ; Set units to millimeters
G90 ; Absolute positioning
G17 ; XY plane selection

G64 P5.0

; Start at origin (X0 Y0 Z5)
G0 Z5 ; Move to safe Z height

; First straight line (50mm in X direction)
G1 X50 Y0 F500 ; Move 50mm in X at feed rate 500

; Half-circle (G2 clockwise arc) with 15mm radius
; Center is 15mm below the current position (Y-15)
; End point is X50 Y-30 (50mm X, 30mm below start)
G2 X50 Y-30 I0 J-15 F500 ; Draw half-circle (180 degrees)

; Second straight line (50mm in X direction)
G1 X0 Y-30 F500 ; Move back

; Retract
G0 Z5 ; Lift tool to safe height

; End program
M30 ; Program end (optional, depending on your machine)
