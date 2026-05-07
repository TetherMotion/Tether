(Square rectangle with Sharp Corners vs Blended)
(Setup)
G21 (Metric)
G90 (Absolute)
F1000

(Move to Start)
G0 X0 Y0

(Rounded/Blended Path - G64 P-tolerance)
G64 P25.0 (Enable Path Blending with large 25mm tolerance)
G1 X0 Y50
G1 X50 Y50
G1 X50 Y0
G1 X0 Y0

M2 (End)
