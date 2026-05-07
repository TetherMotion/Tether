(G64 Staircase Demo - Inside and Outside Strategies)
(Shows 5 steps with inside (positive P), then 5 steps with outside (negative P))
(Step size = 5mm in X and Y)
G21 G90
F1000

G0 X0 Y0

(First 5 steps: Inside strategy - positive P)
G64 P3.0
G1 X0 Y0
G1 X0 Y5
G1 X5 Y5
G1 X5 Y10
G1 X10 Y10
G1 X10 Y15
G1 X15 Y15
G1 X15 Y20
G1 X20 Y20
G1 X20 Y25

(Next 5 steps: Outside/Dogbone strategy - negative P)
G64 P-3.0
G1 X25 Y25
G1 X25 Y30
G1 X30 Y30
G1 X30 Y35
G1 X35 Y35
G1 X35 Y40
G1 X40 Y40
G1 X40 Y45
G1 X45 Y45
G1 X45 Y50

(Final 5 steps: Back to inside strategy)
G64 P3.0
G1 X50 Y50
G1 X50 Y55
G1 X55 Y55
G1 X55 Y60
G1 X60 Y60
G1 X60 Y65
G1 X65 Y65
G1 X65 Y70
G1 X70 Y70
G1 X70 Y75

M2
