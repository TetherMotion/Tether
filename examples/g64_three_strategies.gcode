(G64 Demo - Three Strategies)
(Demonstrates inside, dogbone, and teardrop using sign convention)
(Positive P = inside, Negative P = dogbone/outside)
G21 G90
F1000

(Path with 3 corners, each with different strategy)
G0 X0 Y0

(Corner 1: Inside strategy - positive P stays inside the corner)
G64 P5.0
G1 X0 Y0
G1 X0 Y40
G1 X40 Y40

(Corner 2: Dogbone/Outside strategy - negative P stays outside)
G64 P-5.0
G1 X40 Y80

(Corner 3: Use centered mode for variety)
G64 P0
G1 X80 Y80
G1 X80 Y40

M2
