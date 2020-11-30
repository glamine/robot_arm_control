% ROBOT PARAMETERS : 
Klin = 5;
Krot = 5;

a1 = 0;
a2 = 0.2;
a3 = 0.2;

d1 = 0.2;%adapt au sol
d5 = 0.07;%dépend pince


%Motors
Rw = 0.03;

ratio = 19;
gear_eff = 0.85;

M = 15;
Jmotor = 12e-7;
Jeq = M*Rw^2/4; %(m)*r^2/4
%(12*0.055^2)/14

Jrob = Jmotor + Jeq;
Kv = 4.09e-4; 
tau_m = Jrob/Kv;

Ra = 5.84;
Lint = 560e-6;

La = Lint; 
kphi = 37.83e-3;

tau_e = La/Ra;


K = 1;