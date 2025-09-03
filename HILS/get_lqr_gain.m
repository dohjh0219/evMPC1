close all
clear; clc;

%% Vehicle Spec

Lf = 1.4;
Lr = 1.6;
L = Lf + Lr;

m = 2000;
Jz = 4000;

Cf = 13525;
Cr = 15166;

%% Simulation Environment

V = 15;

% Continuous State Space Model
a = -2 * (Cf+Cr) / (m*V);
b = -1-2*(Cf*Lf-Cr*Lr)/(m*V^2);
c = -2*((Cf*Lf-Cr*Lr)/Jz);
d = -2*(Cf*(Lf^2)+Cr*(Lr^2))/(Jz*V);


A = [a b;c d];
B = [0 (2*Cf)/(m*V); 1/Jz (2*Cf*Lf)/Jz];
C = [1 0; 0 1];
D = [0];

E = [(2*Cf)/(m*V) ; (2*Cf*Lf)/Jz]; 

sys_continous = ss(A,B,C,D);

% Sampling and Discretize
Ts = 0.01;
sys_discrete = c2d(sys_continous, Ts, 'zoh');

An = sys_discrete.A;
Bn = sys_discrete.B;

Q = [4.5e+04 0 ; 0 4.5e+06];
R = [0.01];

[K, S, P] = dlqr(An, Bn(:,1), Q, R)

