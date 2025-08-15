import casadi.*

% ---- Parameters ----
m = 1500;           % mass [kg]
Iz = 3000;          % yaw moment of inertia [kg*m^2]
lf = 1.2;           % distance from CG to front axle [m]
lr = 1.6;           % distance from CG to rear axle [m]
tr = 1.5;           % rear tread [m]
re = 0.3;           % effective radius [m]
Caf = 80000;        % front cornering stiffness [N/rad]
Car = 80000;        % rear cornering stiffness [N/rad]
Cd = 0.3;           % air drag coefficient
Af = 2.2;           % frontal area [m^2]
rho = 1.225;        % air density [kg/m^3]
f = 0.015;          % rolling resistance coefficient
g = 9.81;           % gravity

% ---- States and Inputs ----
r = MX.sym('r');        % yaw rate
beta = MX.sym('beta');  % slip angle
vx = MX.sym('vx');      % longitudinal velocity
x = [r; beta; vx];

TRL = MX.sym('TRL');    % rear left torque input
TRR = MX.sym('TRR');    % rear right torque input
u = [TRL; TRR];

delta = MX.sym('delta');  % front steering angle (disturbance)

vx0 = MX.sym('vx0');  % scheduling parameter

% ---- System matrices (LPV) ----

A = [...
    -(lf^2*Caf + lr^2*Car)/(Iz*vx0),   -(lf*Caf - lr*Car)/Iz,                0;
    -(lf*Caf - lr*Car)/(m*vx0^2) - 1,  -(Caf + Car)/(m*vx0),                0;
    0,                                 0,                  -rho*Af*Cd*vx0/m];

B = [...
    -tr/(2*Iz*re),   tr/(2*Iz*re);
    0,               0;
    1/(m*re),        1/(m*re)];

D = [...
    lf*Caf/Iz;
    Caf/(m*vx0);
    0];

E = [...
    0;
    0;
    rho*Af*Cd*vx0^2/(2*m) - f*g];

xdot = A*x + B*u + D*delta + E;

% ---- CasADi function ----
f_lpv = Function('f_lpv', {x, u, delta, vx0}, {xdot}, ...
    {'x', 'u', 'delta', 'vx0'}, {'xdot'});
