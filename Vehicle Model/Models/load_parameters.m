% -----------------------------------------------------------
% Project: evMPC Torque Vectoring System
% Description: Parameter definitions for Controller & Safety Logic
% Requirement Ref: SRS-001, PERF-001
% -----------------------------------------------------------

%% 1. Safety Parameters (ASIL B)
% [SFR-001] Max Torque Limitation
MAX_TORQUE_NM = 800; 
MIN_TORQUE_NM = -800;

%% 2. Control Parameters
% [FR-002] LQR Control Loop Time
Ts = 0.01; % 10ms sampling time

% Vehicle Specs (Generic EV)
Mass = 1800; % kg
Lf = 1.2; % m (CG to Front)
Lr = 1.4; % m (CG to Rear)

disp('Log: All parameters loaded successfully.');