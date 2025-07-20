%% 상태 및 입력 변수 선언 
import casadi.*

% 상태 변수: v_y (횡속도), r (요각속도), ω_FL, ω_FR, ω_RL, ω_RR (휠 각속도)
x = MX.sym('x',6,1);

v_y = x(1);
r   = x(2);
omega_FL = x(3);
omega_FR = x(4);
omega_RL = x(5);
omega_RR = x(6);

% 입력 변수: δ (조향각), T_FL, T_FR, T_RL, T_RR (각 바퀴 토크)
u = MX.sym('u',5,1);

delta   = u(1);
T_FL    = u(2);
T_FR    = u(3);
T_RL    = u(4);
T_RR    = u(5);

%% 주요 파라미터
m = 1500;        % 차량 질량 [kg], 예시값
I_z = 2500;      % 요 관성 모멘트 [kg·m^2]
l_f = 1.2;       % 무게중심 ~ 앞차축 거리 [m]
l_r = 1.6;       % 무게중심 ~ 뒤차축 거리 [m]
t_f = 1.5;       % 앞바퀴 윤거 [m]
t_r = 1.5;       % 뒷바퀴 윤거 [m]
J_w = 1.2;       % 바퀴 회전 관성 [kg·m^2]
r_e = 0.3;       % 타이어 유효 반경 [m]

C_alpha_f = 80000; % 앞 코너링 강성 [N/rad]
C_alpha_r = 80000; % 뒤 코너링 강성 [N/rad]
C_sigma = 10000;   % 종방향 강성 [N/slip]

v_x = 20; % 고정된 종방향 속도 [m/s]
mu = 0.9; % 마찰계수
g = 9.81; % 중력 가속도

% 기타 필요하면 추가 선언하세용

%% 타이어 위치 좌표
x_FL = l_f;  y_FL =  t_f/2;
x_FR = l_f;  y_FR = -t_f/2;
x_RL = -l_r; y_RL =  t_r/2;
x_RR = -l_r; y_RR = -t_r/2;

%% 타이어 접지점 좌표(차체 좌표계)
v_x_FL = v_x - r*y_FL;
v_y_FL = v_y + r*x_FL;

v_x_FR = v_x - r*y_FR;
v_y_FR = v_y + r*x_FR;

v_x_RL = v_x - r*y_RL;
v_y_RL = v_y + r*x_RL;

v_x_RR = v_x - r*y_RR;
v_y_RR = v_y + r*x_RR;

%% 타이어 조향각 변환 (앞바퀴만 조향)
v_xp_FL = v_x_FL*cos(delta) + v_y_FL*sin(delta);
v_yp_FL = -v_x_FL*sin(delta) + v_y_FL*cos(delta);

v_xp_FR = v_x_FR*cos(delta) + v_y_FR*sin(delta);
v_yp_FR = -v_x_FR*sin(delta) + v_y_FR*cos(delta);

% 뒷바퀴는 δ≈0 가정
v_xp_RL = v_x_RL;
v_yp_RL = v_y_RL;

v_xp_RR = v_x_RR;
v_yp_RR = v_y_RR;

%% 슬립각 계산 (소각 근사)
alpha_FL = v_yp_FL / v_xp_FL;
alpha_FR = v_yp_FR / v_xp_FR;
alpha_RL = v_yp_RL / v_xp_RL;
alpha_RR = v_yp_RR / v_xp_RR;

%% 종방향 속도 및 슬립률
V_FL = sqrt(v_x_FL^2 + v_y_FL^2);
V_FR = sqrt(v_x_FR^2 + v_y_FR^2);
V_RL = sqrt(v_x_RL^2 + v_y_RL^2);
V_RR = sqrt(v_x_RR^2 + v_y_RR^2);

lambda_FL = (r_e*omega_FL - V_FL)/max(abs(r_e*omega_FL), abs(V_FL));
lambda_FR = (r_e*omega_FR - V_FR)/max(abs(r_e*omega_FR), abs(V_FR));
lambda_RL = (r_e*omega_RL - V_RL)/max(abs(r_e*omega_RL), abs(V_RL));
lambda_RR = (r_e*omega_RR - V_RR)/max(abs(r_e*omega_RR), abs(V_RR));

%% 타이어 힘
F_y_FL = -C_alpha_f * alpha_FL;
F_y_FR = -C_alpha_f * alpha_FR;
F_y_RL = -C_alpha_r * alpha_RL;
F_y_RR = -C_alpha_r * alpha_RR;

F_x_FL = 0; % 앞바퀴 토크 미적용 가정
F_x_FR = 0;

F_x_RL = T_RL / r_e;
F_x_RR = T_RR / r_e;

%% 차량 횡방향, 요각 가속도 계산 (3-DOF 운동방정식)
% 횡방향 합력
sum_Fy = (F_y_FL + F_y_FR)*cos(delta) + F_y_RL + F_y_RR;
sum_Fx_front = (F_x_FL + F_x_FR)*sin(delta);

% 횡방향 가속도
v_y_dot = (sum_Fy + sum_Fx_front) / m - v_x * r;

% 요 각가속도
M_z = l_f*(F_y_FL + F_y_FR) - l_r*(F_y_RL + F_y_RR) + t_r/2*(F_x_RR - F_x_RL);

r_dot = M_z / I_z;

%% 바퀴 각가속도
omega_FL_dot = (T_FL - r_e * F_x_FL) / J_w;
omega_FR_dot = (T_FR - r_e * F_x_FR) / J_w;
omega_RL_dot = (T_RL - r_e * F_x_RL) / J_w;
omega_RR_dot = (T_RR - r_e * F_x_RR) / J_w;

%% 상태 미분 벡터 완성
xdot = vertcat(v_y_dot, r_dot, omega_FL_dot, omega_FR_dot, omega_RL_dot, omega_RR_dot);

%% CasADi 함수 생성
f = Function('nlplant_f', {x, u}, {xdot});

f.generate('nlplant_f.c', struct('with_header', true));