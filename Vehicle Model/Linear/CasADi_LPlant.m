import casadi.*

%% 상태 및 입력 변수 선언
vy = MX.sym('vy');   % 횡방향 속도
r  = MX.sym('r');    % 요각속도
vx = MX.sym('vx');   % 종방향 속도

x = vertcat(vy, r, vx);

Mz = MX.sym('Mz');      % 제어 입력: 요잉 모멘트 토크
delta = MX.sym('delta');% 외란: 조향각 (관측 가능한 외란)

u = Mz; % 제어 입력 벡터 (scalar)
d = delta; % 외란 벡터 (scalar)

%% 차량 파라미터 (예시값)
m = 1500;      % 차량 질량 [kg]
Iz = 2500;     % 요 관성 모멘트 [kg·m^2]
lf = 1.2;      % 앞차축까지 거리 [m]
lr = 1.6;      % 뒷차축까지 거리 [m]

Cf = 80000;    % 앞 타이어 코너링 강성 [N/rad]
Cr = 80000;    % 뒤 타이어 코너링 강성 [N/rad]

rho = 1.225;   % 공기 밀도 [kg/m^3]
Cd = 0.3;      % 공기 저항 계수
Af = 2.2;      % 차량 정면 면적 [m^2]
f_roll = 0.015;% 구름 저항 계수
g = 9.81;      % 중력 가속도 [m/s^2]

%% 상태공간 모델 행렬 계산 (vx는 스케줄링 파라미터)

% 참고: vx는 상태이지만, 행렬 계산에선 스케줄링 변수로 간주

A = MX(3,3);
B = MX(3,1);
E = MX(3,1);

% 횡방향 속도(vy) 미분
A(1,1) = -(Cf + Cr)/ (m*vx);
A(1,2) = (-lf*Cf + lr*Cr) / (m*vx) - vx;
A(1,3) = 0; % 종방향 속도 영향(비선형에 가까워 무시)

% 요각속도(r) 미분
A(2,1) = (-lf*Cf + lr*Cr) / (Iz*vx);
A(2,2) = -(lf^2*Cf + lr^2*Cr) / (Iz*vx);
A(2,3) = 0;

% 종방향 속도(vx) 미분에 공기저항, 구름저항 포함
A(3,1) = 0;
A(3,2) = 0;
A(3,3) = -(0.5*rho*Cd*Af*vx + m*g*f_roll)/m;

% 제어 입력 행렬 B (요잉 모멘트 토크 영향)
B(1) = 0;
B(2) = 1/Iz;
B(3) = 0;

% 외란 입력 행렬 E (조향각 delta 영향)
E(1) = Cf / m;
E(2) = lf * Cf / Iz;
E(3) = 0;

%% 상태미분 계산

xdot = A*x + B*u + E*d;

%% CasADi 함수 생성
f = Function('vehicle_lpv', {x, u, d}, {xdot});

%% (필요시) 코드 생성
f.generate('vehicle_lpv.c', struct('with_header', true));