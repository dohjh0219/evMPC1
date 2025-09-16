%% 1. 시뮬레이션 초기 설정 및 파라미터 정의
clc; clear; close all;

% 시뮬레이션 시간 및 차량 물리 파라미터
t_end = 12;         % 시뮬레이션 시간 [s]
dt = 0.01;          % 샘플링 시간 [s]
t = (0:dt:t_end)';  % 시간 벡터 (열벡터)

Lf = 1.4; Lr = 1.6; L = Lf + Lr; m = 2000;
Cf = 13500; Cr = 15100; Jz = 4000;

%% 2. 기하학적 목표 경로(Geometric Path) 데이터 생성
lane_width = 3.75;      % 차선 변경 폭 [m]
start_x = 30;           % 기동 시작 X 위치 [m]
change_length_1 = 50;   % 첫 번째 변경 구간 길이 [m]
straight_length = 30;   % 중간 직진 구간 길이 [m]
change_length_2 = 50;   % 두 번째 변경 구간 길이 [m]

x_path = linspace(0, 300, 10000); % 총 주행거리를 충분히 커버
y_path = zeros(size(x_path));

for i = 1:length(x_path)
    x = x_path(i);
    if x > start_x && x <= start_x + change_length_1, y_path(i) = lane_width/2 * (1 + tanh(6 * (x - (start_x + change_length_1/2)) / (change_length_1/2)));
    elseif x > start_x + change_length_1 && x <= start_x + change_length_1 + straight_length, y_path(i) = lane_width;
    elseif x > start_x + change_length_1 + straight_length && x <= start_x + change_length_1 + straight_length + change_length_2, y_path(i) = lane_width/2 * (1 - tanh(6 * (x - (start_x + change_length_1 + straight_length + change_length_2/2)) / (change_length_2/2)));
    end
end

% 경로의 곡률(Curvature) 계산
dY_dx = gradient(y_path, x_path(2)-x_path(1));
d2Y_dx2 = gradient(dY_dx, x_path(2)-x_path(1));
curvature = d2Y_dx2 ./ (1 + dY_dx.^2).^(3/2);

% Workspace에 'path' 구조체로 경로 데이터 저장 (Simulink 룩업테이블에서 사용)
path.x = x_path;
path.y = y_path;
path.curvature = curvature;


%% 3. 운전자 입력 및 이상적 참조 궤적 생성

% 동적 목표 속도 프로파일 (운전자가 의도하는 이상적인 속도)
v_target = zeros(size(t));
v_initial = 15.0; v_cruise = 25.0; v_final = 15.0;
t_accel_start = 2.0; t_accel_end = 6.0;
t_decel_start = 9.0; t_decel_end = 12.0;
for i = 1:length(t)
    t_current = t(i);
    if t_current < t_accel_start, v_target(i) = v_initial;
    elseif t_current >= t_accel_start && t_current < t_accel_end, v_target(i) = v_initial + (v_cruise - v_initial) * (t_current - t_accel_start) / (t_accel_end - t_accel_start);
    elseif t_current >= t_accel_end && t_current < t_decel_start, v_target(i) = v_cruise;
    else, v_target(i) = v_cruise + (v_final - v_cruise) * (t_current - t_decel_start) / (t_decel_end - t_decel_start);
    end
end

% 이상적인 참조 궤적 계산 (결과 비교/플로팅 용)
x_ideal = zeros(size(t)); y_ideal = zeros(size(t)); theta_ideal = zeros(size(t));
delta_ideal = zeros(size(t)); beta_ideal = zeros(size(t)); yaw_rate_ideal = zeros(size(t));
for i = 2:length(t)
    x_ideal(i) = x_ideal(i-1) + v_target(i-1) * cos(theta_ideal(i-1)) * dt;
    y_ideal(i) = y_ideal(i-1) + v_target(i-1) * sin(theta_ideal(i-1)) * dt;
    current_curvature = interp1(path.x, path.curvature, x_ideal(i), 'linear', 'extrap');
    yaw_rate_ideal(i) = v_target(i) * current_curvature;
    if v_target(i) > 1.0
        gain_yaw_rate = v_target(i) / (L + (m * v_target(i)^2 * (Lr * Cr - Lf * Cf)) / (2 * Cf * Cr * L));
        delta_ideal(i) = yaw_rate_ideal(i) / gain_yaw_rate;
        gain_beta = (Lr - (Lf * m * v_target(i)^2) / (2 * Cr * L)) / (L + (m * v_target(i)^2 * (Lr * Cr - Lf * Cf)) / (2 * Cf * Cr * L));
        beta_ideal(i) = gain_beta * delta_ideal(i);
    end
    theta_ideal(i) = theta_ideal(i-1) + yaw_rate_ideal(i-1) * dt;
end

% 개루프 페달 입력 생성 (수동 튜닝된 값)
acc_pedal = zeros(size(t)); decel_pedal = zeros(size(t));
ACCEL_PEDAL_INPUT = 0.5;
DECEL_PEDAL_INPUT = 0.4; 
acc_pedal(t >= t_accel_start & t < t_accel_end) = ACCEL_PEDAL_INPUT;
decel_pedal(t >= t_decel_start & t < t_decel_end) = DECEL_PEDAL_INPUT;


%% 4. 데이터 저장
% 1. Plant 모델 입력 (운전자 조작)
accdelta_Dataset = Simulink.SimulationData.Dataset;
accdelta_Dataset = accdelta_Dataset.addElement(timeseries(acc_pedal, t), 'acc');
accdelta_Dataset = accdelta_Dataset.addElement(timeseries(decel_pedal, t), 'decel');
accdelta_Dataset = accdelta_Dataset.addElement(timeseries(delta_ideal, t), 'delta');
Scenario = accdelta_Dataset;
save('accdelta.mat', 'Scenario');

% 2. Controller 모델 입력 (게인 스케줄링용 목표 속도)
velocity_Dataset = Simulink.SimulationData.Dataset;
velocity_Dataset = velocity_Dataset.addElement(timeseries(v_target, t), 'v');
Scenario = velocity_Dataset;
save('velocity_profile.mat', 'Scenario');


% ---------------------
file_output = fopen('reference_trajectory.txt', 'w');
fprintf(file_output, 't,x_ideal,y_ideal,beta_ideal,yaw_rate_ideal,delta_ideal,v_target\n'); % Header
for i=1:length(t)
    fprintf(file_output,'%f,%f,%f,%f,%f,%f,%f\n', ...
            t(i), x_ideal(i), y_ideal(i), ...
            beta_ideal(i), yaw_rate_ideal(i), delta_ideal(i), v_target(i));
end
fclose(file_output);

fprintf("Reference Trajectory Generation 끗~!\n");

