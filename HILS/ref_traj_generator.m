%% Initial Setting

clc; clear; close all;

t_end = 10; % 전체 시뮬 시간
dt = 0.01; % time step
t_road = 0:dt:t_end; % time vector

%% Double Lane Change : Steering Angle Generate

degree = 2.0;
delta_dot = deg2rad(degree);
delta = zeros(size(t_road));

steering_time = 1.0; % 스티어링 시작하는 시점.


% Steering Scenario -> 조향각 delta에 저장.
% 0 ~ 1초 : 직진
% 1 ~ 3.5초 : Sine파 조향입력(최대 delta_dot)
% 3.5초 ~ 4.3초 : 직진
% 4.3초 ~ 6.8초 : 반대 방향 Sine파 조향 입력

for i = 1:length(t_road)
    if t_road(i) <= steering_time
        delta(i) = 0;
    elseif t_road(i) <= steering_time + 2.5
        delta(i) = delta_dot * sin(pi/2 * (t_road(i) - (steering_time)) / (0.25 * 2.5));
    elseif t_road(i) <= steering_time + 3.3
        delta(i) = 0;
    elseif t_road(i) <= steering_time + 5.8
        delta(i) = -delta_dot * sin(pi/2 * (t_road(i) - (steering_time)+1.7) / (0.25 * 2.5));
    end
end

%% Vehicle Model : 2DoF Linear Bicycle Model

v = zeros(size(t_road)); % 속도 저장
acc = zeros(size(t_road)); % 가속도 저장

% Vehicle Spec
Lf = 1.4;
Lr = 1.6;
L = Lf + Lr; % Wheel Base

m = 2000; % Vehicle Mass
Cf = 13500; % Cornering Stiffness
Cr = 15100; 
Jz = 4000; % Inertia Moment

V = 15; % Vehicle Velocity
save('V.mat', 'V') % Velocity 데이터 파일 추출

for i = 1 : length(t_road)
    v(i) = V;
end

%% Reference Generator
% Side Slip Angle
x1_ref=(Lr-(Lf*m*V.^2)/(2*Cr*(Lf+Lr)))/((Lf+Lr)+(m*V.^2*(Lr*Cr-Lf*Cf))/(2*Cf*Cr*(Lf+Lr)))*delta;

% Yaw Rate
x2_ref=V/(Lf+Lr+(m*V.^2*(Lr*Cr-Lf*Cf))/(2*Cf*Cr*L))*delta;

%% RWS State-Space Model

a11 = -2*(Cf+Cr)/(m*V);
a12 = -1-2*(Cf*Lf-Cr*Lr)/(m*V^2);
a21 = -2*((Cf*Lf-Cr*Lr)/Jz);
a22 = -2*(Cf*(Lf^2)+Cr*(Lr^2))/(Jz*V);

A = [a11 a12 ;a21 a22];
B = [(2*Cf)/(m*V) (2*Cr)/(m*V); (2*Cf*Lf)/Jz -(2*Cr*Lr)/Jz]; %delta_f, delta_r

C = [1 0; 0 1];
D = [0];

sys_continuous = ss(A, B, C, D); % 상태공간 모델(Continuous Time domain) 생성


% Discretization : ZoH
Ts = 0.01;

sys_discrete = c2d(sys_continuous, Ts, 'zoh');

An = sys_discrete.A;
Bn = sys_discrete.B;


%% LQR Controller Setting(Option! Trajectory Generate에서는 안쓰임!)

Q = [1e-4 0 ; 0 1e-2];
R = [0.01];
[K, S, P] = dlqr(An, Bn(:,2), Q, R); 

%% Global Trajectory Generate(Reference Traj)
% Initial Setting
x_road = zeros(size(t_road));
y_road = zeros(size(t_road));

theta = zeros(size(t_road));

x_road(1) = 0;
y_road(1) = 0;
theta(1) = 0;

% Trajectory Generate
for i = 2 : length(t_road)
    % Numerial Integration 이용
    theta(i) = theta(i-1) + x2_ref(i-1) * dt;
    
    x_road(i) = x_road(i-1) + v(i-1) * cos(theta(i)) * dt;
    y_road(i) = y_road(i-1) + v(i-1) * sin(theta(i)) * dt;
end

%% Plotting

% Plot Global Trajectory
figure
hold on

plot(x_road, y_road)

xlabel('X [m]')
ylabel('Y [m]')
axis equal;

title('Global Trajectory')
hold off

grid on

% Plot Velocity, Acceleration, Steering
figure

subplot(3,1,1)
plot(t_road, v);
title('Velocity Profile');
xlabel('Time [s]');
ylabel('Velocity [m/s]');
grid on;

subplot(3,1,2)
plot(t_road, acc);
title('Acceleration Profile');
xlabel('Time [s]');
ylabel('Acceleration [m/s^2]');
grid on;

subplot(3,1,3)
plot(t_road, delta);
title('Steering Profile');
xlabel('Time [s]');
ylabel('Steering Angle [rad]');
grid on;


%% File Export

file_output = fopen('road_data.txt', 'w');

for i=1:1:length(t_road)
    fprintf(file_output,'%f %f %f %f %f %f \n',t_road(i),x_road(i),y_road(i),x1_ref(i),x2_ref(i),delta(i));
end
fclose(file_output);

file_output = fopen('ref_data.txt', 'w');
for i=1:1:length(t_road)
    fprintf(file_output,'%f, %f, \n',x1_ref(i),x2_ref(i));
end
fclose(file_output);
