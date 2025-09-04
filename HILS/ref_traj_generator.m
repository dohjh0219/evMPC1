clc; clear; close all;

%% Initial Setting

% Simulation Setting
t_end = 10;
dt = 0.01;
t_road = 0:dt:t_end;

% Vehicle Spectification Setting
Lf = 1.4;
Lr = 1.6;
L = Lf + Lr;

m = 2000;
Jz = 4000; 
Cf = 13500;
Cr = 15100;

V = 15;

%% Vehicle Model Definition(2-Dof Bicycle Model)

A_continuous = [-2*(Cf+Cr)/(m*V), -1-2*(Cf*Lf-Cr*Lr)/(m*V^2);
                -2*(Cf*Lf-Cr*Lr)/Jz, -2*(Cf*Lf^2+Cr*Lr^2)/(Jz*V)];

B_continuous = [(2*Cf)/(m*V), (2*Cr)/(m*V);
                (2*Cf*Lf)/Jz, -(2*Cr*Lr)/Jz];

C_matrix = eye(2);
D_matrix = [0];

sys_c = ss(A_continuous, B_continuous, C_matrix, D_matrix);

% Discretization : ZoH
sys_d = c2d(sys_c, dt, 'zoh');
Ad = sys_d.A;
Bd = sys_d.B;

%% Double Lane Change Scenario Generate

steering_start_time = 1.0;
degree = 2.0;
delta_dot_rad = deg2rad(degree);

delta = zeros(size(t_road));

for k = 1:length(t_road)
    time_now = t_road(k);
    
    if time_now > steering_start_time && time_now <= steering_start_time + 2.5
        delta(k) = delta_dot_rad * sin(pi/2 * (time_now - steering_start_time) / (0.25 * 2.5));
    elseif time_now > steering_start_time + 3.3 && time_now <= steering_start_time + 5.8
        delta(k) = -delta_dot_rad * sin(pi/2 * (time_now - (steering_start_time) + 1.7) / (0.25 * 2.5));
    else
        delta(k) = 0;
    end
end

%% Reference State Generate

% Reference Side Slip angle
x1_ref = ( Lr - (Lf*m*V^2)/(2*Cr*L) ) / ( L + (m*V^2*(Lr*Cr-Lf*Cf))/(2*Cf*Cr*L) ) * delta;

% Reference Yaw Rate
x2_ref = V / ( L + (m*V^2*(Lr*Cr-Lf*Cf))/(2*Cf*Cr*L) ) * delta;

%% Global Trajectory Generate

x_road = zeros(size(t_road));
y_road = zeros(size(t_road));
theta = zeros(size(t_road));

x_road(1) = 0;
y_road(1) = 0;
theta(1) = 0;

for i = 1:length(t_road)-1
    theta(i+1) = theta(i) + x2_ref(i) * dt;

    x_road(i+1) = x_road(i) + V * cos(theta(i+1)) * dt;
    y_road(i+1) = y_road(i) + V * sin(theta(i+1)) * dt;
end

%% Export Profile

fid = fopen('driving_profile.txt', 'w');
for i = 1:length(t_road)
    fprintf(fid, '%f %f %f %f %f %f \n', t_road(i), x_road(i), y_road(i), x1_ref(i), x2_ref(i), delta(i));
end
fclose(fid);

fid = fopen('ref_data.txt', 'w');
for i = 1:length(t_road)
    fprintf(fid, '%f, %f, \n', x1_ref(i), x2_ref(i));
end
fclose(fid);

save('V.mat', 'V');

%% Plot
% Global X-Y Trajectory
figure('Name', 'Global Trajectory');
plot(x_road, y_road, 'b-', 'LineWidth', 2);
title('Generated Global Trajectory (Double Lane Change)');
xlabel('X-Position [m]');
ylabel('Y-Position [m]');
axis equal;
grid on;
legend('Vehicle Path');

% Vehicle State Profiles
figure('Name', 'State Profiles');
% 속도 프로파일
subplot(3, 1, 1);
plot(t_road, ones(size(t_road)) * V, 'k-');
title('Velocity Profile');
xlabel('Time [s]');
ylabel('Velocity [m/s]');
ylim([V-5 V+5]);
grid on;

% 가속도 프로파일
subplot(3, 1, 2);
plot(t_road, zeros(size(t_road)), 'k-');
title('Acceleration Profile');
xlabel('Time [s]');
ylabel('Acceleration [m/s^2]');
ylim([-1 1]);
grid on;

% 조향각 프로파일
subplot(3, 1, 3);
plot(t_road, rad2deg(delta), 'r-');
title('Steering Profile');
xlabel('Time [s]');
ylabel('Steering Angle [deg]');
grid on;