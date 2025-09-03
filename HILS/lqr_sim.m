close all
clear; clc;

%% Driving Profile 및 속도 데이터 로딩

road = load('driving_profile.txt');
t_road = road(:,1);
X_road = road(:,2);
Y_road = road(:,3);
beta_road = road(:,4);
yaw_rate_road = road(:,5);
delta_road = road(:,6);

load('V.mat');

%% Baseline Data 로딩

baseline = load('baseline_data.txt');
t_baseline = baseline(:,1);
X_baseline = baseline(:,2);
Y_baseline = baseline(:,3);
beta_baseline = baseline(:,4);
yaw_rate_baseline = baseline(:,5);

%% LQR Simulation

target_model = 'lqr_controller';
open_system(target_model);

tic % timer 

out = sim(target_model);
simTime = toc;
disp(['Total Simulation Time : ', num2str(simTime), 'CPU sec']);

% Data 해석
t = out.tout;
X_lqr = out.X.Data;
Y_lqr = out.Y.Data;
beta_lqr = out.beta.Data;
yaw_rate_lqr = out.yaw_rate.Data;

u_lqr = out.u.Data * 180 / pi; % External Yaw Moment(degree 단위?)

%% Plot
figure
hold on
plot(X_road, Y_road)
plot(X_baseline, Y_baseline)
plot(X_lqr, Y_lqr)
xlabel('X')
ylabel('Y')
legend('Road','Not controlled','LQR')
title('Global Trajectory')
grid on

figure
subplot(2,1,1)
hold on
plot(t_road, beta_road)
plot(t_baseline, beta_baseline)
plot(t, beta_lqr)
xlabel('Time [s]')
ylabel('Side slip angle [rad]')
legend('Road','Not controlled','LQR')
title('Side slip angle')
grid on

subplot(2,1,2)
hold on
plot(t_road,yaw_rate_road)
plot(t_baseline, yaw_rate_baseline)
plot(t,yaw_rate_lqr)
xlabel('Time [s]')
ylabel('Yaw rate [rad/s]')
legend('Road','Not controlled','LQR')
title('Yaw rate')
grid on

figure
plot(t,u_lqr)
xlabel('Time [s]')
ylabel('u')
title('Input u')
grid on

%% LQR Data Export
file_output = fopen('lqr_data.txt', 'w');

for i=1:1:length(t_baseline)
    fprintf(file_output,'%f %f %f %f %f %f \n',t(i),X_lqr(i),Y_lqr(i),beta_lqr(i),yaw_rate_lqr(i), u_lqr(i));
end

fclose(file_output);