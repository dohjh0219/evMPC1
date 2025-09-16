clear; clc;

%% Road data, Velocity data 불러오기 
road = readmatrix('driving_profile.txt');
t_road = road(:,1);
X_road = road(:,2);
Y_road = road(:,3);
beta_road = road(:,4);
yaw_rate_road = road(:,5);
delta_road = road(:,6);
v = road(:,7);

load('velocity_profile.mat');

%% Analytic simulation
v_initial = v(1); 

target_model = 'analytic_model';
open_system(target_model);
tic
out = sim(target_model);
simTime = toc;
disp(['Total simulation time: ',num2str(simTime), ' CPU sec']);

t_analytic = out.tout;
X_analytic = out.X.signals.values;
Y_analytic = out.Y.signals.values;
beta_analytic = out.beta.signals.values;
yaw_rate_analytic = out.yaw_rate.signals.values;


figure
hold on
plot(X_road,Y_road)
plot(X_analytic,Y_analytic)
xlabel('X')
ylabel('Y')
legend('Reference','Analytic')
title('Global Trajectory')
grid on

figure
subplot(2,1,1)
hold on
plot(t_road,beta_road)
plot(t_analytic,beta_analytic)
xlabel('Time [s]')
ylabel('Side slip angle [rad]')
legend('Reference','Analytic')
title('Side slip angle')
grid on

subplot(2,1,2)
hold on
plot(t_road,yaw_rate_road)
plot(t_analytic,yaw_rate_analytic)
xlabel('Time [s]')
ylabel('Yaw rate [rad/s]')
legend('Reference','Analytic')
title('Yaw rate')
grid on

% Analytic ----------------------------------------------------------------
file_output=fopen('analytic_data.txt','w');
for i=1:1:length(t_analytic)
    fprintf(file_output,'%f %f %f %f %f \n',t_analytic(i),X_analytic(i),Y_analytic(i),beta_analytic(i),yaw_rate_analytic(i));
end
fclose(file_output);
% -------------------------------------------------------------------------