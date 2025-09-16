clc; clear; close all;

%% 1. 시뮬레이션 환경 설정

ref_traj_generator; 

load('lqr_gains.mat'); % V_vect, K_matrix를 Workspace로 로드

ref = readmatrix('reference_trajectory.txt');
t_ref      = ref(:,1);
X_ref      = ref(:,2);
Y_ref      = ref(:,3);
beta_ref   = ref(:,4);
yaw_rate_ref = ref(:,5);

%% 2. 베이스라인 (노 제어기) 시뮬레이션 실행

v_initial = 15.0; % 초기 속도 설정
target_model_analytic = 'analytic_model';
out_analytic = sim(target_model_analytic);

t_analytic = out_analytic.X.Time;
X_analytic = out_analytic.X.Data;
Y_analytic = out_analytic.Y.Data;
beta_analytic = out_analytic.beta.Data;
yaw_rate_analytic = out_analytic.yaw_rate.Data;

%% 3. GS-LQR 제어기 시뮬레이션 실행

target_model_lqr = 'lqr_controller';
out_lqr = sim(target_model_lqr);

t_lqr = out_lqr.X.Time;
X_lqr = out_lqr.X.Data;
Y_lqr = out_lqr.Y.Data;
beta_lqr = out_lqr.beta.Data;
yaw_rate_lqr = out_lqr.yaw_rate.Data;
u_lqr = out_lqr.u.Data; 


%% 4. 결과 비교 분석 및 시각화

% Global Trajectory Plot
figure('Name', 'Global Trajectory Comparison')
hold on; grid on; axis equal;
plot(X_ref, Y_ref, 'k--', 'LineWidth', 2);
plot(X_analytic, Y_analytic, 'b-', 'LineWidth', 1.5);
plot(X_lqr, Y_lqr, 'r-', 'LineWidth', 1.5);
xlabel('X [m]'); ylabel('Y [m]');
legend('Reference', 'Not Controlled (Baseline)', 'LQR Controlled');
title('Global Trajectory');

% State Variables Plot
figure('Name', 'State Variables Comparison')
subplot(2,1,1);
hold on; grid on;
plot(t_ref, beta_ref, 'k--', 'LineWidth', 2);
plot(t_analytic, beta_analytic, 'b-');
plot(t_lqr, beta_lqr, 'r-');
xlabel('Time [s]'); ylabel('Side slip angle [rad]');
legend('Reference', 'Not Controlled', 'LQR Controlled');
title('Side slip angle (\beta)');

subplot(2,1,2);
hold on; grid on;
plot(t_ref, yaw_rate_ref, 'k--', 'LineWidth', 2);
plot(t_analytic, yaw_rate_analytic, 'b-');
plot(t_lqr, yaw_rate_lqr, 'r-');
xlabel('Time [s]'); ylabel('Yaw rate [rad/s]');
legend('Reference', 'Not Controlled', 'LQR Controlled');
title('Yaw rate (r)');

% Control Input Plot
figure('Name', 'Control Input')
plot(t_lqr, u_lqr, 'm-', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]'); ylabel('Yaw Moment [Nm]');
title('LQR Control Input (M_z)');

%% 5. LQR 시뮬레이션 결과 파일로 저장
file_output = fopen('lqr_data.txt','w');
for i = 1:length(t_lqr)
    fprintf(file_output,'%f %f %f %f %f %f \n', t_lqr(i), X_lqr(i), Y_lqr(i), beta_lqr(i), yaw_rate_lqr(i), u_lqr(i));
end
fclose(file_output);
disp('"lqr_data.txt" 파일이 저장되었습니다.');