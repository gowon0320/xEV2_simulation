close all
clc


road = load('road_data.txt');
t_road=road(:,1);
X_road=road(:,2);
Y_road=road(:,3);
beta_road=road(:,4);
r_road=road(:,5);

nc = load('analytic_data.txt');
t_nc=nc(:,1);
X_nc=nc(:,2);
Y_nc=nc(:,3);
beta_nc=nc(:,4);
r_nc=nc(:,5);

lqr_unsat = load('lqr_data_unsaturated.txt');
t_lqr_unsat=lqr_unsat(:,1);
X_lqr_unsat=lqr_unsat(:,2);
Y_lqr_unsat=lqr_unsat(:,3);
beta_lqr_unsat=lqr_unsat(:,4);
r_lqr_unsat=lqr_unsat(:,5);
u_lqr_unsat=lqr_unsat(:,6);

lqr_sat = load('lqr_data_saturated.txt');
t_lqr_sat=lqr_sat(:,1);
X_lqr_sat=lqr_sat(:,2);
Y_lqr_sat=lqr_sat(:,3);
beta_lqr_sat=lqr_sat(:,4);
r_lqr_sat=lqr_sat(:,5);
u_lqr_sat=lqr_sat(:,6);

% 불러올거면 이거 (둘 중 하나는 주석)-------------------------
mpc = load('mpc_data.txt');
t_mpc=mpc(:,1);
X_mpc=mpc(:,2);
Y_mpc=mpc(:,3);
beta_mpc=mpc(:,4);
r_mpc=mpc(:,5);
u_mpc=mpc(:,6);
sol_time=mpc(:,7);


% % simulink 실행 후 (둘 중 하나는 주석)-------------------------
% t_mpc=out.tout;
% X_mpc = out.X_c.Data;
% Y_mpc = out.Y_c.Data;
% beta_mpc = out.beta_c.Data;
% r_mpc = out.r_c.Data;
% u_mpc = squeeze(out.u_c.Data);
% sol_time=squeeze(out.sol_time.Data);



% % ----------------------------------------------------------------
% file_output=fopen('analytic_data.txt','w');
% for i=1:1:length(t_mpc)
%     fprintf(file_output,'%f %f %f %f %f \n',t_mpc(i),X_mpc(i),Y_mpc(i),beta_mpc(i),r_mpc(i));
% end
% fclose(file_output);
% % -------------------------------------------------------------------------

% % ----------------------------------------------------------------
% file_output=fopen('lqr_data_saturated.txt','w');
% for i=1:1:length(t_mpc)
%     fprintf(file_output,'%f %f %f %f %f %f \n',t_mpc(i),X_mpc(i),Y_mpc(i),beta_mpc(i),r_mpc(i),u_mpc(i));
% end
% fclose(file_output);
% % -------------------------------------------------------------------------


% ----------------------------------------------------------------
append = zeros(length(t_mpc)-length(u_mpc),1);
u_mpc = [u_mpc;append];
append = zeros(length(t_mpc)-length(sol_time),1);
sol_time = [sol_time;append];

file_output=fopen('mpc_data.txt','w');
for i=1:1:length(t_mpc)
    fprintf(file_output,'%f %f %f %f %f %f %f \n',t_mpc(i),X_mpc(i),Y_mpc(i),beta_mpc(i),r_mpc(i),u_mpc(i),sol_time(i));
end
fclose(file_output);
% -------------------------------------------------------------------------

% Data length 일치하게 하도록 세팅
t=0:0.01:10;
u_mpc = u_mpc(1:length(t),1);
sol_time = sol_time(1:length(t),1);

%%
% % ISO 기준 width 1.4m 기준 double lane change------------------------------
% x1 = [0, 15];
% x2 = [45, 70];
% x3 = [95, 125];
%
% y11 = [0.895, 0.895];
% y12 = [4.5350, 4.5350];
% y13 = [1.1750, 1.1750];
%
% y21 = [-0.895, -0.895];
% y22 = [2.6050, 2.6050];
% y23 = [-0.895, -0.895];


% 색상 지정
color_nc = [0 0.4470 0.7410]; % 파란색 (기본)
color_lqr_unsat = [0.8500 0.3250 0.0980]; % 빨간색
color_lqr_sat = [0.9290 0.6940 0.1250]; % 노란색
color_mpc = [0.4940 0.1840 0.5560]; % 보라색

% Global Trajectory
figure
hold on
% plot(x1, y11, 'k--', 'LineWidth', 1.5, 'DisplayName', '')
% plot(x1, y21, 'k--', 'LineWidth', 1.5, 'DisplayName', '')
% plot(x2, y12, 'k--', 'LineWidth', 1.5, 'DisplayName', '')
% plot(x2, y22, 'k--', 'LineWidth', 1.5, 'DisplayName', '')
% plot(x3, y13, 'k--', 'LineWidth', 1.5, 'DisplayName', '')
% plot(x3, y23, 'k--', 'LineWidth', 1.5, 'DisplayName', '')
plot(X_road, Y_road, 'k--', 'LineWidth', 1, 'DisplayName', 'd') % road
plot(X_nc, Y_nc, 'Color', color_nc, 'DisplayName', 'nc') % nc
plot(X_lqr_unsat, Y_lqr_unsat, 'Color', color_lqr_unsat, 'DisplayName', 'lqr unsat') % lqr_unsat
plot(X_lqr_sat, Y_lqr_sat, 'Color', color_lqr_sat, 'DisplayName', 'lqr sat') % lqr_sat
plot(X_mpc, Y_mpc, 'Color', color_mpc, 'DisplayName', 'mpc') % mpc
xlabel('X [m]')
ylabel('Y [m]')
xlim([-10 160])
ylim([-2 3])
title('Global Trajectory')
legend show
hold off
grid on

% Side slip angle & Yaw rate
figure
subplot(2,1,1)
hold on
plot(t_road, beta_road, 'k--', 'LineWidth', 1, 'DisplayName', 'd') % road
plot(t_nc, beta_nc, 'Color', color_nc, 'DisplayName', 'nc') % nc
plot(t_lqr_unsat, beta_lqr_unsat, 'Color', color_lqr_unsat, 'DisplayName', 'lqr unsat') % lqr_unsat
plot(t_lqr_sat, beta_lqr_sat, 'Color', color_lqr_sat, 'DisplayName', 'lqr sat') % lqr_sat
plot(t_mpc, beta_mpc, 'Color', color_mpc, 'DisplayName', 'mpc') % mpc
xlabel('Time [s]')
ylabel('Side slip angle [rad]')
legend('show')
title('Side slip angle')
grid on

subplot(2,1,2)
hold on
plot(t_road, r_road, 'k--', 'LineWidth', 1, 'DisplayName', 'd') % road
plot(t_nc, r_nc, 'Color', color_nc, 'DisplayName', 'nc') % nc
plot(t_lqr_unsat, r_lqr_unsat, 'Color', color_lqr_unsat, 'DisplayName', 'lqr unsat') % lqr_unsat
plot(t_lqr_sat, r_lqr_sat, 'Color', color_lqr_sat, 'DisplayName', 'lqr sat') % lqr_sat
plot(t_mpc, r_mpc, 'Color', color_mpc, 'DisplayName', 'mpc') % mpc
xlabel('Time [s]')
ylabel('Yaw rate [rad/s]')
legend('show')
title('Yaw rate')
grid on

% Input u
figure
hold on
plot(t_lqr_unsat, u_lqr_unsat, 'Color', color_lqr_unsat, 'DisplayName', 'lqr unsat') % lqr_unsat
plot(t_lqr_sat, u_lqr_sat, 'Color', color_lqr_sat, 'DisplayName', 'lqr sat') % lqr_sat
plot(t, u_mpc, 'Color', color_mpc, 'DisplayName', 'mpc') % mpc
xlabel('Time [s]')
ylabel('u')
legend('show')
title('Input u (direct yaw moment)')
grid on

% Computational Time
figure
hold on
plot(t, sol_time, 'Color', color_mpc, 'DisplayName', 'mpc') % mpc
xlabel('Step')
ylabel('Time [ms]')
title('MPC computational time')
grid on

% figure
% hold on
% plot(t_c, iter)
% xlabel('Step')
% ylabel('Count')
% title('Iteration')
% grid on

%% Error 계산

% NC
[t_nc_unique, idx] = unique(t_nc, 'stable');

beta_nc_unique = beta_nc(idx);
beta_nc_interp = interp1(t_nc_unique, beta_nc_unique, t_road, 'linear', 'extrap');

r_nc_unique = r_nc(idx);
r_nc_interp = interp1(t_nc_unique, r_nc_unique, t_road, 'linear', 'extrap');

% LQR (Unsaturated)
[t_lqr_unsat_unique, idx] = unique(t_lqr_unsat, 'stable');

beta_lqr_unsat_unique = beta_lqr_unsat(idx);
beta_lqr_unsat_interp = interp1(t_lqr_unsat_unique, beta_lqr_unsat_unique, t_road, 'linear', 'extrap');

r_lqr_unsat_unique = r_lqr_unsat(idx);
r_lqr_unsat_interp = interp1(t_lqr_unsat_unique, r_lqr_unsat_unique, t_road, 'linear', 'extrap');

% LQR (Saturated)
[t_lqr_sat_unique, idx] = unique(t_lqr_sat, 'stable');

beta_lqr_sat_unique = beta_lqr_sat(idx);
beta_lqr_sat_interp = interp1(t_lqr_sat_unique, beta_lqr_sat_unique, t_road, 'linear', 'extrap');

r_lqr_sat_unique = r_lqr_sat(idx);
r_lqr_sat_interp = interp1(t_lqr_sat_unique, r_lqr_sat_unique, t_road, 'linear', 'extrap');

% MPC
[t_mpc_unique, idx] = unique(t_mpc, 'stable');

beta_mpc_unique = beta_mpc(idx);
beta_mpc_interp = interp1(t_mpc_unique, beta_mpc_unique, t_road, 'linear', 'extrap');

r_mpc_unique = r_mpc(idx);
r_mpc_interp = interp1(t_mpc_unique, r_mpc_unique, t_road, 'linear', 'extrap');


% Error 계산
error_beta_nc = beta_nc_interp - beta_road;
error_beta_lqr_unsat = beta_lqr_unsat_interp - beta_road;
error_beta_lqr_sat = beta_lqr_sat_interp - beta_road;
error_beta_mpc = beta_mpc_interp - beta_road;

error_r_nc = r_nc_interp - r_road;
error_r_lqr_unsat = r_lqr_unsat_interp - r_road;
error_r_lqr_sat = r_lqr_sat_interp - r_road;
error_r_mpc = r_mpc_interp - r_road;


% RMSE 계산
rmse_beta_nc = sqrt(mean((beta_nc_interp - beta_road).^2));
rmse_beta_lqr_unsat = sqrt(mean((beta_lqr_unsat_interp - beta_road).^2));
rmse_beta_lqr_sat = sqrt(mean((beta_lqr_sat_interp - beta_road).^2));
rmse_beta_mpc = sqrt(mean((beta_mpc_interp - beta_road).^2));

rmse_r_nc = sqrt(mean((r_nc_interp - r_road).^2));
rmse_r_lqr_unsat = sqrt(mean((r_lqr_unsat_interp - r_road).^2));
rmse_r_lqr_sat = sqrt(mean((r_lqr_sat_interp - r_road).^2));
rmse_r_mpc = sqrt(mean((r_mpc_interp - r_road).^2));


% RMSE 결과 출력
disp('Side slip angle RMSE:');
fprintf('NC: %.10f\n', rmse_beta_nc);
fprintf('LQR (Unsaturated): %.10f\n', rmse_beta_lqr_unsat);
fprintf('LQR (Saturated): %.10f\n', rmse_beta_lqr_sat);
fprintf('MPC: %.10f\n', rmse_beta_mpc);

disp('Yaw rate RMSE:');
fprintf('NC: %.10f\n', rmse_r_nc);
fprintf('LQR (Unsaturated): %.10f\n', rmse_r_lqr_unsat);
fprintf('LQR (Saturated): %.10f\n', rmse_r_lqr_sat);
fprintf('MPC: %.10f\n', rmse_r_mpc);


figure
subplot(2,1,1)
hold on
plot(t_road, error_beta_nc, 'Color', color_nc, 'DisplayName', 'nc')
plot(t_road, error_beta_lqr_unsat, 'Color', color_lqr_unsat, 'DisplayName', 'lqr unsat')
plot(t_road, error_beta_lqr_sat, 'Color', color_lqr_sat, 'DisplayName', 'lqr sat')
plot(t_road, error_beta_mpc, 'Color', color_mpc, 'DisplayName', 'mpc')
xlabel('Time [s]')
ylabel('Error')
title('Side slip angle error')
legend('show')
grid on

subplot(2,1,2)
hold on
plot(t_road, error_r_nc, 'Color', color_nc, 'DisplayName', 'nc')
plot(t_road, error_r_lqr_unsat, 'Color', color_lqr_unsat, 'DisplayName', 'lqr unsat')
plot(t_road, error_r_lqr_sat, 'Color', color_lqr_sat, 'DisplayName', 'lqr sat')
plot(t_road, error_r_mpc, 'Color', color_mpc, 'DisplayName', 'mpc')
xlabel('Time [s]')
ylabel('Error')
title('Yaw rate error')
legend('show')
grid on