close all
clear
clc

%% Road data 불러오기 
road = load('road_data.txt');
t_road=road(:,1);
X_road=road(:,2);
Y_road=road(:,3);
beta_road=road(:,4);
yaw_rate_road=road(:,5);
delta_road=road(:,6);
load('V.mat');

%% Analytic data 불러오기 / 7dof 시뮬레이션 결과(해석 모델 결과)
analytic = load('analytic_data.txt');
t_analytic=analytic(:,1);
X_analytic=analytic(:,2);
Y_analytic=analytic(:,3);
beta_analytic=analytic(:,4);
yaw_rate_analytic=analytic(:,5);

%% LQR simulation
target_model = 'lqr_controller';
open_system(target_model);
tic
out = sim(target_model);
simTime = toc;
disp(['Total simulation time: ',num2str(simTime), ' CPU sec']);

t = out.tout;
X_lqr = out.X.Data;
Y_lqr = out.Y.Data;
beta_lqr = out.beta.Data;
yaw_rate_lqr = out.yaw_rate.Data;
u_lqr = out.u.Data(:,3);


figure
hold on
plot(X_road,Y_road)
plot(X_analytic,Y_analytic)
plot(X_lqr,Y_lqr)
xlabel('X')
ylabel('Y')
legend('Road','Not controlled','LQR')
title('Global Trajectory')
grid on

figure
subplot(2,1,1)
hold on
plot(t_road,beta_road)
plot(t_analytic,beta_analytic)
plot(t,beta_lqr)
xlabel('Time [s]')
ylabel('Side slip angle [rad]')
legend('Road','Not controlled','LQR')
title('Side slip angle')
grid on

subplot(2,1,2)
hold on
plot(t_road,yaw_rate_road)
plot(t_analytic,yaw_rate_analytic)
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

% LQR ---------------------------------------------------------------------
file_output=fopen('lqr_data.txt','w');
for i=1:1:length(t_analytic)
    fprintf(file_output,'%f %f %f %f %f %f \n',t(i),X_lqr(i),Y_lqr(i),beta_lqr(i),yaw_rate_lqr(i), u_lqr(i));
end
fclose(file_output);
% -------------------------------------------------------------------------