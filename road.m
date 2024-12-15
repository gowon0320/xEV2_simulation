clc
clear
close all

t_end = 10;
dt = 0.01;
t_road = 0:dt:t_end;

%% velocity, acceleration
v = zeros(size(t_road))';
acc = zeros(size(t_road));

a = 1.4;
b = 1.6;
L = a + b;% wheel base
V = 15; % constant speed
save('V.mat','V')

for i = 1:length(t_road)
    v(i) = V;
end


%% double lane changex
degree = 2.0;
delta_dot = deg2rad(degree);
delta = zeros(size(t_road));

steering_time = 1.0;

for i = 1:length(t_road)
    if t_road(i) <= steering_time
        delta(i) = 0;
    elseif t_road(i) <= steering_time+2.5
        delta(i) = delta_dot * sin(pi/2 * (t_road(i) - (steering_time)) / (0.25*2.5));
    elseif t_road(i) <= steering_time+3.3
        delta(i) = 0;
    elseif t_road(i) <= steering_time+5.8
        delta(i) = -delta_dot * sin(pi/2 * (t_road(i) - (steering_time)+1.7) / (0.25*2.5));
    end
end
%%
% % circular track 
% steering_time = 1.0;
% for i = 1:length(t_road)
%     if t_road(i) <= steering_time
%         delta(i) = 0;
%     elseif t_road(i) <= steering_time+0.625*4
%         delta(i) = delta_dot * sin(pi/2 * (t_road(i) - (steering_time)) / (4*0.25*2.5));
%         % delta(i) = delta_dot;
%     elseif t_road(i) <= steering_time+9
%         delta(i) = delta_dot;
%     end
% end


% v=v';
% delta=delta';


%% trajectory

% Based on "Rajamani Vehicle Dynamics and Control 8.2.3"
% stability 기반 beta, yaw_rate-------------------------------------------
m=2000;
Lf=1.4;
Lr=1.6;
L=3;
% Cf=12e3;
% Cr=11e3;
% Cf = 30e3;
% Cr = 30e3;
Cf = 13525.714;
Cr = 15166.667;

% k_ss    = -m*(Lf*Cf-Lf*Cr)/(2*Cf*Cr*L^2);
% G_beta  = (1-m*V^2*Lf/(2*Cr*L*Lr))/(1+k_ss)*Lr/L;
% G_omega = 1/(1+k_ss.*V^2).*V/L;
% % x1_ref = delta*G_beta;
% x2_ref = delta*G_omega;

x1_ref=(Lr-(Lf*m*V.^2)/(2*Cr*(Lf+Lr)))/((Lf+Lr)+(m*V.^2*(Lr*Cr-Lf*Cf))/(2*Cf*Cr*(Lf+Lr)))*delta;
x2_ref=V/(Lf+Lr+(m*V.^2*(Lr*Cr-Lf*Cf))/(2*Cf*Cr*L))*delta;

% -------------------------------------------------------------------------
x_road = zeros(size(t_road));
y_road = zeros(size(t_road));
theta = zeros(size(t_road));
x_road(1) = 0;
y_road(1) = 0;
theta(1) = 0;

for i = 2:length(t_road)
    theta(i) = theta(i-1) + x2_ref(i-1) * dt;
    x_road(i) = x_road(i-1) + v(i-1) * cos(theta(i)) * dt;
    y_road(i) = y_road(i-1) + v(i-1) * sin(theta(i)) * dt;
end

% x_road = zeros(size(t_road));
% y_road = zeros(size(t_road));
% theta = zeros(size(t_road));
% yaw_rate_road = zeros(size(t_road));
% beta_road = zeros(size(t_road));
% x_road(1) = 0;
% y_road(1) = 0;
% theta(1) = 0;
% yaw_rate_road(1) = 0;
% beta_road(1) = 0;
% 
% for i = 2:length(t_road)
%     beta_road(i) = -atan2(b*tan(delta(i-1)),L);
%     yaw_rate_road(i) = tan(delta(i-1))*v(i-1)/L;
%     theta(i) = theta(i-1) + yaw_rate_road(i-1) * dt;
%     x_road(i) = x_road(i-1) + v(i-1) * cos(theta(i)) * dt;
%     y_road(i) = y_road(i-1) + v(i-1) * sin(theta(i)) * dt;
% end



%% plot

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


figure
hold on
% plot(x1, y11, 'k--', 'LineWidth', 1.5); % double lane change
% plot(x1, y21, 'k--', 'LineWidth', 1.5);
% plot(x2, y12, 'k--', 'LineWidth', 1.5);
% plot(x2, y22, 'k--', 'LineWidth', 1.5);
% plot(x3, y13, 'k--', 'LineWidth', 1.5);
% plot(x3, y23, 'k--', 'LineWidth', 1.5);

plot(x_road,y_road)

xlabel('X [m]')
ylabel('Y [m]')
axis equal;
% xlim([-5 130])
% ylim([-10 10])
% title('ISO 3888-1:2018(E) Dashed Outline')
title('Global Trajectory')
hold off

grid on
% -------------------------------------------------------------------------

figure
subplot(3,1,1)
plot(t_road, v);
title('Velocity Profile');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
grid on;

subplot(3,1,2)
plot(t_road, acc);
title('Acceleration Profile');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
grid on;

subplot(3,1,3)
plot(t_road, delta);
title('Steering Profile');
xlabel('Time (s)');
ylabel('Steering Angle (rad)');
grid on;

% % kinematics기반 데이터
% file_output=fopen('road_data.txt','w');
% for i=1:1:length(t_road)
%     fprintf(file_output,'%f %f %f %f %f %f \n',t_road(i),x_road(i),y_road(i),beta_road(i),yaw_rate_road(i),delta(i));
% end
% fclose(file_output);

% -------------------------------------------------------
file_output=fopen('road_data.txt','w');
for i=1:1:length(t_road)
    fprintf(file_output,'%f %f %f %f %f %f \n',t_road(i),x_road(i),y_road(i),x1_ref(i),x2_ref(i),delta(i));
end
fclose(file_output);

file_output=fopen('ref_data.txt','w');
for i=1:1:length(t_road)
    fprintf(file_output,'%f, %f, \n',x1_ref(i),x2_ref(i));
end
fclose(file_output);

