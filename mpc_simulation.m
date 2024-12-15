clear
close all
clc

% -------------------------------------------------------------------------
% Road data 불러오기 
road = load('road_data.txt');
t_road=road(:,1);
X_road=road(:,2);
Y_road=road(:,3);
beta_road=road(:,4);
yaw_rate_road=road(:,5);
delta_road=road(:,6);
load('V.mat');

% Analytic data 불러오기 / 7dof 시뮬레이션 결과(해석 모델 결과)
analytic = load('analytic_data.txt');
t_analytic=analytic(:,1);
X_analytic=analytic(:,2);
Y_analytic=analytic(:,3);
beta_analytic=analytic(:,4);
yaw_rate_analytic=analytic(:,5);

% LQR data 불러오기 / 7dof 시뮬레이션 결과(해석 모델 결과)
lqr = load('lqr_data.txt');
t_lqr=lqr(:,1);
X_lqr=lqr(:,2);
Y_lqr=lqr(:,3);
beta_lqr=lqr(:,4);
yaw_rate_lqr=lqr(:,5);
u_lqr=lqr(:,6);
% -------------------------------------------------------------------------


%% SIMULINK INTERFACE SETTINGS
% To use FORCESPRO in a Simulink model, there are two interfaces available:
%     *  A S-function Simulink Block using the non-inlined
%     FORCESPRO S-function (to use: set interface as 'sfunction')
%     * A MATLAB Code Simulink Block which uses the coder interface of FORCESPRO
%     and produces an inlined FORCESPRO S-function (to use: set interface as 'coder')
interface = 'sfunction';
% The FORCESPRO Simulink block is available in two versions, the standard
% and the compact. The compact version combines/stacks inputs and outputs that
% refer to the same problem matrices when the stage dimensions are the same.
% To use the standard version instead, set compact to false
compact = true;
% set to false to run whole execution without stopping at each step
interactive = true;


%% Prerequisites

% stop any previous executions and delete previous simulink models
target_model = 'mpc_controller';
% close_system(target_model, 0);


%% 1) MPC parameter setup

% system
Lf = 1.4;
Lr = 1.6;
m = 2000;
Jz = 4000;

Cf = 12e3; Cr = 11e3;
L = Lf+Lr;
V = 15; % LTI system (constant speed V = 15mps)

a = -2*(Cf+Cr)/(m*V);
b = -1-2*(Cf*Lf-Cr*Lr)/(m*V^2);
c = -2*((Cf*Lf-Cr*Lr)/Jz);
d = -2*(Cf*(Lf^2)+Cr*(Lr^2))/(Jz*V);

A = [a b; c d];
B = [0 ; 1/Jz];
E_u = [(2*Cf)/(m*V); (2*Cf*Lf)/Jz];

Q1 = 10^5; Q2 = 10^7;
Q = [Q1 0 ; 0 Q2]; R = [0.01]; % gain_1
% Q = [10^5 0 ; 0 10^7]; R = [0.01]; % gain_1
% Q = [10^6 0 ; 0 10^8]; R = [0.01]; % gain_2
% Q = [10^7 0 ; 0 10^9]; R = [0.01]; % gain_3

[~,S] = dlqr(A,B,Q,R);


[nx,nu] = size(B);
np = 3; % number of runtime parameter (references) (reference side-sipe angle, reference yaw rate, steering angle)
ny = 2; % number of output variables (side-slip angle, yaw rate)

C = eye(nx);
D = zeros(nx,1);

% weights in the objective
w_o_trac = [Q1 Q2]; % output reference tracking weight
w_i_trac = [0.01]; % control input tracking weight

% bounds
umin = -800;     umax = 800;
xmin = [-10*pi/180, -0.5]; xmax = [+10*pi/180, +0.5];


%% 2) FORCESPRO multistage form
% assume variable ordering z(i) = [ui; xi] for i=1...N

% dimensions
N           = 3;        % horizon length
model.N     = N;        % horizon lengtha
model.nvar  = nx+nu;    % number of variables
model.neq   = nx;       % number of equality constraints
model.npar  = np;        % number of runtime parameters
model.nh    = 0;           % number of nonlinear inequality constraints

% objective
model.objective = @(z, p) ([p(1); p(2)] - [z(2); z(3)])'*diag(w_o_trac)*([p(1); p(2)] - [z(2); z(3)]) + ...
    z(1)*diag(w_i_trac)*z(1);
model.objectiveN = @(z, p) ([p(1); p(2)] - [z(2); z(3)])'*S*([p(1); p(2)] - [z(2); z(3)]);

% z(1)   z(2)    z(3)
%  u1    x1       x2
%  Mz   beta   yaw rate

% equalities
integrator_stepsize = 0.001; % sampling time
model.eq = @(z,p) RK4(z(2:3), z(1), @(x,u,p) continuousDynamics(x,u,p),integrator_stepsize,p(3)); % 확인 필요

model.E = [zeros(nx,nu),eye(nx)]; % selection matrix

% inequalities
model.lb = [umin, xmin];
model.ub = [umax, xmax];


% initial state
model.xinitidx = nu+1:nu+nx;
% model.xinitidx = 2:3;

%% 3) Generate FORCESPRO solver

% setup options
codeoptions = getOptions('FORCESPRONLPsolver');
codeoptions.printlevel = 0;
codeoptions.cleanup = 0;
codeoptions.BuildSimulinkBlock = 1;

% generate code
output1 = newOutput('u0', 1, 1);
FORCES_NLP(model, codeoptions, output1);


%% 4) Model Initialization (needed for Simulink only)

% initial conditions
x_init = [0; 0];

% initial guess
x0 = zeros(model.N*model.nvar,1);

%% 5) Open Simulink Model

if compact
    forces_simulink_lib = [codeoptions.name, 'compact_lib'];
else
    forces_simulink_lib = [codeoptions.name, '_lib'];
end

open_system(target_model)

%% 6) Add FORCESPRO Simulink block to Simulink model

fullpath = fullfile(codeoptions.name, 'interface', forces_simulink_lib);
open_system(fullpath);


add_block([forces_simulink_lib, '/', codeoptions.name], [target_model, '/', codeoptions.name]);
close_system(forces_simulink_lib, 0);

set_param([target_model, '/', codeoptions.name], 'Position', [545   480   980   560]);

%% Simulation
x0 = repmat([0;0;0],N,1);
delta = yaw_rate_road; % equal for road case

target_model = 'mpc_controller';
open_system(target_model);


% Mux 블록 추가 및 선 추가
add_block('built-in/Mux', [gcs,'/Mux'],'Inputs',num2str(N),'DisplayOption','bar','position',[475   532   480   638]);
for i=1:N
    inportN = strcat('Mux/',num2str(i)); 
    add_line('mpc_controller','Mux1/1', inportN);
end
tic
out = sim(target_model);
simTime = toc;
disp(['Total simulation time: ',num2str(simTime), ' CPU sec']);

% Mux 블록 제거 및 선 제거
delete_block('mpc_controller/FORCESPRONLPsolver')
for i=1:N
    inportN = strcat('Mux/',num2str(i)); 
    delete_line('mpc_controller','Mux1/1', inportN);
end
delete_block('mpc_controller/Mux')


t_mpc = out.tout;
X_mpc = out.X.Data;
Y_mpc = out.Y.Data;
beta_mpc = out.beta.Data;
yaw_rate_mpc = out.yaw_rate.Data;
u_mpc = squeeze(out.u.Data);

%% Plot -------------------------------------------------------------------
figure
hold on
plot(X_road,Y_road)
plot(X_analytic,Y_analytic)
plot(X_lqr,Y_lqr)
plot(X_mpc,Y_mpc)
xlabel('X')
ylabel('Y')
legend('Road','Not controlled','LQR','MPC')
title('Global Trajectory')
grid on

figure
subplot(2,1,1)
hold on
plot(t_road,beta_road)
plot(t_analytic,beta_analytic)
plot(t_lqr,beta_lqr)
plot(t_mpc,beta_mpc)
xlabel('Time [s]')
ylabel('Side slip angle [rad]')
legend('Road','Not controlled','LQR','MPC')
title('Side slip angle')
grid on

subplot(2,1,2)
hold on
plot(t_road,yaw_rate_road)
plot(t_analytic,yaw_rate_analytic)
plot(t_lqr,yaw_rate_lqr)
plot(t_mpc,yaw_rate_mpc)
xlabel('Time [s]')
ylabel('Yaw rate [rad/s]')
legend('Road','Not controlled','LQR','MPC')
title('Yaw rate')
grid on

figure
hold on
plot(t_lqr,u_lqr)
plot(t_mpc,u_mpc)
xlabel('Time [s]')
ylabel('u')
legend('LQR','MPC')
title('Input u')
grid on


% MPC ---------------------------------------------------------------------
for i=1:1:length(t_analytic)
mpc_data(i,:)=[t_mpc(i),X_mpc(i),Y_mpc(i),beta_mpc(i),yaw_rate_mpc(i), u_mpc(i)];
end
filename = sprintf('mpc_data_N=%d_Q=[%d,%d]_R=%f.txt', N, w_o_trac(1),w_o_trac(2), w_i_trac);
writematrix(mpc_data, filename, 'FileType', 'text');
% -------------------------------------------------------------------------

