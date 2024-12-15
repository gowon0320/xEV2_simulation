function [xDot] = continuousDynamics(x, u, p)

% state x = [x1, x2]
%       [sideslip angle, yaw rate]
% input u = [u1]
%           [Mz]

%% set parameters
Lf = 1.4;
Lr = 1.6;
m = 2000;
Jz = 4000;

Cf = 12e3; Cr = 11e3;
L = Lf+Lr;
V = 15;

a = -2*(Cf+Cr)/(m*V);
b = -1-2*(Cf*Lf-Cr*Lr)/(m*V^2);
c = -2*((Cf*Lf-Cr*Lr)/Jz);
d = -2*(Cf*(Lf^2)+Cr*(Lr^2))/(Jz*V);
A = [a b;c d];
B = [0 ; 1/Jz];
E_u = [(2*Cf)/(m*V) ; (2*Cf*Lf)/Jz];

xDot = [ A(1,:)*[x(1);x(2)] + B(1,1)*u(1)+E_u(1,1)*p(1);
    A(2,:)*[x(1);x(2)] + B(2,1)*u(1)+E_u(2,1)*p(1)];


end
