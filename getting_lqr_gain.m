clear
close all
clc

%============차량 제원 =================================================
Lf = 1.4;
Lr = 1.6;
m = 2000;
Jz = 4000;

% Cf = 12e3;
% Cr = 11e3;

Cf = 13525;
Cr = 15166;

L = Lf+Lr; 


% Q = [5*10^5 0 ; 0 5*10^7]; R = [0.01];
% Q = [3*10^5 0 ; 0 3*10^7]; R = [0.01]; % lqr gain

% Q = [3.5e+04 0 ; 0 3.5e+06]; R = [0.01];
% Q = [4e+04 0 ; 0 4e+06]; R = [0.01];
Q = [4.5e+04 0 ; 0 4.5e+06]; R = [0.01];

%=======================================================================


V=15;


% 연속 상태공간 모델 정의
    a = -2*(Cf+Cr)/(m*V);
    b = -1-2*(Cf*Lf-Cr*Lr)/(m*V^2);
    c = -2*((Cf*Lf-Cr*Lr)/Jz);
    d = -2*(Cf*(Lf^2)+Cr*(Lr^2))/(Jz*V);
    A = [a b;c d];
    B = [0 (2*Cf)/(m*V); 1/Jz (2*Cf*Lf)/Jz];
    E=[(2*Cf)/(m*V) ; (2*Cf*Lf)/Jz]; 

    C = [ 1 0; 0 1];
    D = [0];

sys_continuous = ss(A, B, C, D); % 상태공간 모델 생성

% 샘플링 시간 설정
Ts = 0.01; % 예: 0.01초

% 이산화 (Zero-order hold 방식 사용)
sys_discrete = c2d(sys_continuous, Ts, 'zoh');

An = sys_discrete.A;
Bn = sys_discrete.B;

[K,S,P] = dlqr(An,Bn(:,1),Q,R);



% for i=1:1:100
%     V(i)=i*0.2;
% end
% 
% K = zeros(1,2);
% 
% for i=1:1:100
%     a = -2*(Cf+Cr)/(m*V(i));
%     b = -1-2*(Cf*Lf-Cr*Lr)/(m*V(i)^2);
%     c = -2*((Cf*Lf-Cr*Lr)/Jz);
%     d = -2*(Cf*(Lf^2)+Cr*(Lr^2))/(Jz*V(i));
%     A = [a b;c d];
%     B = [0 ; 1/Jz];
% 
%     K = lqr(A,B,Q,R);
%     Ve(i,:) = [V(i); K(1); K(2)];
% end


% file_output=fopen('gain.txt','w');
% for i=1:1:100
%     fprintf(file_output,'%f %f %f\n',Ve(i,1),Ve(i,2),Ve(i,3));
% end
% fclose(file_output);
