clear
close all
clc

%============차량 제원 =================================================
Lf = 1.4;
Lr = 1.6;
L = Lf+Lr; 

m = 2000;
Jz = 4000; % 차량 body 질량관성모멘트 

Cf = 13525;
Cr = 15166; 


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

    C = [1 0; 0 1];
    D = [0];

sys_continuous = ss(A, B, C, D); % 상태공간 모델 생성

% 샘플링 시간 설정
Ts = 0.01; % 예: 0.01초

% 이산화 (Zero-order hold 방식 사용)
sys_discrete = c2d(sys_continuous, Ts, 'zoh');

An = sys_discrete.A;
Bn = sys_discrete.B;

Q = [4.5e+04 0 ; 0 4.5e+06]; R = [0.01];
[K,S,P] = dlqr(An,Bn(:,1),Q,R);



