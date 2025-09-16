clear
close all
clc

%% Vehicle dynamics Specification 
Lf = 1.4;
Lr = 1.6;
L = Lf+Lr; 

m = 2000;
Jz = 4000; % 차량 body 질량관성모멘트 

Cf = 13525;
Cr = 15166; 

%% Getting GS LQR Gain
% 게인을 계산할 속도 벡터 정의 (예: 5 ~ 30 m/s)
V_vect = (5:1:30)'; % 속도 벡터
K_matrix = [];      % 게인을 저장할 빈 행렬 초기화

% 2. for 루프를 사용하여 속도별 게인 계산
for i = 1:length(V_vect)
    V = V_vect(i); % 현재 속도
    
    % 연속 상태공간 모델 정의 (속도 V에 따라 행렬이 변함)
    a = -2*(Cf+Cr)/(m*V);
    b = -1-2*(Cf*Lf-Cr*Lr)/(m*V^2);
    c = -2*((Cf*Lf-Cr*Lr)/Jz);
    d = -2*(Cf*(Lf^2)+Cr*(Lr^2))/(Jz*V);
    A = [a b;c d];
    B = [0 (2*Cf)/(m*V); 1/Jz (2*Cf*Lf)/Jz];
    E=[(2*Cf)/(m*V) ; (2*Cf*Lf)/Jz]; 

    C = [1 0; 0 1];
    D = [0];

    sys_continuous = ss(A, B, C, D);

    % 이산화
    Ts = 0.01;
    sys_discrete = c2d(sys_continuous, Ts, 'zoh');
    An = sys_discrete.A;
    Bn = sys_discrete.B;

    % LQR 게인 계산
    Q = [4.5e+04 0 ; 0 4.5e+06]; R = [0.01];
    [K,S,P] = dlqr(An,Bn(:,1),Q,R);
    
    % 계산된 게인 K를 K_matrix에 추가
    K_matrix = [K_matrix; K];
end

% 3. 생성된 속도 벡터와 게인 행렬을 .mat 파일로 저장
save('lqr_gains.mat', 'V_vect', 'K_matrix');
disp('lqr_gains.mat 파일이 성공적으로 생성되었습니다.');
disp('K_matrix 크기:');
disp(size(K_matrix));