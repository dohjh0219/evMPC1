import casadi.*

% Parameters
Np = 10;        % 예측 수평선
dt = 0.1;       % 샘플 시간

% 초기 상태, 입력, 외란 초기값
xk = MX.sym('xk', 3);           % [r; beta; vx]
uk = MX.sym('uk', 2);           % [TRL; TRR]
deltak = MX.sym('deltak');      % delta (조향각)

% f_lpv_discrete(x, u, delta) 형태로 정의된 CasADi 함수 사용
% 상태 저장용 변수
X = MX.zeros(3, Np+1);
U = MX.sym('U', 2, Np);         % 예측 구간 입력
Delta = MX.sym('Delta', 1, Np); % 예측 구간 외란

% 초기 상태 설정
X(:,1) = xk;

for k = 1:Np
    X(:,k+1) = f_lpv_discrete(X(:,k), U(:,k), Delta(k));
end

% 출력: X (예측 상태 시퀀스), U (입력 시퀀스)