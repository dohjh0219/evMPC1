function f_lpv_discrete = generate_f_lpv_discrete()
    import casadi.*

    %% 차량 파라미터
    m = 1500;
    Iz = 3000;
    lf = 1.2;
    lr = 1.6;
    tr = 1.5;
    re = 0.3;
    Caf = 80000;
    Car = 80000;
    Cd = 0.3;
    Af = 2.2;
    rho = 1.225;
    f = 0.015;
    g = 9.81;

    dt = 0.05;     % 이산화 시간 간격
    vx0 = 15;      % 선형화 기준 속도

    %% 선형 시스템 행렬 (연속시간)
    A = [...
        -(lf^2*Caf + lr^2*Car)/(Iz*vx0),   -(lf*Caf - lr*Car)/Iz,                0;
        -(lf*Caf + lr*Car)/(m*vx0^2) - 1,  -(Caf + Car)/(m*vx0),                0;
        0,                                 0,                  -rho*Af*Cd*vx0/m];

    B = [...
        -tr/(2*Iz*re),   tr/(2*Iz*re);
        0,               0;
        1/(m*re),        1/(m*re)];

    D = [...
        lf*Caf/Iz;
        Caf/(m*vx0);
        0];

    E = [...
        0;
        0;
        rho*Af*Cd*vx0^2/(2*m) - f*g];

    %% 이산화 (MATLAB에서 미리 계산)
    Ad_num = expm(A*dt);
    Bd_num = A \ ((Ad_num - eye(3)) * B);
    Dd_num = A \ ((Ad_num - eye(3)) * D);
    Ed_num = A \ ((Ad_num - eye(3)) * E);

    % CasADi MX 타입으로 변환
    Ad = MX(Ad_num);
    Bd = MX(Bd_num);
    Dd = MX(Dd_num);
    Ed = MX(Ed_num);

    %% CasADi 심볼릭 변수 선언
    r    = MX.sym('r');
    beta = MX.sym('beta');
    vx   = MX.sym('vx');
    x    = [r; beta; vx];

    TRL  = MX.sym('TRL');
    TRR  = MX.sym('TRR');
    u    = [TRL; TRR];

    delta = MX.sym('delta');  % 조향 외란

    %% 이산화 시스템 모델
    x_next = Ad * x + Bd * u + Dd * delta + Ed;

    %% CasADi 함수 생성
    f_lpv_discrete = Function('f_lpv_discrete', {x, u, delta}, {x_next}, ...
                              {'x', 'u', 'delta'}, {'x_next'});

    % (Optional) 생성된 C 코드 저장
    % f_lpv_discrete.generate('f_lpv_discrete.c', struct('with_header', true));
end