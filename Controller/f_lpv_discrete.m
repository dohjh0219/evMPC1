function f_lpv_discrete = generate_f_lpv_discrete()
    import casadi.*

    % 차량 파라미터
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

    dt = 0.05;  % 샘플 시간

    % 심볼릭 변수 정의
    r    = MX.sym('r');
    beta = MX.sym('beta');
    vx   = MX.sym('vx');
    x    = [r; beta; vx];

    TRL  = MX.sym('TRL');
    TRR  = MX.sym('TRR');
    u    = [TRL; TRR];

    delta = MX.sym('delta');
    vx0 = MX.sym('vx0'); % scheduling 변수 (속도)

    % 시스템 행렬 (연속시간)
    A = [...
        -(lf^2*Caf + lr^2*Car)/(Iz*vx0),   -(lf*Caf - lr*Car)/Iz,                0;
        -(lf*Caf - lr*Car)/(m*vx0^2) - 1,  -(Caf + Car)/(m*vx0),                0;
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

    % 연속 시스템 행렬을 하나의 큰 행렬로 묶기 (ZOH 이산화를 위해)
    M = [A, B, D, E; zeros(4,7)];

    % 상태수 3, 입력수 2, 외란 1, 상수 1 => 총 7 상태확장

    % 행렬 M * dt 에 대한 matrix exponential (이산화)
    Md = expm(M*dt);

    % 분할
    Ad = Md(1:3,1:3);
    Bd = Md(1:3,4:5);
    Dd = Md(1:3,6);
    Ed = Md(1:3,7);

    % 다음 상태식
    x_next = Ad * x + Bd * u + Dd * delta + Ed;

    % CasADi 함수 생성
    f_lpv_discrete = Function('f_lpv_discrete', {x, u, delta, vx0}, {x_next}, ...
        {'x','u','delta','vx0'}, {'x_next'});
end