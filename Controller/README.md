# **✅ LPV-MPC 설계 전체 절차**

---

## **📌 1단계: 연속시간 LPV 모델 구성**

**모델 형태**

$$
\dot{x} = A(v_x)x + Bu + D(v_x)\delta + E
$$

- 상태 $x = [v_y, r, v_x]^T$
- 입력 $u = [T_{RL}, T_{RR}]^T$
- 외란 $\delta$: 조향각 (관측 가능한 외란)
- 파라미터 $v_x$: scheduling parameter로 동적으로 바뀜

**목표**: 이 모델을 기반으로, 매 step마다 $v_x$ 값에 따라 선형 시스템을 구성하여 예측에 사용

---

## **📌 2단계: 이산화 (Zero-Order Hold 기반)**

**이산 모델 형태**

$$
x_{k+1} = A_d(v_x) x_k + B_d u_k + D_d(v_x) \delta_k + E_d
$$

이산화는 시간 간격 $\Delta t$마다 입력을 유지(ZOH)한다고 가정하고 다음 수식 사용:

$$
A_d = \exp(A \Delta t), \quad
B_d = A^{-1}(A_d - I) B, \quad
D_d = A^{-1}(A_d - I) D, \quad
E_d = A^{-1}(A_d - I) E
$$

- `expm()`사용 가능 (CasADi 또는 MATLAB)
- 혹은 테일러 급수로 근사 (정밀도 조정 가능)
- $v_x$ 값은 각 스텝마다 바뀌므로 **step-by-step**으로 업데이트해야 함

---

## **📌 3단계: 예측 수평선 설정 (Prediction Horizon)**

- 예측 수평선 $N_p$: 10 ~ 20 정도 (너가 실험하면서 조정)
- 시간 간격 $\Delta t$: 0.05 ~ 0.1 sec

예측 모델을 반복:

```matlab
for k = 1:Np
    x(k+1) = A_d(vx_k)*x(k) + B_d*u(k) + D_d(vx_k)*delta(k) + E_d
end
```

- 각 스텝에서 $v_x$는 $x(k)(3)$ (세 번째 요소)로부터 가져옴
- 반복적인 선형 시스템 시뮬레이션

---

## **📌 4단계: 목적 함수 정의**

### **🎯 상태 추적 성능**

$$
J_1 = \sum_{k=1}^{N_p} (x(k) - x_{\text{ref}}(k))^T Q (x(k) - x_{\text{ref}}(k))
$$

- Q: 상태 오차 가중치 행렬

### **⚡ 에너지 소모 최소화**

$$
J_2 = \sum_{k=0}^{N_p-1} \frac{P_{\text{mech}}(k) + P_{\text{mech}}(k+1)}{2} \cdot \Delta t
$$

- 기계적 전력:
    
$$
P_{\text{mech}} = T_{ij} \cdot \omega_{ij} = \frac{T_{ij} \cdot v_x}{r_e}
$$
    

### **🔗 종합 목적함수**

$$
J = \lambda J_1 + (1 - \lambda) J_2
$$

- $\lambda$: 성능 vs 에너지 효율 간 트레이드오프 계수 (0.8~0.95 추천)

---

## **📌 5단계: 제약 조건 설정**

| **구분** | **제약** |
| --- | --- |
| **입력** | $T_{\text{min}} \leq T_{ij}(k) \leq T_{\text{max}}$ |
| **상태** | $\dot{\psi}{\min} \leq \dot{\psi}(k) \leq \dot{\psi}{\max}  \beta_{\min} \leq \beta(k) \leq \beta_{\max}  v_{\min} \leq v_x(k) \leq v_{\max}$ |
| **초기 조건** | $x(0) = x_{\text{meas}}$ |

옵저버가 없을 경우, 측정 가능한 상태만 사용하거나 추정값을 사용

---

## **📌 6단계: 최적화 문제 구성 및 CasADi Solver 적용**

```matlab
opti = casadi.Opti();

U = opti.variable(2, Np); % 입력 변수
X = opti.variable(3, Np+1); % 상태 변수
opti.subject_to(X(:,1) == x0); % 초기조건

for k=1:Np
    vx_k = X(3,k); % vx를 scheduling param으로 사용
    Ad_k = get_Ad(vx_k); % CasADi function으로 미리 생성
    Bd_k = get_Bd(vx_k); ...
    X(:,k+1) == Ad_k*X(:,k) + Bd_k*U(:,k) + ...
end

opti.minimize(J1 + J2);
opti.subject_to(...); % 상태, 입력 제약 조건
opti.solver('ipopt');
sol = opti.solve();
```

## **📌 7단계: Simulink 연동 (나중 단계)**

- MPC 코드를 MEX 파일로 빌드하거나
- MATLAB Function 블록에서 LPV-MPC solver를 직접 호출
- 또는 External Mode로 Real-Time 연결
