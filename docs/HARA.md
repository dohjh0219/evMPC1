# Hazard Analysis and Risk Assessment (HARA)

## 1. Item Definition
본 문서는 **전기차용 토크 벡터링 제어 시스템(evMPC)**에 대한 위험 분석 및 위험 평가(HARA) 결과를 기술한다. 본 시스템은 차량의 요 모멘트(Yaw Moment)를 제어하여 선회 안정성을 향상시키는 기능을 수행한다.

* **Item**: Torque Vectoring Controller
* **Function**: Compute and generate differential torque for vehicle stability.
* **Target Vehicle**: Rear-Wheel Drive (RWD) Electric Vehicle.

## 2. Risk Assessment (ASIL Determination)
ISO 26262 Part 3에 의거하여 오작동 동작(Malfunction Behavior)과 주행 상황(Operational Situation)을 결합해 위험도(ASIL)를 산정한다.

### 2.1. Parameters
* **S (Severity)**: S0 (No injuries) ~ S3 (Life-threatening)
* **E (Exposure)**: E0 (Impossible) ~ E4 (High probability)
* **C (Controllability)**: C0 (Controllable in general) ~ C3 (Difficult to control)

### 2.2. HARA Table
| ID | Function | Malfunction | Operational Situation | Hazard Description | S | E | C | **ASIL** | Safety Goal ID |
|:--:|:--------:|:-----------:|:---------------------:|:------------------:|:-:|:-:|:-:|:--------:|:--------------:|
| **HZ-01** | 요 모멘트 생성 | **의도치 않은 최대 토크 출력**<br>(Unintended Max Torque) | 고속 주행 (100kph) 중<br>직진 주행 | 차량이 급격하게 한쪽으로 쏠리며 차선 이탈 및 충돌 발생 | S3 | E3 | C3 | **ASIL D** | **SG-01** |
| **HZ-02** | 요 모멘트 생성 | **반대 방향 토크 출력**<br>(Reverse Torque) | 곡선로 주행 중<br>(운전자 조향 입력 중) | 차량이 조향 의도와 반대로 거동하여 언더스티어/오버스티어 심화 | S3 | E3 | C2 | **ASIL C** | **SG-01** |
| **HZ-03** | 제어 신호 전송 | **통신 지연 및 두절**<br>(Communication Loss) | 일반 도심 주행 | 제어기 출력이 갱신되지 않아(Stuck) 구동축에 잘못된 토크 지속 인가 | S2 | E4 | C2 | **ASIL B** | **SG-02** |

---

## 3. Safety Goals (SG)
위험 분석 결과에 따라 최상위 안전 목표(Safety Goal)를 도출한다. 이 목표는 하위의 기능 안전 요구사항(FSR/SFR)으로 구체화된다.

### **[SG-01] 비의도적 요 모멘트 방지**
* **Description**: 제어기 오작동으로 인해 타이어 접지력을 상실시킬 수준의 과도한 요 모멘트가 발생해서는 안 된다.
* **Target ASIL**: **ASIL D** (보수적 관점 적용)
* **Safe State**: 제어 출력(Additional Yaw Moment)을 0으로 차단(Deactivate).
* **Derived Requirements**:
    * SRS -> `SFR-001` (출력 제한: ±800Nm)
    * SRS -> `SFR-003` (입력 유효성 검사)

### **[SG-02] 제어 루프 무결성 보장**
* **Description**: 통신 지연이나 연산 오류로 인해 오래된(Stale) 제어 값이 액추에이터에 전달되어서는 안 된다.
* **Target ASIL**: **ASIL B**
* **Safe State**: 통신 오류 3회 연속 감지 시 제어기 기능 정지(Standby 모드 전환).
* **Derived Requirements**:
    * SRS -> `SFR-002` (실행 시간 감시)
    * SRS -> `IFR-003` (Checksum 검사)

---

## 4. Traceability Summary
> HARA(Safety Goal)와 SRS(Safety Requirement) 간의 추적성 연결

* **SG-01** → `SFR-001`, `SFR-003`
* **SG-02** → `SFR-002`, `IFR-003`