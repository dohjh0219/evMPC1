# ISO 26262-Compliant EV Torque Vectoring System
![MATLAB](https://img.shields.io/badge/MATLAB-R2024a-orange.svg) ![Simulink](https://img.shields.io/badge/Simulink-MBD-blue.svg) ![License](https://img.shields.io/badge/License-MIT-green.svg) ![Status](https://img.shields.io/badge/Test-Passing-success.svg)

## 1. Project Overview
본 프로젝트는 **ISO 26262 기능안전 표준**을 준수하는 전기차(EV)용 토크 벡터링 제어기 설계 및 검증 프로젝트입니다.
Model-Based Design (MBD) 방법론을 적용하여 요구사항 도출부터 모델링, 자동화된 단위 테스트까지 수행하였습니다.

### 🎯 Key Objectives
* **Safety First:** HARA를 통한 위험 분석 및 ASIL 목표 설정.
* **Traceability:** 요구사항(SRS) ↔ 설계(Design) ↔ 검증(Test) 간 양방향 추적성 확보.
* **Automated Verification:** MATLAB Unit Test Framework를 활용한 CI/CD 기반 자동 검증 환경 구축.

---

## 2. System Architecture
### V-Model Process
본 프로젝트는 자동차 소프트웨어 개발 표준인 V-Model을 따릅니다.
1. **Design Phase:** [HARA](docs/HARA.md) → [SRS](docs/SRS.md) → [Traceability Matrix](docs/Traceability_Matrix.md)
2. **Implementation Phase:** Simulink Model (`models/`)
3. **Verification Phase:** Automated Unit Test (`tests/`)

### Core Features (Safety Mechanism)
| Feature | Requirement ID | Implementation | ASIL |
|:---:|:---:|:---:|:---:|
| **Output Saturation** | `SFR-001` | `Safety_Limiter.slx` | **B** |
| **Max Torque Check** | `SFR-001` | Output Torque ≤ 800Nm | **B** |
| **Fail-Safe Logic** | `SFR-003` | Input Range Check | **A** |

---

## 3. Verification & Results
MATLAB 스크립트를 통해 자동으로 테스트를 수행하고 리포트를 생성합니다.

### ✅ Automated Test Report
* **Test Tool:** `matlab.unittest.TestRunner`
* **Log File:** [test_log.txt](results/test_log.txt)
* **Full Report:** [TestReport.pdf](results/TestReport.pdf) (Download to view)

### Traceability Status
- [x] **[SFR-001]** Output Limit (800Nm) -> **Verified** by `tests/t_SafetyLimiter.m`
- [ ] **[FR-002]** LQR Logic -> *Planned*

---

## 4. How to Run
```matlab
% 1. Clone this repository
git clone [https://github.com/YourID/evMPC1.git](https://github.com/YourID/evMPC1.git)

% 2. Open MATLAB and run the test script
run('scripts/run_all_tests.m')

% 3. Check results in 'results/' folder