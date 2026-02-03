# Software Requirements Specification (SRS)

## 1. Project Information
| Item | Details |
|------|---------|
| **Project Name** | EV Torque Vectoring Control System (evMPC) |
| **Version** | v1.0 |
| **Author** | Junhyeong Doh |
| **Last Updated** | 2025-09-12 |
| **Target Hardware** | Teensy 4.1 (ARM Cortex-M7) |

## 2. Overview
본 문서는 전기차(EV)의 주행 안정성 향상을 위한 토크 벡터링 제어기 소프트웨어의 요구사항을 정의한다. 본 시스템은 Simulink로 제작된 정밀 차량 모델과 연동하는 HIL(Hardware-in-the-Loop) 환경에서 동작하며, ISO 26262 기능안전 표준을 준수하여 설계되었다.

## 3. Requirements

### 3.1. Functional Requirements (FR)
| ID | Requirement Description | Verification Method |
|----|------------------------|---------------------|
| **FR-001** | **요 모멘트 계산**<br>제어기는 차량의 현재 상태(종방향 속도, 요레이트, 조향각)를 입력받아, 목표 요레이트를 추종하기 위한 추가 요 모멘트($M_z$)를 계산해야 한다. | Unit Test (TC-FR-01) |
| **FR-002** | **LQR 게인 스케줄링**<br>제어기는 사전에 정의된 `Velocity-Gain Lookup Table`을 참조하여, 현재 차속에 맞는 최적의 LQR 게인 $K$를 10ms 주기로 갱신해야 한다. | Model Simulation |
| **FR-003** | **LPV-MPC 제어**<br>제어기는 LPV 모델을 기반으로 예측 구간(Prediction Horizon) 동안의 거동을 예측하고, 제약 조건을 만족하는 최적 제어 입력을 산출해야 한다. | Model Simulation |
| **FR-004** | **제어기 상태 관리**<br>제어기는 `Standby`, `Active`, `Fault` 상태를 가지며, 진단 결과에 따라 상태 전이 다이어그램(Stateflow)대로 동작해야 한다. | Stateflow Test |

### 3.2. Performance Requirements (PERF)
| ID | Requirement Description | Success Criteria |
|----|------------------------|------------------|
| **PERF-001** | **실행 시간 (Real-time constraints)**<br>전체 제어 알고리즘의 연산 시간(Turn-around time)은 제어 주기인 **10ms**를 초과해서는 안 된다. | Profiling Report |
| **PERF-002** | **추종 정확도**<br>ISO 3888-1 (Double Lane Change) 시나리오(80kph)에서, 목표 요레이트 대비 RMS 오차율은 **5% 이내**여야 한다. | HIL Simulation |
| **PERF-003** | **동작 범위**<br>제어기는 차량 속도 **20 ~ 120