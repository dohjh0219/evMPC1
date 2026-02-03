# Requirements Traceability Matrix (RTM)

본 문서는 ISO 26262 준수를 위해 **안전 목표(HARA)**, **소프트웨어 요구사항(SRS)**, **설계(Design)**, **검증(Test)** 간의 양방향 추적성을 관리한다.

## 1. Safety Goal to Software Requirement (SG -> SRS)
| Safety Goal ID | Safety Goal Description | Linked SRS ID | Requirement Summary |
|:--------------:|:------------------------|:-------------:|:--------------------|
| **SG-01** | 비의도적 요 모멘트 방지 (ASIL D) | **SFR-001** | 출력 토크 800Nm 제한 (Saturation) |
| **SG-01** | 비의도적 요 모멘트 방지 (ASIL D) | **SFR-003** | 센서 입력 유효범위 검사 |
| **SG-02** | 제어 루프 무결성 보장 (ASIL B) | **SFR-002** | 10ms 실행 시간 감시 (Watchdog) |
| **SG-02** | 제어 루프 무결성 보장 (ASIL B) | **IFR-003** | 통신 패킷 Checksum 검사 |

## 2. Requirement to Verification (SRS -> Test)
*현재 단계: 테스트 계획 수립 중 (TBD)*

| SRS ID | Requirement Description | Implementation (Block/File) | Test Case ID | Status |
|:------:|:-----------------------|:---------------------------:|:------------:|:------:|
| **SFR-001** | 출력 토크 800Nm 제한 | `Safety_Limiter.slx` (예정) | TC-SFR-001 | 🚧 In Progress |
| **FR-001** | 요 모멘트 계산 | `Yaw_Controller.slx` (예정) | TC-FR-001 | ⬜ Open |
| **FR-002** | LQR 게인 스케줄링 | `LQR_Gain_Map.m` (예정) | TC-FR-002 | ⬜ Open |

---
**Status Legend:**
* ⬜ Open: 구현 전
* 🚧 In Progress: 구현 중
* ✅ Verified: 테스트 통과