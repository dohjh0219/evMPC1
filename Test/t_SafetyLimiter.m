classdef t_SafetyLimiter < matlab.unittest.TestCase
    % [Test Specification]
    % Target: Safety_Limiter.slx
    % Requirement: SFR-001 (Max Torque Limitation)
    
    properties
        ModelName = 'Safety_Limiter';
    end
    
    methods (TestClassSetup)
        function loadParameters(testCase)
            % 1. 파라미터 로드 (load_params.m 실행)
            % 경로를 맞춰주기 위해 상위 폴더의 스크립트 실행
            run('../scripts/load_params.m'); 
            
            % 2. 모델 로드
            load_system(testCase.ModelName);
        end
    end
    
    methods (TestClassTeardown)
        function closeSystem(testCase)
            close_system(testCase.ModelName, 0);
        end
    end
    
    methods (Test)
        function verifyUpperSaturation(testCase)
            % [Scenario 1] 과도한 양의 토크 입력 (1000Nm)
            % Expectation: 800Nm로 제한되어야 함 (ASIL B)
            
            % Arrange (입력 설정)
            in = Simulink.SimulationInput(testCase.ModelName);
            % *주의: 모델의 Inport 이름을 확인하세요 (여기선 'Torque_Demand'로 가정)
            in = in.setExternalInput([0, 1000; 10, 1000]); 
            
            % Act (시뮬레이션 실행)
            out = sim(in);
            
            % Assert (결과 검증)
            % 모델의 Outport 이름이 'Safe_Torque'라고 가정하고 마지막 값 확인
            actual_torque = out.yout{1}.Values.Data(end);
            expected_torque = 800;
            
            testCase.verifyEqual(actual_torque, expected_torque, ...
                '실패: 출력이 800Nm에서 제한되지 않았습니다.');
        end
        
        function verifyNormalOperation(testCase)
            % [Scenario 2] 정상 범위 토크 입력 (500Nm)
            % Expectation: 입력 그대로 500Nm가 출력되어야 함
            
            in = Simulink.SimulationInput(testCase.ModelName);
            in = in.setExternalInput([0, 500; 10, 500]);
            
            out = sim(in);
            
            actual_torque = out.yout{1}.Values.Data(end);
            expected_torque = 500;
            
            testCase.verifyEqual(actual_torque, expected_torque, ...
                '실패: 정상 범위 입력이 왜곡되었습니다.');
        end
    end
end