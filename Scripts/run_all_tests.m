% -----------------------------------------------------------
% Script: run_all_tests.m
% Description: 전체 테스트 자동 실행 및 결과 리포트 생성 (수정버전)
% Output: ../results/TestReport.pdf, ../results/test_log.txt
% -----------------------------------------------------------

clear; clc;

% 1. 프로젝트 경로 설정
currentDir = fileparts(mfilename('fullpath'));
projectDir = fileparts(currentDir);
addpath(genpath(projectDir));

% 결과 저장 폴더 생성
resultDir = fullfile(projectDir, 'Results');
if ~exist(resultDir, 'dir')
    mkdir(resultDir);
end

% 2. 텍스트 로그 저장 시작 (diary 기능 사용)
logFile = fullfile(resultDir, 'test_log.txt');
if exist(logFile, 'file')
    delete(logFile);
end
diary(logFile); % 지금부터 명령창의 모든 내용이 파일로 저장됩니다.

% 3. 테스트 슈트 로드
import matlab.unittest.TestSuite
import matlab.unittest.TestRunner
import matlab.unittest.plugins.TestReportPlugin

testFolder = fullfile(projectDir, 'tests');
suite = TestSuite.fromFolder(testFolder);

% 4. 테스트 러너 설정
runner = TestRunner.withTextOutput;

% [핵심] PDF 리포트 생성 플러그인 추가
pdfFile = fullfile(resultDir, 'TestReport.pdf');
% 혹시 PDF 생성에서 에러가 나면 이 부분(try-catch)이 잡아줍니다.
try
    pluginPDF = TestReportPlugin.producingPDF(pdfFile, ...
        'IncludingCommandWindowText', true, ...
        'IncludingPassingDiagnostics', true);
    runner.addPlugin(pluginPDF);
catch ME
    disp('⚠️ 경고: 현재 MATLAB 버전에서 PDF 리포트 생성을 지원하지 않을 수 있습니다.');
    disp(['에러 메시지: ' ME.message]);
end

% 5. 테스트 실행
disp('---------------------------------------------------');
disp('   🚀 [System] 자동화 테스트를 시작합니다...');
disp(['   📂 결과 저장 경로: ' resultDir]);
disp('---------------------------------------------------');

results = runner.run(suite);

% 6. 결과 요약 및 로그 종료
disp(' ');
disp('---------------------------------------------------');
if all([results.Passed])
    disp('   ✅ [Success] 모든 테스트를 통과했습니다!');
else
    disp('   ❌ [Fail] 일부 테스트가 실패했습니다.');
end
disp('---------------------------------------------------');

diary off; % 로그 저장 종료