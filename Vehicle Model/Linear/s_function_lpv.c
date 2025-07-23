#define S_FUNCTION_NAME  s_function_lpv
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "f_lpv.h"  // CasADi가 만든 헤더

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);  // 파라미터 개수 없음
    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, 7);  // x(3) + u(2) + delta(1) + vx0(1) = 총 7개 입력
    ssSetInputPortDirectFeedThrough(S, 0, 1);

    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 3);  // xdot = 3개 출력

    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
}
static void mdlInitializeSampleTimes(SimStruct *S) {
    ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

static void mdlOutputs(SimStruct *S, int_T tid) {
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S, 0);
    real_T *y = ssGetOutputPortRealSignal(S, 0);

    // 입력값 해석
    double x_vec[3] = {*uPtrs[0], *uPtrs[1], *uPtrs[2]};
    double u_vec[2] = {*uPtrs[3], *uPtrs[4]};
    double delta = *uPtrs[5];
    double vx0 = *uPtrs[6];

    // CasADi 입력 포인터 설정
    const casadi_real* arg[4];
    arg[0] = x_vec;
    arg[1] = u_vec;
    arg[2] = &delta;
    arg[3] = &vx0;

    // 출력 버퍼
    casadi_real res0[3];  // xdot
    casadi_real* res[1];
    res[0] = res0;

    // 임시 작업 메모리 (CasADi 요구사항: 크기 미리 알 수 없음 → 0으로 호출)
    casadi_int iw[1];   // 최소 임시 크기
    casadi_real w[1];   // 최소 임시 크기

    // 함수 호출
    f_lpv(arg, res, iw, w, 0);

    // 출력 복사
    y[0] = res0[0];
    y[1] = res0[1];
    y[2] = res0[2];
}

static void mdlTerminate(SimStruct *S) {}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif