#define S_FUNCTION_NAME  nlplant_sfun
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "nlplant_f.h"  // CasADi에서 생성한 헤더 파일 포함

static void mdlInitializeSizes(SimStruct *S) {
    int_T n_in  = nlplant_f_n_in();
    int_T n_out = nlplant_f_n_out();

    ssSetNumInputPorts(S, n_in);
    ssSetNumOutputPorts(S, n_out);

    for (int i = 0; i < n_in; ++i) {
        const int_T* sp = nlplant_f_sparsity_in(i);
        ssSetInputPortWidth(S, i, sp[0]);
        ssSetInputPortDirectFeedThrough(S, i, 1);
    }

    for (int i = 0; i < n_out; ++i) {
        const int_T* sp = nlplant_f_sparsity_out(i);
        ssSetOutputPortWidth(S, i, sp[0]);
    }

    int_T sz_arg, sz_res, sz_iw, sz_w;
    nlplant_f_work(&sz_arg, &sz_res, &sz_iw, &sz_w);

    ssSetNumRWork(S, sz_w);
    ssSetNumIWork(S, sz_iw);
    ssSetNumPWork(S, sz_arg + sz_res);
}

static void mdlOutputs(SimStruct *S, int_T tid) {
    int_T sz_arg, sz_res, sz_iw, sz_w;
    nlplant_f_work(&sz_arg, &sz_res, &sz_iw, &sz_w);

    void** p = ssGetPWork(S);
    const real_T** arg = (const real_T**)p;
    real_T** res = (real_T**)(p + sz_arg);
    real_T* w = ssGetRWork(S);
    int_T* iw = ssGetIWork(S);

    for (int i = 0; i < nlplant_f_n_in(); ++i) {
        arg[i] = *ssGetInputPortRealSignalPtrs(S, i);
    }
    for (int i = 0; i < nlplant_f_n_out(); ++i) {
        res[i] = ssGetOutputPortRealSignal(S, i);
    }

    nlplant_f(arg, res, iw, w, 0);
}

static void mdlInitializeSampleTimes(SimStruct *S) {
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

static void mdlTerminate(SimStruct *S) {}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif