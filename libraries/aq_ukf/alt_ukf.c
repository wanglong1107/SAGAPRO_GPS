#include "alt_ukf.h"
#include "nav_ukf.h"
//#include "imu.h"
#include <string.h>

altUkfStruct_t  altUkfData;


void altUkfTimeUpdate(float *in, float *noise, float *out, float *u, float dt, int n) {
    float acc;
    int i;

    // assume out == in
    out = in;

    for (i = 0; i < n; i++) {
        acc = u[0] + in[ALT_STATE_BIAS*n + i];

        out[ALT_STATE_BIAS*n + i] = in[ALT_STATE_BIAS*n + i] + (noise[ALT_NOISE_BIAS*n + i] * dt);
        out[ALT_STATE_VEL*n + i] = in[ALT_STATE_VEL*n + i] + (acc * dt) + (noise[ALT_NOISE_VEL*n + i] * dt);
        out[ALT_STATE_POS*n + i] = in[ALT_STATE_POS*n + i] - (in[ALT_STATE_VEL*n + i] * dt) - (acc * dt * dt * 0.5f);
    }
}

void altUkfPresUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = x[ALT_STATE_POS] + noise[0];     // return altitude
}

static void altDoPresUpdate(float measuredPres) {
    float noise;        // measurement variance
    float y;            // measurment

    noise = ALT_PRES_NOISE;
    y = navUkfPresToAlt(measuredPres);
    srcdkfMeasurementUpdate(altUkfData.kf, 0, &y, 1, 1, &noise, altUkfPresUpdate);
}

void altUkfProcess(float measuredPres) {
    float accIn[3];
    float acc[3];

    accIn[0] = ap_accel.x + UKF_ACC_BIAS_X;
    accIn[1] = ap_accel.y + UKF_ACC_BIAS_Y;
    accIn[2] = ap_accel.z + UKF_ACC_BIAS_Z;

    // rotate acc to world frame
    navUkfRotateVectorByQuat(acc, accIn, &UKF_Q1);
    acc[2] += GRAVITY;

    srcdkfTimeUpdate(altUkfData.kf, &acc[2], ((float32_t)navUkfData.ap_outerTimestep)*1.0E-6);

    altDoPresUpdate(measuredPres);
}

void altUkfInit(void) {
    float Q[ALT_S];		// state variance
    float V[ALT_V];		// process variance

    memset((void *)&altUkfData, 0, sizeof(altUkfData));

    altUkfData.kf = srcdkfInit(ALT_S, ALT_M, ALT_V, ALT_N, altUkfTimeUpdate);

    altUkfData.x = srcdkfGetState(altUkfData.kf);

    Q[ALT_STATE_POS] = 5.0f;
    Q[ALT_STATE_VEL] = 1e-6f;
    Q[ALT_STATE_BIAS] = 0.05f;

    V[ALT_NOISE_BIAS] = ALT_BIAS_NOISE;
    V[ALT_NOISE_VEL] = ALT_VEL_NOISE;

    srcdkfSetVariance(altUkfData.kf, Q, V, 0, 0);

    ALT_POS = navUkfPresToAlt(ap_baro.pressure);
    ALT_VEL = 0.0f;
    ALT_BIAS = 0.0f;
}
