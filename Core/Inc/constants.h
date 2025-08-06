// Variables
#define POLE_CNT 28
#define ENCODER_RES 2400
#define CLK 170000000
#define PWM_PERIOD 4250
#define ADC_BITS 4096
#define ADC_SCALING 13.2 / ADC_BITS
#define ADC_COMP_SCALING 0.05

// Constants
#define TRUE 1
#define FALSE 0
#define SQRT3ON2 0.86602540378
#define ONEONSQRT3 0.57735026919 // This scales _va and _vb by 1/sqrt(3) such that when |(vq, vd)| = 1, the SVM is saturated
#define MAGNETIC_AGL_ENCODER_CNT (ENCODER_RES * 2 / POLE_CNT)
#define ENCODER_TO_ANGLE (65536.0/(float)MAGNETIC_AGL_ENCODER_CNT)
#define CALIB_PWR 0.1

//#define REV_ENC_DIR
