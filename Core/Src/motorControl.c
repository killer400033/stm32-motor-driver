#include "motorControl.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "main.h"
#include "constants.h"

extern void fixedToFloat(uint32_t *input, float *output);

typedef struct {
	float error;
	float integral;
} PIDState;

typedef struct {
	PIDState pid_state;
	uint64_t prev_time;
	int32_t prev_enc_cnt;
} RPMState;

typedef struct {
	PIDState q_pid;
	PIDState d_pid;
	uint64_t prev_time;
} FOCState;

volatile uint32_t foc_pid_loop_overrun = 0;
volatile uint32_t rpm_pid_loop_overrun = 0;
volatile uint32_t enc_overrun_cnt = 0;
volatile uint32_t millis = 0;
volatile TrigState adc_state = {0};
volatile enum AdcReadMap curr_adc_read_map = 0;

// Function Declarations
void calculateSVM(float _vq, float _vd, uint32_t enc);
void doFOCPIDLoop(float set_iq, float set_id, float *_vq, float *_vd, FOCState *foc_state);
void doRPMPIDLoop(float  set_speed, float *_iq, RPMState *rpm_state);
float calculatePID(PIDState *prev_state, float curr_error, float time, float _P, float _I);
void encoderCalib(void);

static inline uint64_t getMicroSeconds() {
	return (uint64_t)millis * 1000 + (uint64_t)LL_TIM_GetCounter(TIM7);
}

void mainLoop(void) {
	float _vq = 0;
	float _vd = 0;
	float _iq = 0;

	RPMState rpm_state = {0};
	FOCState foc_state = {0};
	// Calibrate Encoder Position
	encoderCalib();

	// Start Timers
	LL_TIM_EnableCounter(TIM7);
	LL_TIM_EnableCounter(TIM16);
	LL_TIM_EnableCounter(TIM17);

	// Main Running Loop
	while (1) {
		// Run Speed PID Loop
		if (rpm_pid_loop_overrun) {
			doRPMPIDLoop(100, &_vq, &rpm_state);
			rpm_pid_loop_overrun--;
		}


		// Run FOC PID Loop
		if (foc_pid_loop_overrun) {
			//doFOCPIDLoop(_iq, 0, &_vq, &_vd, &foc_state);
			foc_pid_loop_overrun--;
		}

		// Updating Motor Output
		calculateSVM(_vq, _vd, LL_TIM_GetCounter(TIM2));
	}
}

void encoderCalib(void) {
	calculateSVM(0, CALIB_PWR, 0);
	LL_mDelay(500);
	TIM2->CNT = 0;
}

void doRPMPIDLoop(float set_speed, float *_iq, RPMState *rpm_state) {
	uint64_t curr_time = getMicroSeconds();
	int32_t curr_enc = LL_TIM_GetCounter(TIM2);
	uint32_t curr_enc_overrun = enc_overrun_cnt;

	float time = (float)(curr_time - rpm_state->prev_time) / 1000000.0;
	float revs = ((float)curr_enc_overrun + ((float)(curr_enc - rpm_state->prev_enc_cnt) / (float)ENCODER_RES));
	float speed = revs / time;

    *_iq = calculatePID(&(rpm_state->pid_state), (set_speed - speed), time, 0.05, 0.5);

	rpm_state->prev_enc_cnt = curr_enc;
	rpm_state->prev_time = curr_time;
	enc_overrun_cnt -= curr_enc_overrun;
}

void doFOCPIDLoop(float set_iq, float set_id, float *_vq, float *_vd, FOCState *foc_state) {
	float _ia;
	float _ib;
	float _i1;
	float _i2;
	float _i3;
	float _iq;
	float _id;

	uint64_t curr_time = getMicroSeconds();
	float time = (float)(curr_time - foc_state->prev_time) / 1000000.0;

	uint32_t raw_output[2];
	float trig_output[2];

	// CORDIC code
#ifdef REV_ENC_DIR
	LL_CORDIC_WriteData(CORDIC, ((uint32_t)(((float)((ENCODER_RES - adc_state.encoder) % MAGNETIC_AGL_ENCODER_CNT)) * ENCODER_TO_ANGLE)) | 0x7FFF0000);
#else
	LL_CORDIC_WriteData(CORDIC, ((uint32_t)(((float)(adc_state.encoder % MAGNETIC_AGL_ENCODER_CNT)) * ENCODER_TO_ANGLE)) | 0x7FFF0000);
#endif

	_i1 = (float)(adc_state.adc_out[0] - ADC_BITS / 2.0) * ADC_SCALING;
	_i2 = (float)(adc_state.adc_out[1] - ADC_BITS / 2.0) * ADC_SCALING;
	_i3 = (float)(adc_state.adc_out[2] - ADC_BITS / 2.0) * ADC_SCALING;

	// Clarke Transform
	if (adc_state.adc_read_map == ignore_i1) {
		_ia = -_i2 -_i3;
		_ib = ONEONSQRT3*_i2 - ONEONSQRT3*_i3;
	}
	else if (adc_state.adc_read_map == ignore_i2) {
		_ia = _i1;
		_ib = -ONEONSQRT3*_i1 - 2*ONEONSQRT3*_i3;
	}
	else {
		_ia = _i1;
		_ib = ONEONSQRT3*_i1 + 2*ONEONSQRT3*_i2;
	}

	raw_output[0] = LL_CORDIC_ReadData(CORDIC) & 0xFFFF;
	raw_output[1] = LL_CORDIC_ReadData(CORDIC) >> 16;
	fixedToFloat(raw_output, trig_output);

	// Park Transform (d axis aligns with a axis)
	_id = trig_output[1] * _ia + trig_output[0] * _ib;
	_iq = -trig_output[0] * _ia + trig_output[1] * _ib;

	*_vq = calculatePID(&(foc_state->q_pid), (set_iq - _iq), time, 0.5, 0);
	*_vd = calculatePID(&(foc_state->d_pid), (set_id - _id), time, 0.5, 0);

	foc_state->prev_time = curr_time;
}

float calculatePID(PIDState *prev_state, float curr_error, float time, float _P, float _I) {
	float proportional = _P * curr_error;
	float integral = prev_state->integral + _I*time*0.5*(curr_error + prev_state->error);
	float output = proportional + integral;

	prev_state->error = curr_error;

	bool is_saturated = false;
	bool is_winding = false;
	if (output < 0) {
		output = 0;
		is_saturated = true;
		is_winding = curr_error < 0;
	}
	else if (output > 1) {
		output = 1;
		is_saturated = true;
		is_winding = curr_error > 0;
	}

    if (!(is_saturated && is_winding)) {
    	prev_state->integral = integral;
    }

	return output;
}

void calculateSVM(float _vq, float _vd, uint32_t enc) {
	float _va;
	float _vb;
	float _v1;
	float _v2;
	float _v3;
	float _max;
	float _min;
	float _mid;
	float trig_output[2];
	uint32_t raw_output[2];

	_vq *= 0.7;

	// CORDIC code
#ifdef REV_ENC_DIR
	LL_CORDIC_WriteData(CORDIC, ((uint32_t)(((float)((ENCODER_RES - enc) % MAGNETIC_AGL_ENCODER_CNT)) * ENCODER_TO_ANGLE)) | 0x7FFF0000);
#else
	LL_CORDIC_WriteData(CORDIC, ((uint32_t)(((float)(enc % MAGNETIC_AGL_ENCODER_CNT)) * ENCODER_TO_ANGLE)) | 0x7FFF0000);
#endif
	// This stalls the CPU memory bus until the cordic calculation is complete, so if for some reason we need fast interrupts later, this must be changed to polling
	raw_output[0] = LL_CORDIC_ReadData(CORDIC) & 0xFFFF;
	raw_output[1] = LL_CORDIC_ReadData(CORDIC) >> 16;
	fixedToFloat(raw_output, trig_output);

	// Example Timer code

	// Inverse Park Transform (d axis aligns with a axis)
	_va = trig_output[1] * _vd - trig_output[0] * _vq;
	_vb = trig_output[0] * _vd + trig_output[1] * _vq;

	// Inverse Clarke Transform
	_v1 = _va * ONEONSQRT3; // Multiplying by 1/sqrt(3) scales the vectors such that when |(va, vb)| = 1, the SVM is saturated
	_v2 = (-_va*0.5 + _vb*SQRT3ON2) * ONEONSQRT3;
	_v3 = (-_va*0.5 - _vb*SQRT3ON2) * ONEONSQRT3;

	// Space Vector Modulation
	// Finds the min and max value in v1, v2, v3
	if (_v1 > _v2) {
		_min = _v2;
		_max = _v1;
		curr_adc_read_map = ignore_i1;
	}
	else {
		_min = _v1;
		_max = _v2;
		curr_adc_read_map = ignore_i2;
	}

	if (_v3 > _max) {
		_max = _v3;
		curr_adc_read_map = ignore_i3;
	}
	else if (_v3 < _min) {
		_min = _v3;
	}

	// Converts the sinusoidal waveform into SVM between 0 and 1
	_mid = (_min + _max) / 2.0;
	_v1 += 0.5 - _mid;
	_v2 += 0.5 - _mid;
	_v3 += 0.5 - _mid;

	// Clamp v1, v2, v3 between 0 to 1. This automatically applies Overmodulation into a trapezoidal waveform
	_v1 = (_v1 < 0) ? 0 : ((_v1 > 1) ? 1 : _v1);
	_v2 = (_v2 < 0) ? 0 : ((_v2 > 1) ? 1 : _v2);
	_v3 = (_v3 < 0) ? 0 : ((_v3 > 1) ? 1 : _v3);

	LL_TIM_OC_SetCompareCH1(TIM1, (float)PWM_PERIOD*_v1);
	LL_TIM_OC_SetCompareCH2(TIM1, (float)PWM_PERIOD*_v2);
	LL_TIM_OC_SetCompareCH3(TIM1, (float)PWM_PERIOD*_v3);
}
