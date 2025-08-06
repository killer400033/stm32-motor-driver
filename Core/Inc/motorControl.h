#include <stdint.h>

enum AdcReadMap {
	ignore_i1,
	ignore_i2,
	ignore_i3,
};

typedef struct {
	uint32_t encoder;
	int16_t adc_out[3];
	enum AdcReadMap adc_read_map;
} TrigState;

extern void mainLoop(void);
