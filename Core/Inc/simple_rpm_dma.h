#ifndef SIMPLE_RPM_DMA_H
#define SIMPLE_RPM_DMA_H

#include <stdint.h>

void RPM_DMA_Init(void);
void RPM_DMA_UpdateValue(int32_t rpm);
void RPM_UART_StartReceive(void);
int32_t RPM_GetTargetValue(void);

#endif 