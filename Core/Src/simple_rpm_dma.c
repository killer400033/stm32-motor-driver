#include "simple_rpm_dma.h"
#include "main.h"

extern UART_HandleTypeDef huart2;

static volatile int32_t rpm_value = 0;
static volatile int32_t rpm_target = 0;
static uint8_t rx_buffer[4];

void RPM_DMA_Init(void)
{
    // Start continuous DMA transmission of the rpm_value
    HAL_UART_Transmit_DMA(&huart2, (uint8_t*)&rpm_value, sizeof(int32_t));
    
    // Start receiving RPM commands
    RPM_UART_StartReceive();
}

void RPM_DMA_UpdateValue(int32_t rpm)
{
    rpm_value = rpm;
}

void RPM_UART_StartReceive(void)
{
    HAL_UART_Receive_IT(&huart2, rx_buffer, sizeof(int32_t));
}

int32_t RPM_GetTargetValue(void)
{
    return rpm_target;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        // Restart transmission immediately
        HAL_UART_Transmit_DMA(&huart2, (uint8_t*)&rpm_value, sizeof(int32_t));
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        // Copy received 4 bytes to rpm_target
        rpm_target = *(int32_t*)rx_buffer;
        
        // Start receiving next command
        RPM_UART_StartReceive();
    }
}