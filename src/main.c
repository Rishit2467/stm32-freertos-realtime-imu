#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ============================================================================
 * 1. SYSTEM CONSTANTS & LOGIC ANALYSER BSRR MACROS
 * ============================================================================ */
#define DT_SEC              0.01f   // 10 ms sample period (100 Hz)
#define COMP_ALPHA          0.98f   // Complementary filter weight
#define MOVING_AVG_WINDOW   5       // 5-sample moving average in Task 2
#define TILT_THRESHOLD_DEG  30.0f   // Alarm threshold

// Direct Register BSRR macros (~15 ns toggle time for Logic Analyser)
#define LA_T1_HIGH()        (GPIOC->BSRR = GPIO_PIN_0)
#define LA_T1_LOW()         (GPIOC->BSRR = (uint32_t)GPIO_PIN_0 << 16U)
#define LA_T2_HIGH()        (GPIOC->BSRR = GPIO_PIN_1)
#define LA_T2_LOW()         (GPIOC->BSRR = (uint32_t)GPIO_PIN_1 << 16U)
#define LA_T3_HIGH()        (GPIOC->BSRR = GPIO_PIN_2)
#define LA_T3_LOW()         (GPIOC->BSRR = (uint32_t)GPIO_PIN_2 << 16U)

/* ============================================================================
 * 2. DATA STRUCTURES & RTOS HANDLES
 * ============================================================================ */
typedef struct {
    float pitch;
    float roll;
    uint32_t timestamp_ms;
} IMU_AngleMsg_t;

typedef struct {
    float filtered_pitch;
    float filtered_roll;
    uint8_t tilt_alarm;
    uint32_t sample_count;
} SharedSystemState_t;

UART_HandleTypeDef huart2;
IWDG_HandleTypeDef hiwdg;

TaskHandle_t xTask1Handle = NULL;
TaskHandle_t xTask2Handle = NULL;
TaskHandle_t xTask3Handle = NULL;

QueueHandle_t xAngleQueue = NULL;
SemaphoreHandle_t xStateMutex = NULL;

SharedSystemState_t g_SystemState = {0};
volatile uint8_t g_Task1Alive = 0;
volatile uint8_t g_Task2Alive = 0;

/* External FreeRTOS Tick Handler called from TIM3 ISR */
extern void xPortSysTickHandler(void);

/* ============================================================================
 * 3. SYNTHETIC IMU GENERATOR (Replaced by Bare-Metal I2C1 in Phase 2)
 * ============================================================================ */
static void Read_Synthetic_IMU(float *ax, float *ay, float *az, float *gx, float *gy) {
    static uint32_t tick_count = 0;
    float t = tick_count * DT_SEC;
    tick_count++;

    // Smooth 0.5 Hz pitch oscillation (+/- 38 deg) to trigger >30 deg threshold
    float true_pitch_rad = (38.0f * M_PI / 180.0f) * sinf(2.0f * M_PI * 0.5f * t);
    float true_roll_rad  = (15.0f * M_PI / 180.0f) * cosf(2.0f * M_PI * 0.25f * t);

    float noise = ((int)(tick_count % 7) - 3) * 0.01f;
    *ax = sinf(true_pitch_rad) + noise;
    *ay = sinf(true_roll_rad)  - noise;
    *az = cosf(true_pitch_rad) * cosf(true_roll_rad);

    *gx = 38.0f * 2.0f * M_PI * 0.5f * cosf(2.0f * M_PI * 0.5f * t);
    *gy = -15.0f * 2.0f * M_PI * 0.25f * sinf(2.0f * M_PI * 0.25f * t);
}

/* ============================================================================
 * 4. FREERTOS TASKS
 * ============================================================================ */

/* TASK 1: IMU Acquisition & Complementary Filter (10 ms period, Priority 3) */
void vTask1_IMUAcquisition(void *pvParameters) {
    (void)pvParameters;
    float ax, ay, az, gx, gy;
    float pitch = 0.0f, roll = 0.0f;
    IMU_AngleMsg_t msg;

    for (;;) {
        // Sleep until TIM3 ISR fires notification every 10.0 ms
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        LA_T1_HIGH(); // PC0 HIGH

        Read_Synthetic_IMU(&ax, &ay, &az, &gx, &gy);

        float accel_pitch = atan2f(ax, sqrtf(ay * ay + az * az)) * (180.0f / M_PI);
        float accel_roll  = atan2f(ay, sqrtf(ax * ax + az * az)) * (180.0f / M_PI);

        pitch = COMP_ALPHA * (pitch + gx * DT_SEC) + (1.0f - COMP_ALPHA) * accel_pitch;
        roll  = COMP_ALPHA * (roll  + gy * DT_SEC) + (1.0f - COMP_ALPHA) * accel_roll;

        msg.pitch = pitch;
        msg.roll  = roll;
        msg.timestamp_ms = xTaskGetTickCount();
        xQueueSend(xAngleQueue, &msg, 0);

        g_Task1Alive = 1;

        LA_T1_LOW();  // PC0 LOW
    }
}

/* TASK 2: Data Processing & Threshold Detection (50 ms period, Priority 2) */
void vTask2_DataProcessing(void *pvParameters) {
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(50);

    IMU_AngleMsg_t rx_msg;
    float pitch_buf[MOVING_AVG_WINDOW] = {0};
    float roll_buf[MOVING_AVG_WINDOW]  = {0};
    uint8_t buf_idx = 0;

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        LA_T2_HIGH(); // PC1 HIGH

        while (xQueueReceive(xAngleQueue, &rx_msg, 0) == pdPASS) {
            pitch_buf[buf_idx] = rx_msg.pitch;
            roll_buf[buf_idx]  = rx_msg.roll;
            buf_idx = (buf_idx + 1) % MOVING_AVG_WINDOW;
        }

        float sum_p = 0.0f, sum_r = 0.0f;
        for (int i = 0; i < MOVING_AVG_WINDOW; i++) {
            sum_p += pitch_buf[i];
            sum_r += roll_buf[i];
        }
        float avg_pitch = sum_p / (float)MOVING_AVG_WINDOW;
        float avg_roll  = sum_r / (float)MOVING_AVG_WINDOW;

        uint8_t alarm = (fabsf(avg_pitch) > TILT_THRESHOLD_DEG ||
                         fabsf(avg_roll)  > TILT_THRESHOLD_DEG) ? 1 : 0;

        // Nucleo Green LED (PA5) lights up on Tilt Alarm (>30 deg)
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, alarm ? GPIO_PIN_SET : GPIO_PIN_RESET);

        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            g_SystemState.filtered_pitch = avg_pitch;
            g_SystemState.filtered_roll  = avg_roll;
            g_SystemState.tilt_alarm     = alarm;
            g_SystemState.sample_count++;
            xSemaphoreGive(xStateMutex);
        }

        g_Task2Alive = 1;

        LA_T2_LOW();  // PC1 LOW
    }
}

/* TASK 3: JSON Telemetry TX & Watchdog Supervision (500 ms period, Priority 1) */
void vTask3_TelemetryTX(void *pvParameters) {
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(500);

    SharedSystemState_t local_copy = {0};
    char json_buf[128];

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        LA_T3_HIGH(); // PC2 HIGH

        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            local_copy = g_SystemState;
            xSemaphoreGive(xStateMutex);
        }

        int len = snprintf(json_buf, sizeof(json_buf),
            "{\"ts\":%lu,\"pitch\":%.2f,\"roll\":%.2f,\"alarm\":%u,\"seq\":%lu}\r\n",
            (unsigned long)xTaskGetTickCount(),
            local_copy.filtered_pitch,
            local_copy.filtered_roll,
            local_copy.tilt_alarm,
            (unsigned long)local_copy.sample_count);

        HAL_UART_Transmit(&huart2, (uint8_t *)json_buf, len, 25);

        if (g_Task1Alive && g_Task2Alive) {
            g_Task1Alive = 0;
            g_Task2Alive = 0;
            HAL_IWDG_Refresh(&hiwdg);
        }

        LA_T3_LOW();  // PC2 LOW
    }
}

/* ============================================================================
 * 5. INTERRUPT SERVICE ROUTINES (SysTick & TIM3 Register-Level ISR)
 * ============================================================================ */
void SysTick_Handler(void) {
    HAL_IncTick();
}

/**
 * @brief TIM3 Hardware ISR firing at 1000 Hz (1.0 ms).
 *        1. Drives FreeRTOS 1 ms kernel tick.
 *        2. Every 10 ticks (10.0 ms), fires direct Task Notification to wake Task 1.
 */
void TIM3_IRQHandler(void) {
    if (TIM3->SR & TIM_SR_UIF) {
        TIM3->SR &= ~TIM_SR_UIF; // Clear update interrupt flag

        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
            // 1. Advance FreeRTOS Tick
            xPortSysTickHandler();

            // 2. Wake Task 1 every 10 ms (100 Hz) via ISR Notification
            static uint8_t decimator = 0;
            decimator++;
            if (decimator >= 10) {
                decimator = 0;
                BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                if (xTask1Handle != NULL) {
                    vTaskNotifyGiveFromISR(xTask1Handle, &xHigherPriorityTaskWoken);
                    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
                }
            }
        }
    }
}

/* ============================================================================
 * 6. HARDWARE PERIPHERAL INITIALIZATION (64 MHz Clock, GPIO, UART2, TIM3, IWDG)
 * ============================================================================ */
static void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    // HSI (8 MHz) / 2 * 16 = 64 MHz SystemCoreClock (works on every Nucleo-F103RB)
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;   // HCLK = 64 MHz
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;    // APB1 = 32 MHz (TIM3 clk = 64 MHz)
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;    // APB2 = 64 MHz
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

static void GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // PA5: Onboard User Green LED (LD2)
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PC0, PC1, PC2: Logic Analyser Task Profiling Pins
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

static void USART2_Init(void) {
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    // PA2 -> USART2_TX, PA3 -> USART2_RX
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);
}

static void TIM3_BareMetal_Init(void) {
    // Enable TIM3 clock on APB1 (Timer clock = 64 MHz)
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // 64 MHz / 64 = 1 MHz (1 us tick); 1000 ticks = 1.0 ms (1000 Hz)
    TIM3->PSC = 64 - 1;
    TIM3->ARR = 1000 - 1;
    TIM3->DIER |= TIM_DIER_UIE; // Enable update interrupt

    // Priority MUST be >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (5)
    NVIC_SetPriority(TIM3_IRQn, 6);
    NVIC_EnableIRQ(TIM3_IRQn);

    TIM3->CR1 |= TIM_CR1_CEN;   // Start TIM3
}

static void IWDG_Init(void) {
    // LSI is ~40 kHz. Prescaler 32 -> 1250 Hz -> Reload 1500 = ~1.2 sec watchdog timeout
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
    hiwdg.Init.Reload = 1500;
    HAL_IWDG_Init(&hiwdg);
}

/* ============================================================================
 * 7. MAIN ENTRY POINT
 * ============================================================================ */
int main(void) {
    HAL_Init();
    SystemClock_Config();

    // Ensure 4 priority bits (0 subpriority bits) as required by FreeRTOS on Cortex-M3
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    GPIO_Init();
    USART2_Init();
    IWDG_Init();

    xAngleQueue = xQueueCreate(16, sizeof(IMU_AngleMsg_t));
    xStateMutex = xSemaphoreCreateMutex();

    xTaskCreate(vTask1_IMUAcquisition, "IMU_Acq",  256, NULL, 3, &xTask1Handle);
    xTaskCreate(vTask2_DataProcessing, "Data_Proc", 256, NULL, 2, &xTask2Handle);
    xTaskCreate(vTask3_TelemetryTX,    "Telem_TX",  512, NULL, 1, &xTask3Handle);

    TIM3_BareMetal_Init();

    vTaskStartScheduler();

    while (1) {}
}