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
 * 1. SYSTEM CONSTANTS & GPIO ENTRY/EXIT MACROS
 * ============================================================================ */
#define DT_T1_SEC           0.01f       // Task 1 period: 10 ms
#define DT_T2_SEC           0.05f       // Task 2 period: 50 ms
#define LPF_ALPHA           0.80f       // Digital low-pass filter smoothing factor (0..1)
#define CPU_CLK_HZ          64000000UL  // 64 MHz SYSCLK

// Direct BSRR GPIO entry/exit toggles for task timing markers (PC0, PC1, PC2)
#define GPIO_T1_HIGH()      (GPIOC->BSRR = GPIO_PIN_0)
#define GPIO_T1_LOW()       (GPIOC->BSRR = (uint32_t)GPIO_PIN_0 << 16U)
#define GPIO_T2_HIGH()      (GPIOC->BSRR = GPIO_PIN_1)
#define GPIO_T2_LOW()       (GPIOC->BSRR = (uint32_t)GPIO_PIN_1 << 16U)
#define GPIO_T3_HIGH()      (GPIOC->BSRR = GPIO_PIN_2)
#define GPIO_T3_LOW()       (GPIOC->BSRR = (uint32_t)GPIO_PIN_2 << 16U)

/* ============================================================================
 * 2. DATA STRUCTURES, PID STATE, TIMING METRICS & RTOS HANDLES
 * ============================================================================ */
typedef struct {
    float setpoint;
    float feedback;
    uint32_t tick_ms;
} ControlInput_Msg_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float dt;
    float integral;
    float prev_error;
    float out_min;
    float out_max;
} PID_Controller_t;

typedef struct {
    uint32_t last_entry_cyccnt;
    uint32_t last_period_cycles;
    uint32_t last_exec_cycles;
    uint32_t wcet_cycles;
    uint8_t  has_prev_entry;
} TaskTiming_t;

typedef struct {
    float setpoint;
    float output;
    float error;
    uint32_t seq;
    uint32_t t1_period_us;
    uint32_t t2_period_us;
    uint32_t t3_period_us;
    uint32_t t1_wcet_us;
    uint32_t t2_wcet_us;
    uint32_t t3_wcet_us;
} TelemetryState_t;

UART_HandleTypeDef huart2;
IWDG_HandleTypeDef hiwdg;

TaskHandle_t xTask1Handle = NULL;
TaskHandle_t xTask2Handle = NULL;
TaskHandle_t xTask3Handle = NULL;

QueueHandle_t     xControlQueue = NULL;
SemaphoreHandle_t xStateMutex   = NULL;

TelemetryState_t g_TelemetryState = {0};

TaskTiming_t g_T1_Timing = {0};
TaskTiming_t g_T2_Timing = {0};
TaskTiming_t g_T3_Timing = {0};

volatile uint8_t g_Task1Alive = 0;
volatile uint8_t g_Task2Alive = 0;

extern void xPortSysTickHandler(void);

/* ============================================================================
 * 3. HARDWARE TIMER / DWT CYCLE COUNTER & SIGNAL PROCESSING HELPERS
 * ============================================================================ */
static void DWT_CycleCounter_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline uint32_t Timing_OnEntry(TaskTiming_t *t) {
    uint32_t now = DWT->CYCCNT;
    if (t->has_prev_entry) {
        t->last_period_cycles = now - t->last_entry_cyccnt;
    }
    t->last_entry_cyccnt = now;
    t->has_prev_entry = 1;
    return now;
}

static inline void Timing_OnExit(TaskTiming_t *t, uint32_t entry_cyccnt) {
    uint32_t elapsed = DWT->CYCCNT - entry_cyccnt;
    t->last_exec_cycles = elapsed;
    if (elapsed > t->wcet_cycles) {
        t->wcet_cycles = elapsed;
    }
}

static inline uint32_t Cycles_To_Us(uint32_t cycles) {
    return cycles / (CPU_CLK_HZ / 1000000UL);
}

// Simulates X/Y joystick-like inputs (square/step setpoint + slight noise)
// Replace with ADC1 reads when wiring a physical joystick to PA0/PA1.
static void Read_Simulated_XY_Channels(float *x_setpoint, float *y_feedback_raw) {
    static uint32_t step_counter = 0;
    float t = step_counter * DT_T1_SEC;
    step_counter++;

    // Step setpoint between 25.0 and 75.0 every 4 seconds
    int phase = ((int)(t / 4.0f)) % 2;
    float base_sp = (phase == 0) ? 75.0f : 25.0f;

    // Small deterministic ripple to exercise the digital low-pass filter
    float ripple = 2.0f * sinf(2.0f * (float)M_PI * 3.0f * t);
    *x_setpoint = base_sp + ripple;

    // Secondary channel / disturbance signal
    *y_feedback_raw = 0.5f * sinf(2.0f * (float)M_PI * 1.0f * t);
}

static void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd,
                     float dt, float out_min, float out_max) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = dt;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->out_min = out_min;
    pid->out_max = out_max;
}

static float PID_Compute(PID_Controller_t *pid, float setpoint, float measurement) {
    float error = setpoint - measurement;

    float p_term = pid->kp * error;

    // Integral with output clamping anti-windup
    pid->integral += error * pid->dt;
    float max_i = (pid->ki != 0.0f) ? (pid->out_max / pid->ki) : pid->out_max;
    float min_i = (pid->ki != 0.0f) ? (pid->out_min / pid->ki) : pid->out_min;
    if (pid->integral > max_i) pid->integral = max_i;
    if (pid->integral < min_i) pid->integral = min_i;
    float i_term = pid->ki * pid->integral;

    float derivative = (error - pid->prev_error) / pid->dt;
    float d_term = pid->kd * derivative;
    pid->prev_error = error;

    float out = p_term + i_term + d_term;
    if (out > pid->out_max) out = pid->out_max;
    if (out < pid->out_min) out = pid->out_min;
    return out;
}

/* ============================================================================
 * 4. FREERTOS TASKS
 * ============================================================================ */

/* TASK 1: Input Acquisition & Digital Low-Pass Filter (10 ms via TIM3 ISR, Prio 3) */
void vTask1_InputAndFilter(void *pvParameters) {
    (void)pvParameters;
    float raw_x = 0.0f, raw_y = 0.0f;
    float filt_x = 50.0f, filt_y = 0.0f;
    ControlInput_Msg_t msg;

    for (;;) {
        // Block until TIM3 ISR notifies every 10 ms
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        GPIO_T1_HIGH();
        uint32_t t_start = Timing_OnEntry(&g_T1_Timing);

        Read_Simulated_XY_Channels(&raw_x, &raw_y);

        // First-order exponential low-pass filter
        filt_x = LPF_ALPHA * filt_x + (1.0f - LPF_ALPHA) * raw_x;
        filt_y = LPF_ALPHA * filt_y + (1.0f - LPF_ALPHA) * raw_y;

        msg.setpoint = filt_x;
        msg.feedback = filt_y;
        msg.tick_ms  = xTaskGetTickCount();

        // Overwrite oldest if queue is full so Task 2 always gets fresh data
        if (xQueueSend(xControlQueue, &msg, 0) != pdPASS) {
            ControlInput_Msg_t discard;
            xQueueReceive(xControlQueue, &discard, 0);
            xQueueSend(xControlQueue, &msg, 0);
        }

        g_Task1Alive = 1;

        Timing_OnExit(&g_T1_Timing, t_start);
        GPIO_T1_LOW();
    }
}

/* TASK 2: Closed-Loop PID Controller & Virtual Plant (50 ms, Priority 2) */
void vTask2_PIDController(void *pvParameters) {
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(50);

    PID_Controller_t pid;
    PID_Init(&pid, 1.8f, 2.5f, 0.15f, DT_T2_SEC, 0.0f, 100.0f);

    ControlInput_Msg_t rx_msg;
    float current_sp = 50.0f;
    float plant_output = 50.0f; // Simulated motor/plant state (0..100%)

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        GPIO_T2_HIGH();
        uint32_t t_start = Timing_OnEntry(&g_T2_Timing);

        // Drain queue to get the latest filtered setpoint
        while (xQueueReceive(xControlQueue, &rx_msg, 0) == pdPASS) {
            current_sp = rx_msg.setpoint;
        }

        float error = current_sp - plant_output;
        float control_effort = PID_Compute(&pid, current_sp, plant_output);

        // First-order plant dynamics: dy/dt = (-y + k * u) / tau
        float tau = 0.35f;
        float k_gain = 1.0f;
        plant_output += ((-plant_output + k_gain * control_effort) / tau) * DT_T2_SEC;
        if (plant_output < 0.0f) plant_output = 0.0f;
        if (plant_output > 100.0f) plant_output = 100.0f;

        // Visual indicator on onboard LD2 (PA5) when tracking error exceeds 15 units
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, (fabsf(error) > 15.0f) ? GPIO_PIN_SET : GPIO_PIN_RESET);

        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            g_TelemetryState.setpoint     = current_sp;
            g_TelemetryState.output       = plant_output;
            g_TelemetryState.error        = error;
            g_TelemetryState.seq++;
            g_TelemetryState.t1_period_us = Cycles_To_Us(g_T1_Timing.last_period_cycles);
            g_TelemetryState.t2_period_us = Cycles_To_Us(g_T2_Timing.last_period_cycles);
            g_TelemetryState.t1_wcet_us   = Cycles_To_Us(g_T1_Timing.wcet_cycles);
            g_TelemetryState.t2_wcet_us   = Cycles_To_Us(g_T2_Timing.wcet_cycles);
            xSemaphoreGive(xStateMutex);
        }

        g_Task2Alive = 1;

        Timing_OnExit(&g_T2_Timing, t_start);
        GPIO_T2_LOW();
    }
}

/* TASK 3: UART JSON Telemetry & Watchdog Supervision (500 ms, Priority 1) */
void vTask3_TelemetryUART(void *pvParameters) {
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(500);

    TelemetryState_t snap = {0};
    char uart_buf[256];

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        GPIO_T3_HIGH();
        uint32_t t_start = Timing_OnEntry(&g_T3_Timing);

        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            snap = g_TelemetryState;
            snap.t3_period_us = Cycles_To_Us(g_T3_Timing.last_period_cycles);
            snap.t3_wcet_us   = Cycles_To_Us(g_T3_Timing.wcet_cycles);
            xSemaphoreGive(xStateMutex);
        }

        int len = snprintf(uart_buf, sizeof(uart_buf),
            "{\"ts\":%lu,\"sp\":%.2f,\"out\":%.2f,\"err\":%.2f,"
            "\"period_us\":[%lu,%lu,%lu],\"wcet_us\":[%lu,%lu,%lu],\"seq\":%lu}\r\n",
            (unsigned long)xTaskGetTickCount(),
            snap.setpoint,
            snap.output,
            snap.error,
            (unsigned long)snap.t1_period_us,
            (unsigned long)snap.t2_period_us,
            (unsigned long)snap.t3_period_us,
            (unsigned long)snap.t1_wcet_us,
            (unsigned long)snap.t2_wcet_us,
            (unsigned long)snap.t3_wcet_us,
            (unsigned long)snap.seq);

        HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, (uint16_t)len, 30);

        // Kick IWDG only when higher-priority tasks have executed since last check
        if (g_Task1Alive && g_Task2Alive) {
            g_Task1Alive = 0;
            g_Task2Alive = 0;
            HAL_IWDG_Refresh(&hiwdg);
        }

        Timing_OnExit(&g_T3_Timing, t_start);
        GPIO_T3_LOW();
    }
}

/* ============================================================================
 * 5. INTERRUPT SERVICE ROUTINES (SysTick & TIM3 Register-Level ISR)
 * ============================================================================ */
void SysTick_Handler(void) {
    HAL_IncTick();
}

/**
 * @brief TIM3 ISR at 1000 Hz (1.0 ms):
 *        - Advances FreeRTOS tick
 *        - Every 10 ticks (10.0 ms), wakes Task 1 via direct task notification
 */
void TIM3_IRQHandler(void) {
    if (TIM3->SR & TIM_SR_UIF) {
        TIM3->SR &= ~TIM_SR_UIF;

        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
            xPortSysTickHandler();

            static uint8_t prescale_10ms = 0;
            prescale_10ms++;
            if (prescale_10ms >= 10) {
                prescale_10ms = 0;
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
 * 6. HARDWARE INITIALIZATION (64 MHz Clock, GPIO, USART2, TIM3, IWDG)
 * ============================================================================ */
static void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;   // 64 MHz
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;    // 32 MHz (TIM3 clock = 64 MHz)
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;    // 64 MHz
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

static void GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // PA5: Onboard User LED (LD2)
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PC0, PC1, PC2: Task Entry/Exit GPIO Profiling Pins
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
    GPIO_InitStruct.Pin = GPIO_PIN_2; // USART2_TX
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3; // USART2_RX
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
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // 64 MHz / 64 = 1 MHz tick; ARR = 1000 - 1 => 1000 Hz (1.0 ms interrupt)
    TIM3->PSC = 64 - 1;
    TIM3->ARR = 1000 - 1;
    TIM3->DIER |= TIM_DIER_UIE;

    // Priority >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (5)
    NVIC_SetPriority(TIM3_IRQn, 6);
    NVIC_EnableIRQ(TIM3_IRQn);

    TIM3->CR1 |= TIM_CR1_CEN;
}

static void IWDG_Init(void) {
    // ~1.2 s timeout at ~40 kHz LSI
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
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    GPIO_Init();
    USART2_Init();
    DWT_CycleCounter_Init();
    IWDG_Init();

    xControlQueue = xQueueCreate(16, sizeof(ControlInput_Msg_t));
    xStateMutex   = xSemaphoreCreateMutex();

    xTaskCreate(vTask1_InputAndFilter, "Task1_LPF",  256, NULL, 3, &xTask1Handle);
    xTaskCreate(vTask2_PIDController,  "Task2_PID",  384, NULL, 2, &xTask2Handle);
    xTaskCreate(vTask3_TelemetryUART,  "Task3_UART", 512, NULL, 1, &xTask3Handle);

    TIM3_BareMetal_Init();
    vTaskStartScheduler();

    while (1) {}
}