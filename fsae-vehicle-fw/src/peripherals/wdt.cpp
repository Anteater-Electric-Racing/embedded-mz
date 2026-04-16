#include <Arduino.h>
#include <Watchdog_t4.h>
#include <arduino_freertos.h>

#include "peripherals/wdt.h"
#include "utils/utils.h"

#define ADC_FAULT_TIME_THRESHOLD_MS 100
#define MAIN_FAULT_TIME_THRESHOLD_MS 100
#define VCU_FAULT_TIME_THRESHOLD_MS 100
#define CAN_FAULT_TIME_THRESHOLD_MS 100

// Global watchdog tick tracking variables
TickType_t adc_last_run_tick = 0;
TickType_t main_last_run_tick = 0;
TickType_t vcu_last_run_tick = 0;
TickType_t can_last_run_tick = 0;

// Bitmask flag definition
static uint8_t WDT_BIT_ADC = 0b0001;
static uint8_t WDT_BIT_MAIN = 0b0010;
static uint8_t WDT_BIT_VCU = 0b0100;
static uint8_t WDT_BIT_CAN = 0b1000;

static uint8_t WDT_REQUIRED_MASK = 0b0000; // 0b000 represents no flags

static constexpr uint32_t WDT_CHECK_PERIOD_MS = 100;

static WDT_T4<WDT1> WDT;

void WDT_Init() {
    TickType_t now = xTaskGetTickCount();
    adc_last_run_tick = now;
    main_last_run_tick = now;
    vcu_last_run_tick = now;
    can_last_run_tick = now;

    WDT_timings_t config;

    config.timeout = 1.0; // second before reset
    config.trigger = 0.0;
    config.callback = nullptr;

    WDT.begin(config);

    Serial.println("Watchdog initialized (1 second timeout)");
}

void WDT_Update_Task(void *pvParameters) {
    TickType_t now;

    TickType_t adc_ageTicks;
    uint32_t adc_ageMs;

    TickType_t main_ageTicks;
    uint32_t main_ageMs;

    TickType_t vcu_ageTicks;
    uint32_t vcu_ageMs;

    TickType_t can_ageTicks;
    uint32_t can_ageMs;

    uint8_t mask;

    for (;;) {
        now = xTaskGetTickCount();

        adc_ageTicks = now - adc_last_run_tick;
        adc_ageMs = adc_ageTicks * portTICK_PERIOD_MS;

        main_ageTicks = now - main_last_run_tick;
        main_ageMs = main_ageTicks * portTICK_PERIOD_MS;

        vcu_ageTicks = now - vcu_last_run_tick;
        vcu_ageMs = vcu_ageTicks * portTICK_PERIOD_MS;

        can_ageTicks = now - can_last_run_tick;
        can_ageMs = can_ageTicks * portTICK_PERIOD_MS;

        mask = 0b0000;

        if (adc_ageMs >= ADC_FAULT_TIME_THRESHOLD_MS) {
            mask |= WDT_BIT_ADC;
        }
        if (main_ageMs >= MAIN_FAULT_TIME_THRESHOLD_MS) {
            mask |= WDT_BIT_MAIN;
        }
        if (vcu_ageMs >= VCU_FAULT_TIME_THRESHOLD_MS) {
            mask |= WDT_BIT_VCU;
        }
        if (can_ageMs >= CAN_FAULT_TIME_THRESHOLD_MS) {
            mask |= WDT_BIT_CAN;
        }

        // pet if 0b0000
        if (mask == WDT_REQUIRED_MASK) {
            WDT.feed(); // pet hardware watchdog
            Serial.println("WDT fed successfully");
        } else {
            if (mask & WDT_BIT_ADC) {
                Serial.println("WDT: ADC thread overdue");
            }
            if (mask & WDT_BIT_MAIN) {
                Serial.println("WDT: Main thread overdue");
            }
            if (mask & WDT_BIT_VCU) {
                Serial.println("WDT: VCU thread overdue");
            }
            if (mask & WDT_BIT_CAN) {
                Serial.println("WDT: CAN thread overdue");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(WDT_CHECK_PERIOD_MS)); // 100ms delay
    }
}