#pragma once

#include <arduino_freertos.h>

extern TickType_t adc_last_run_tick;
extern TickType_t can_last_run_tick;
extern TickType_t main_last_run_tick;
extern TickType_t vcu_last_run_tick;

void WDT_Init();
void threadWDT(void *pvParameters);