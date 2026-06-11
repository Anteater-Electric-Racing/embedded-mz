#pragma once

#include <ADC.h>
#define SENSOR_PIN_AMT_ADC0 11
#define SENSOR_PIN_AMT_ADC1 11

extern uint16_t adc0Pins[SENSOR_PIN_AMT_ADC0];
extern uint16_t adc0Index;
extern uint16_t adc0Reads[SENSOR_PIN_AMT_ADC0];

extern uint16_t adc1Pins[SENSOR_PIN_AMT_ADC1];
extern uint16_t adc1Index;
extern uint16_t adc1Reads[SENSOR_PIN_AMT_ADC1];

extern ADC *adc;

void ADC_Init();
uint16_t ADC_GetAPPS1Value();
uint16_t ADC_GetAPPS2Value();
void threadADC(void *pvParameters);
