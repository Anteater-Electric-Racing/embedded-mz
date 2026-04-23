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

enum SensorIndexesADC0 
{    // TODO: Update with real values
    APPS_1_INDEX            = 5,
    APPS_2_INDEX            = 4, // A4
    SUSP_TRAV_LINPOT1       = 6,
    SUSP_TRAV_LINPOT2       = 7,
    SUSP_TRAV_LINPOT3       = 8,
    SUSP_TRAV_LINPOT4       = 9,

    BSE_1_INDEX             = 3,
    BSE_2_INDEX             = 2,
    THERMISTOR_1_INDEX      = 0, // A0
    THERMISTOR_2_INDEX      = 10, // A1
    THERMISTOR_3_INDEX,      // A2
    THERMISTOR_4_INDEX       // A3
};

enum SensorIndexesADC1 
{ // TODO: Update with real values
    APPS_1_INDEX2,
    APPS_2_INDEX2,
    BSE_1_INDEX2,
    BSE_2_INDEX2,
    SUSP_TRAV_LINPOT12,
    SUSP_TRAV_LINPOT22,
    SUSP_TRAV_LINPOT32,
    SUSP_TRAV_LINPOT42
};

void ADC_Init();
void threadADC(void *pvParameters);
