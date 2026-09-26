// Anteater Electric Racing
#include "fault_log.h"
#include "arduino_freertos.h"
#include <Arduino.h>

static FaultLogEntry_t logBuf[FAULT_LOG_CAPACITY];
static uint16_t head = 0;
static uint16_t count = 0;
static uint32_t lastBits = 0;

static void ResetState(void) {
    head = 0;
    count = 0;
    lastBits = 0;
}

void FaultLog_Init(void) {
    taskENTER_CRITICAL();
    ResetState();
    taskEXIT_CRITICAL();
}

void FaultLog_CheckBits(uint32_t currentBits) {
    taskENTER_CRITICAL();

    uint32_t newlySet = currentBits & ~lastBits; // bits that went 0->1
    lastBits = currentBits;

    for (uint8_t bit = 0; bit < 32 && newlySet != 0; bit++) {
        if (newlySet & (1UL << bit)) {
            logBuf[head].timestamp_ms =
                xTaskGetTickCount() * portTICK_PERIOD_MS;
            logBuf[head].bit = bit;

            head = (head + 1) % FAULT_LOG_CAPACITY;
            if (count < FAULT_LOG_CAPACITY)
                count++;

            newlySet &= ~(1UL << bit); // lets the loop exit early once done
        }
    }

    taskEXIT_CRITICAL();
}

uint16_t FaultLog_Count(void) { return count; }

bool FaultLog_Get(uint16_t index, FaultLogEntry_t *out) {
    if (index >= count || out == nullptr)
        return false;
    // Once the buffer has wrapped, 'head' points at the oldest entry.
    uint16_t start = (count < FAULT_LOG_CAPACITY) ? 0 : head;
    uint16_t realIndex = (start + index) % FAULT_LOG_CAPACITY;
    taskENTER_CRITICAL();
    *out = logBuf[realIndex];
    taskEXIT_CRITICAL();
    return true;
}

void FaultLog_Clear(void) {
    taskENTER_CRITICAL();
    ResetState();
    taskEXIT_CRITICAL();
}

void FaultLog_Print(void) {
    Serial.println("--- Fault Log ---");
    FaultLogEntry_t entry;
    for (uint16_t i = 0; i < FaultLog_Count(); i++) {
        if (FaultLog_Get(i, &entry)) {
            Serial.print(entry.timestamp_ms);
            Serial.print(" ms: bit ");
            Serial.println(entry.bit);
        }
    }
    Serial.println("-----------------");

    // Uptime since this board last booted -- not wall-clock time, there's
    // no RTC here. If this is much smaller than how long the car's
    // actually been on, that gap is a reset/crash that happened somewhere
    // between power-up and now.
    uint32_t nowMs = xTaskGetTickCount() * portTICK_PERIOD_MS;
    Serial.print("Current uptime: ");
    Serial.print(nowMs);
    Serial.print(" ms (");
    Serial.print(nowMs / 1000);
    Serial.println(" s since boot)");
}