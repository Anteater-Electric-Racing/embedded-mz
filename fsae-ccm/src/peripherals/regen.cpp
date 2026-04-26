#include "peripherals/regen.h"
#include "./utils/utils.h"
#include "./vehicle/devices/bse.h"
#include "arduino_freertos.h"

// Anteater Electric Racing, 2026

/* Notes:

Temporarily i am assumign 20% brake input means 100% regen, and linearly mapping
between 0-20% brake to 0-100% regen. This is just for testing purposes and can
be updated with a more complex mapping if needed.

Furthermore, we still need to figure out how to prioritize regen vs friction
braking to avoid both being active, and how to smoothly transition between the
two.

*/

float regen_GetRegenPercent() {
    regenData.brakePercent =
        CLAMP(LINEAR_MAP(BSE_GetBSEReading()->bseRear_Reading,
                         BSE_LOWER_THRESHOLD, BSE_UPPER_THRESHOLD, 0, 1),
              0, 1);

    regenData.regenPercent = CLAMP(
        LINEAR_MAP(regenData.brakePercent, 0, 0.2, 0, 1), 0,
        1); // For now, regen percent is directly proportional to brake percent.
            // This can be updated with a more complex mapping if needed.

    return regenData.regenPercent;
}
static TickType_t lastWakeTime;
bool regenActive() {
    return regenData.regenPercent > 0 &&
           (regenData.brakePercent > 0 && regenData.brakePercent <= 0.2);
}
void threadRegen(void *pvParameters) {
    lastWakeTime = xTaskGetTickCount();
    while (true) {
        regen_GetRegenPercent();
        if (regenActive())
            regen_GetRegenPercent();
        vTaskDelayUntil(&lastWakeTime, TICKTYPE_FREQUENCY);
    }
}