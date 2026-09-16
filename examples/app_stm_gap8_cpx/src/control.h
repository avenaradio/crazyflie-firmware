#ifndef CONTROL_H
#define CONTROL_H

#include "FreeRTOS.h"
#include "queue.h"
#include "aideck_global_parameters.h"

#define MAX_SPEED 0.5f // m/s

void taskAppControl(void *argument);
void land(float absoluteHeight_m, float duration_s);
float calculateDistance(GoToFixPosition_t point1, GoToFixPosition_t point2);

#endif