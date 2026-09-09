#ifndef CONTROL_H
#define CONTROL_H

#include "FreeRTOS.h"
#include "queue.h"

#define MAX_SPEED 0.5f // m/s

void taskAppControl(void *argument);
void goToFixedCoordinates(float x, float y, float z, float duration_s);
void land(float absoluteHeight_m, float duration_s);

#endif