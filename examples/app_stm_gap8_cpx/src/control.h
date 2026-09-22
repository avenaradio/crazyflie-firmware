#ifndef CONTROL_H
#define CONTROL_H

#include "FreeRTOS.h"
#include "queue.h"
#include "global_queues.h"

#define MAX_SPEED 0.5f // m/s

void control_task(void *argument);
void land(float absoluteHeight_m, float duration_s);
float calculate_distance(GoToPosition_t point1, GoToPosition_t point2);

#endif