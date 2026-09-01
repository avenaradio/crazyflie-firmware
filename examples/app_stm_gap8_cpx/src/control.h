#ifndef CONTROL_H
#define CONTROL_H

#include "FreeRTOS.h"
#include "queue.h"

void taskAppControl(void *argument);
void goToFixedCoordinates(float x, float y, float z, float duration_s);
void land(float absoluteHeight_m, float duration_s);

typedef struct {
    float x;
    float y;
    float z;
    float duration;
} GotoCoordinates_t;

extern QueueHandle_t gotoQueue;


#endif