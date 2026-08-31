#ifndef CONTROL_H
#define CONTROL_H

void taskAppControl(void *argument);
void goToFixedCoordinates(float x, float y, float z, float yaw, float duration_s);
void land(float absoluteHeight_m, float duration_s);

#endif