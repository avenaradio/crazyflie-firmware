#ifndef AIDECK_GLOBAL_PARAMETERS_H
#define AIDECK_GLOBAL_PARAMETERS_H

#include "FreeRTOS.h"
#include "queue.h"

typedef struct{
    float x;
    float y;
    float z;
    float batteryP;
} Parameters_t;
BaseType_t parameters_set(const Parameters_t *parameters);
BaseType_t parameters_get(Parameters_t *parameters);

typedef struct{
    float x;
    float y;
    float z;
} GoToFixPosition_t;
BaseType_t goto_fix_position_set(const GoToFixPosition_t *goto_fix_position);
BaseType_t goto_fix_position_get(GoToFixPosition_t *goto_fix_position);

void aideck_parameters_init(void);

#endif /* AIDECK_GLOBAL_PARAMETERS_H */
