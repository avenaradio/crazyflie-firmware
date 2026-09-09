#include "FreeRTOS.h"
#include "queue.h"
#include "FreeRTOSConfig.h"
#include "aideck_global_parameters.h"

QueueHandle_t parameters_queue = NULL;
QueueHandle_t goto_fix_position_queue = NULL;

void aideck_parameters_init(void){
    parameters_queue = xQueueCreate(1, sizeof(Parameters_t));
    goto_fix_position_queue = xQueueCreate(1, sizeof(GoToFixPosition_t));
}

BaseType_t parameters_set(const Parameters_t *parameters){
    if(parameters == NULL || parameters_queue == NULL){
        return pdFALSE;
    }
    // Queue length is 1, so this replaces the previous value.
    return xQueueOverwrite(parameters_queue, parameters);
}

BaseType_t parameters_get(Parameters_t *parameters){
    if (parameters == NULL || parameters_queue == NULL){
        return pdFALSE;
    }
    // Peek keeps the latest value in the queue.
    return xQueuePeek(
        parameters_queue,
        parameters,
        portMAX_DELAY
    );
}

BaseType_t goto_fix_position_set(const GoToFixPosition_t *goto_fix_position){
    if(goto_fix_position == NULL || goto_fix_position_queue == NULL){
        return pdFALSE;
    }
    // Queue length is 1, so this replaces the previous value.
    return xQueueOverwrite(goto_fix_position_queue, goto_fix_position);
}

BaseType_t goto_fix_position_get(GoToFixPosition_t *goto_fix_position){
    if (goto_fix_position == NULL || goto_fix_position_queue == NULL){
        return pdFALSE;
    }
    // Peek keeps the latest value in the queue.
    return xQueuePeek(
        goto_fix_position_queue,
        goto_fix_position,
        portMAX_DELAY
    );
}