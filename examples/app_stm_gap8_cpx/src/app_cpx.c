#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "cpx.h"
#include "cpx_internal_router.h"

#include "control.h"
#include "global_queues.h"

#define DEBUG_MODULE "APP_CPX_C"
#include "debug.h"

typedef enum {
    CPX_IF_INIT = 0,
    CPX_IF_GOTO_FIXED_COORDINATES = 1,
    CPX_IF_NEW_PARAMETERS = 2,
} CPXInternalFunction_t;

// Function prototypes
void send_parameters_to_esp(Parameters_t parameters);
static void cpx_receive_callback(const CPXPacket_t* cpxRx); // Callback that is called when a CPX packet arrives
static void send_cpx_task(void *pvParameters);
void write_float_to_uint8_array(float value, uint8_t array[], uint16_t position);
float read_uin8_array_to_float(const uint8_t array[], uint16_t position);

void initAppCpx(void){
    // Register a callback for CPX packets.
    // Packets sent to destination=CPX_T_STM32 and function=CPX_F_APP will arrive here
    cpxRegisterAppMessageHandler(cpx_receive_callback);
    xTaskCreate(send_cpx_task, "AiDeck Send CPX Task", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
}

//----------------------------------------------------------- send -------------------------------------------------------//
static void send_cpx_task(void *pvParameters) {
    Parameters_t parameters = {0};
    while (1) {
        if (get_parameters(&parameters) == pdPASS) {
            send_parameters_to_esp(parameters);
        }
        vTaskDelay(100);
    }
}

static CPXPacket_t txp_to_esp;
void send_parameters_to_esp(Parameters_t parameters){
    int parameters_count = 4;
    //DEBUG_PRINT("Sending Params to ESP x=%.4f, y=%.4f, z=%.4f, b=%.4f\n", (double)parameters.x,(double)parameters.y,(double)parameters.z,(double)parameters.batteryP);
    cpxInitRoute(CPX_T_STM32, CPX_T_ESP32, CPX_F_APP, &txp_to_esp.route); // Add route to txp_to_esp
    txp_to_esp.data[0] = (uint8_t)CPX_IF_NEW_PARAMETERS;
    write_float_to_uint8_array(parameters.x, txp_to_esp.data, 1);
    write_float_to_uint8_array(parameters.y, txp_to_esp.data, 4+1);
    write_float_to_uint8_array(parameters.z, txp_to_esp.data, 8+1);
    write_float_to_uint8_array(parameters.batteryP, txp_to_esp.data, 12+1);
    txp_to_esp.dataLength = 1 + parameters_count*4;
    cpxSendPacketBlocking(&txp_to_esp); // Send message
}

//------------------------------------------------- receive ----------------------------------------------------//

static void cpx_receive_callback(const CPXPacket_t* cpxRx) {
  //DEBUG_PRINT("Got packet from %d for %d, internal function=%d\n", cpxRx->route.source, cpxRx->route.function, cpxRx->data[0]);
  // TODO switch case to call different functions
  switch (cpxRx->data[0]) {
    case CPX_IF_GOTO_FIXED_COORDINATES: {
        GoToPosition_t coordinates = {
                .x = read_uin8_array_to_float(cpxRx->data, 1),
                .y = read_uin8_array_to_float(cpxRx->data,4+1),
                .z = read_uin8_array_to_float(cpxRx->data, 8+1)
            };
            BaseType_t result = set_goto_position(&coordinates);
            if (result != pdPASS) {
                DEBUG_PRINT("Failed to set goto_fix_position_set\n");
            }
        break;
    }
    default:
            break;
  }
}

//------------------------------------- HELPER FUNCTINS ----------------------------------------------------//

void write_float_to_uint8_array(float value, uint8_t array[], uint16_t position){
    if (sizeof(float) != 4){
        return;
    }
    memcpy(&array[position], &value, sizeof(float));
}

float read_uin8_array_to_float(const uint8_t array[],uint16_t position){
    float value;
    if (sizeof(float) != 4){
        return 0.0f;
    }
    memcpy(&value, &array[position], sizeof(value));
    return value;
}
