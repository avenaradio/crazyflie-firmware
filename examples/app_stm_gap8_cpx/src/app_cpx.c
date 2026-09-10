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
#include "aideck_global_parameters.h"

#define DEBUG_MODULE "APP_CPX_C"
#include "debug.h"

typedef enum {
    CPX_IF_INIT = 0,
    CPX_IF_GOTO_FIXED_COORDINATES = 1,
    CPX_IF_NEW_PARAMETERS = 2,
} CPXInternalFunction_t;

// Function prototypes
void sendParametersToEsp(Parameters_t parameters);
static void cpxPacketCallback(const CPXPacket_t* cpxRx); // Callback that is called when a CPX packet arrives
static void app_send_cpx_task(void *pvParameters);
void writeFloatToUint8Array(float value, uint8_t array[], uint16_t position);
float readUint8ArrayToFloat(const uint8_t array[], uint16_t position);

void initAppCpx(void){
    // Register a callback for CPX packets.
    // Packets sent to destination=CPX_T_STM32 and function=CPX_F_APP will arrive here
    cpxRegisterAppMessageHandler(cpxPacketCallback);
    xTaskCreate(app_send_cpx_task, "AiDeck Send CPX Task", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
}

//----------------------------------------------------------- send -------------------------------------------------------//
static void app_send_cpx_task(void *pvParameters) {
    Parameters_t parameters = {0};
    while (1) {
        if (parameters_get(&parameters) == pdPASS) {
            sendParametersToEsp(parameters);
        }
        vTaskDelay(100);
    }
}

static CPXPacket_t txp_to_esp;
void sendParametersToEsp(Parameters_t parameters){
    int parameters_count = 4;
    //DEBUG_PRINT("Sending Params to ESP x=%.4f, y=%.4f, z=%.4f, b=%.4f\n", (double)parameters.x,(double)parameters.y,(double)parameters.z,(double)parameters.batteryP);
    cpxInitRoute(CPX_T_STM32, CPX_T_ESP32, CPX_F_APP, &txp_to_esp.route); // Add route to txp_to_esp
    txp_to_esp.data[0] = (uint8_t)CPX_IF_NEW_PARAMETERS;
    writeFloatToUint8Array(parameters.x, txp_to_esp.data, 1);
    writeFloatToUint8Array(parameters.y, txp_to_esp.data, 4+1);
    writeFloatToUint8Array(parameters.z, txp_to_esp.data, 8+1);
    writeFloatToUint8Array(parameters.batteryP, txp_to_esp.data, 12+1);
    txp_to_esp.dataLength = 1 + parameters_count*4;
    cpxSendPacketBlocking(&txp_to_esp); // Send message
}

//------------------------------------------------- receive ----------------------------------------------------//

static void cpxPacketCallback(const CPXPacket_t* cpxRx) {
  DEBUG_PRINT("Got packet from %d for %d, internal function=%d\n", cpxRx->route.source, cpxRx->route.function, cpxRx->data[0]);
  // TODO switch case to call different functions
  switch (cpxRx->data[0]) {
    case CPX_IF_GOTO_FIXED_COORDINATES: {
        GoToFixPosition_t coordinates = {
                .x = readUint8ArrayToFloat(cpxRx->data, 1),
                .y = readUint8ArrayToFloat(cpxRx->data,4+1),
                .z = readUint8ArrayToFloat(cpxRx->data, 8+1)
            };
            BaseType_t result = goto_fix_position_set(&coordinates);
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

/**
 * Converts a float into an int32_t by multiplying it by 10,000,
 * then writes the 4 bytes into a uint8_t array in MSB-first order.
 *
 * @param value     Float value to convert
 * @param array     Array to write into
 * @param position  Starting position in the array
 */
void writeFloatToUint8Array(float value, uint8_t array[], uint16_t position){
    int32_t converted_value = (int32_t)(value * 10000.0f);
    uint32_t bytes = (uint32_t)converted_value;

    array[position + 0] = (uint8_t)((bytes >> 24) & 0xFF);
    array[position + 1] = (uint8_t)((bytes >> 16) & 0xFF);
    array[position + 2] = (uint8_t)((bytes >> 8) & 0xFF);
    array[position + 3] = (uint8_t)(bytes & 0xFF);
}

/**
 * Reads 4 MSB-first bytes from a uint8_t array,
 * converts them to an int32_t, and divides by 10,000
 * to recover the original float value.
 *
 * @param array     Array to read from
 * @param position  Starting position in the array
 * @return          Reconstructed float value
 */
float readUint8ArrayToFloat(const uint8_t array[], uint16_t position){
    uint32_t bytes =
        ((uint32_t)array[position + 0] << 24) |
        ((uint32_t)array[position + 1] << 16) |
        ((uint32_t)array[position + 2] << 8)  |
        ((uint32_t)array[position + 3]);

    int32_t converted_value = (int32_t)bytes;

    return (float)converted_value / 10000.0f;
}