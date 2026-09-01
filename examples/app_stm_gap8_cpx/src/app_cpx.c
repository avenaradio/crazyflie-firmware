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

#define DEBUG_MODULE "APP_CPX_C"
#include "debug.h"

#define FLOAT_STRING_SIZE 9

typedef enum {
  CPX_IF_GOTO_FIXED_COORDINATES = 1,
  CPX_IF_NEW_PARAMETERS = 2,
} CPXInternalFunction_t;

// Function prototypes
void sendParametersToEsp(float x, float y, float z, float batteryPercent);
static void cpxPacketCallback(const CPXPacket_t* cpxRx); // Callback that is called when a CPX packet arrives
void cpxToEsp(const char * data);
const char *floatToString(float value);
float stringToFloat(const char *text);

void initAppCpx(void){
    // Register a callback for CPX packets.
    // Packets sent to destination=CPX_T_STM32 and function=CPX_F_APP will arrive here
    cpxRegisterAppMessageHandler(cpxPacketCallback);
}

//----------------------------------------------------------- send -------------------------------------------------------//

static CPXPacket_t txp_to_esp;
void sendParametersToEsp(float x, float y, float z, float batteryP){
    const char *xs = floatToString(x);
    const char *ys = floatToString(y);
    const char *zs = floatToString(z);
    const char *batteryPs= floatToString(batteryP);
    size_t length = 1;
    length += strlen(xs);
    length += strlen(ys);
    length += strlen(zs);
    length += strlen(batteryPs);
    char data[length];
    data[0] = (char)CPX_IF_NEW_PARAMETERS;
    size_t position = 1;
    strcpy(data + position, xs);
    position += strlen(xs);
    strcpy(data + position, ys);
    position += strlen(ys);
    strcpy(data + position, zs);
    position += strlen(zs);
    strcpy(data + position, batteryPs);
    position += strlen(batteryPs);

    cpxInitRoute(CPX_T_STM32, CPX_T_ESP32, CPX_F_APP, &txp_to_esp.route); // Add route to txp_to_esp
    memcpy(txp_to_esp.data, data, length);
    txp_to_esp.dataLength = length;
    cpxSendPacketBlocking(&txp_to_esp); // Send message
}

// // Send string to esp
// static CPXPacket_t txp_to_esp;
// void cpxToEsp(const char * data) {
//     if (data == NULL) {
//         return;
//     }
//     size_t length = strlen(data);
//     // Leave room for the terminating '\0'
//     if (length >= sizeof(txp_to_esp.data)) {
//         length = sizeof(txp_to_esp.data) - 1;
//     }
//     cpxInitRoute(CPX_T_STM32, CPX_T_ESP32, CPX_F_APP, &txp_to_esp.route); // Add route to txp_to_esp
//     memcpy(txp_to_esp.data, data, length);
//     txp_to_esp.data[length] = '\0';
//     txp_to_esp.dataLength = length + 1;
//     cpxSendPacketBlocking(&txp_to_esp); // Send message
//     return;
// }

//------------------------------------------------- receive ----------------------------------------------------//

static void cpxPacketCallback(const CPXPacket_t* cpxRx) {
  DEBUG_PRINT("Got packet from %d for %d, if=%d: (%s)\n", cpxRx->route.source, cpxRx->route.function, cpxRx->data[0], cpxRx->data);
  // TODO switch case to call different functions
  switch (cpxRx->data[0]) {
    case CPX_IF_GOTO_FIXED_COORDINATES: {
        GotoCoordinates_t coordinates = {
                .x = stringToFloat(&cpxRx->data[1]),
                .y = stringToFloat(&cpxRx->data[1 + FLOAT_STRING_SIZE]),
                .z = stringToFloat(&cpxRx->data[1 + FLOAT_STRING_SIZE * 2]),
                .duration = stringToFloat(&cpxRx->data[1 + FLOAT_STRING_SIZE * 3])
            };
            // Wait up to 100 ms if the queue is full.
            BaseType_t result = xQueueSend(gotoQueue, &coordinates, pdMS_TO_TICKS(100));
            if (result != pdPASS) {
                // Queue was full or an error occurred.
                DEBUG_PRINT("Failed to queue coordinates\n");
            }
        break;
    }
    default:
            break;
  }
}

//------------------------------------- HELPER FUNCTINS ----------------------------------------------------//

/** Converts float into -99.9999 string 
* @return 9 char string "-99.9999\n"
*/
const char *floatToString(float value){
    // FLOAT_STRING_SIZE must be 9
    static char text[9];
    int pos = 0;
    int whole;
    int fraction;
    if(value < 0.0f){
        text[pos++] = '-';
        value = -value; // Make positive
    }
    whole = (int)value;
    fraction = (int)((value - whole) * 10000.0f);
    text[pos++] = '0' + (whole / 10);
    text[pos++] = '0' + (whole % 10);
    text[pos++] = '.';
    text[pos++] = '0' + (fraction / 1000);
    text[pos++] = '0' + ((fraction / 100) % 10);
    text[pos++] = '0' + ((fraction / 10) % 10);
    text[pos++] = '0' + (fraction % 10);
    text[pos] = '\0';
    return text;
}

/** Converts a string to a float. */
float stringToFloat(const char *text){
    if (text == NULL) {
        return 0.0f;
    }
    while (*text == ' ' || *text == '\t' ||
           *text == '\n' || *text == '\r') {
        text++;
    }
    bool negative = false;
    if (*text == '+' || *text == '-') {
        negative = (*text == '-');
        text++;
    }
    float value = 0.0f;
    while (*text >= '0' && *text <= '9') {
        value = value * 10.0f + (float)(*text - '0');
        text++;
    }
    if (*text == '.') {
        text++;
        float factor = 0.1f;
        while (*text >= '0' && *text <= '9') {
            value += (float)(*text - '0') * factor;
            factor *= 0.1f;
            text++;
        }
    }
    return negative ? -value : value;
}
