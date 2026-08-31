#include <stddef.h>
#include <string.h>

#define DEBUG_MODULE "APP_CPX_C"
#include "debug.h"

#include "cpx.h"
#include "cpx_internal_router.h"

// Callback that is called when a CPX packet arrives
static void cpxPacketCallback(const CPXPacket_t* cpxRx);
// Sends string to ESP32
void cpxToEsp(const char * data);

void initAppCpx(void){
    // Register a callback for CPX packets.
    // Packets sent to destination=CPX_T_STM32 and function=CPX_F_APP will arrive here
    cpxRegisterAppMessageHandler(cpxPacketCallback);
}

// Send string to esp
static CPXPacket_t txp_to_esp;
void cpxToEsp(const char * data) {
    if (data == NULL) {
        return;
    }
    size_t length = strlen(data);
    // Leave room for the terminating '\0'
    if (length >= sizeof(txp_to_esp.data)) {
        length = sizeof(txp_to_esp.data) - 1;
    }
    cpxInitRoute(CPX_T_STM32, CPX_T_ESP32, CPX_F_APP, &txp_to_esp.route); // Add route to txp_to_esp
    memcpy(txp_to_esp.data, data, length);
    txp_to_esp.data[length] = '\0';
    txp_to_esp.dataLength = length + 1;
    cpxSendPacketBlocking(&txp_to_esp); // Send message
    return;
}

static void cpxPacketCallback(const CPXPacket_t* cpxRx) {
  DEBUG_PRINT("Got packet from %d: (%s)\n",cpxRx->route.source, cpxRx->data);
}