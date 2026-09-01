
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "commander.h"
#include "crtp_commander_high_level.h"
#include "supervisor.h"

#include "control.h"

#define DEBUG_MODULE "APP_CONTROL_C"
#include "debug.h"

QueueHandle_t gotoQueue;

//Prototypes
void goToFixedCoordinates(float x, float y, float z, float duration_s);
void land(float absoluteHeight_m, float duration_s);

void taskAppControl(void *argument){
    // Init queue
    gotoQueue = xQueueCreate(5, sizeof(GotoCoordinates_t));
    GotoCoordinates_t coordinates;
    // Init high-level commander
    crtpCommanderHighLevelInit();
    vTaskDelay(M2T(1000));
    while(1){
        vTaskDelay(M2T(50));
        if (xQueueReceive(gotoQueue, &coordinates, portMAX_DELAY) == pdPASS) {
            DEBUG_PRINT("Moving to x=%f, y=%f, z=%f, speed=%f\n", (double)coordinates.x, (double)coordinates.y, (double)coordinates.z, (double)coordinates.duration);

            // Perform the movement here.
            // goToFixedCoordinates(coordinates.x, coordinates.y, coordinates.z, coordinates.duration);
        }
    }
}

void goToFixedCoordinates(float x, float y, float z, float duration_s){
    if(!supervisorIsFlying()){
        // Arm
        supervisorRequestArming(true);
        vTaskDelay(M2T(500));
        // Takeoff
        crtpCommanderHighLevelTakeoff(0.5, 2);
        vTaskDelay(M2T(2000));
    }
    // Goto coordinates
    crtpCommanderHighLevelGoTo(x, y, z, 0, duration_s, false);
    vTaskDelay(M2T(duration_s * 1000));
}

void land(float absoluteHeight_m, float duration_s){
    // Land
    crtpCommanderHighLevelLand(absoluteHeight_m, duration_s);
    vTaskDelay(M2T(duration_s * 1000));
    supervisorRequestArming(false);
}