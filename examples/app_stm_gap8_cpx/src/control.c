
#include "FreeRTOS.h"
#include "task.h"
#include "commander.h"
#include "crtp_commander_high_level.h"
#include "supervisor.h"

#define DEBUG_MODULE "APP_CONTROL_C"
#include "debug.h"

//Prototypes
void goToFixedCoordinates(float x, float y, float z, float yaw, float duration_s);
void land(float absoluteHeight_m, float duration_s);

void taskAppControl(void *argument){
    // Init high-level commander
    crtpCommanderHighLevelInit();
    vTaskDelay(M2T(1000));
    while(1){
        vTaskDelay(M2T(5000));
        // add task if needed
    }
}

void goToFixedCoordinates(float x, float y, float z, float yaw, float duration_s){
    if(1){ // TODO check if armed, check if flying seperatly
        // Arm
        supervisorRequestArming(true);
        vTaskDelay(M2T(500));
        // Takeoff
        crtpCommanderHighLevelTakeoff(0.5, 2);
        vTaskDelay(M2T(2000));
    }
    // Goto coordinates
    crtpCommanderHighLevelGoTo(x, y, z, yaw, duration_s, false);
    vTaskDelay(M2T(duration_s * 1000));
}

void land(float absoluteHeight_m, float duration_s){
    // Land
    crtpCommanderHighLevelLand(absoluteHeight_m, duration_s);
    vTaskDelay(M2T(duration_s * 1000));
}