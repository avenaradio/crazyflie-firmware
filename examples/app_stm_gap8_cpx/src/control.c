
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "commander.h"
#include "crtp_commander_high_level.h"
#include "supervisor.h"

#include "control.h"

#define DEBUG_MODULE "APP_CONTROL_C"
#include "debug.h"
#include "aideck_global_parameters.h"

//Prototypes
void goToFixedCoordinates(float x, float y, float z, float duration_s);
void land(float absoluteHeight_m, float duration_s);

void taskAppControl(void *argument){
    GoToFixPosition_t coordinates;
    // Init high-level commander
    crtpCommanderHighLevelInit();
    vTaskDelay(M2T(1000));
    while(1){
        vTaskDelay(M2T(50));
        if (goto_fix_position_get(&coordinates) == pdPASS) {
            DEBUG_PRINT("Moving to x=%f, y=%f, z=%f\n", (double)coordinates.x, (double)coordinates.y, (double)coordinates.z);

            // Perform the movement here.
            // TODO calc distance to get duration from MAX_SPEED
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