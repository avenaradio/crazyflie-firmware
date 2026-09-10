#include <float.h>
#include <math.h>

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
float calculateDistance(GoToFixPosition_t point1, GoToFixPosition_t point2);

void taskAppControl(void *argument){
    GoToFixPosition_t received_coordinates;
    Parameters_t current_parameters;
    // Init high-level commander
    crtpCommanderHighLevelInit();
    vTaskDelay(M2T(1000));
    while(1){
        vTaskDelay(M2T(50));
        if (goto_fix_position_get(&received_coordinates) == pdPASS) {
            if (parameters_get(&current_parameters) == pdPASS) {
                GoToFixPosition_t current_position = {
                    .x = current_parameters.x,
                    .y = current_parameters.y,
                    .z = current_parameters.z
                };
                float distance = calculateDistance(current_position, received_coordinates);
                // Calculate and check travel time
                float travel_time = distance / MAX_SPEED;
                if (!isfinite((double)travel_time) || travel_time < 0.1f) {
                    travel_time = 0.1f;
                }
                if (travel_time > 60.0f) {
                    travel_time = 60.0f;
                }
                DEBUG_PRINT("Moving to x=%f, y=%f, z=%f, travel_time=%f\n", (double)received_coordinates.x, (double)received_coordinates.y, (double)received_coordinates.z, (double)travel_time);
                goToFixedCoordinates(received_coordinates.x, received_coordinates.y, received_coordinates.z, travel_time);
            }
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
    vTaskDelay(M2T((uint32_t)(duration_s * 1000.0f)));
}

void land(float absoluteHeight_m, float duration_s){
    // Land
    crtpCommanderHighLevelLand(absoluteHeight_m, duration_s);
    vTaskDelay(M2T(duration_s * 1000));
    supervisorRequestArming(false);
}

/**
 * Calculates the Euclidean distance between two 3D points.
 *
 * @return Distance between point1 and point2
 */
float calculateDistance(GoToFixPosition_t point1, GoToFixPosition_t point2){
    float dx = point2.x - point1.x;
    float dy = point2.y - point1.y;
    float dz = point2.z - point1.z;
    return sqrtf((dx * dx) + (dy * dy) + (dz * dz));
}