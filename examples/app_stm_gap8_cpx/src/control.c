#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

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

#define WAYPOINT_DISTANCE 0.02f // in m
#define AVOID_RADIUS 0.40f // in m

Parameters_t current_parameters = {0};

//Prototypes
int travelTo(const GoToFixPosition_t current_pos, const GoToFixPosition_t new_pos);
float goToFixedCoordinates(const GoToFixPosition_t start_waypoint, const GoToFixPosition_t end_waypoint);
bool avoid_collisions(float duration);
void land(float absoluteHeight_m, float duration_s);
float calculateDistance(GoToFixPosition_t point1, GoToFixPosition_t point2);
GoToFixPosition_t *createLinearWaypoints(GoToFixPosition_t start, GoToFixPosition_t end, float spacing, size_t *outCount);
static float clamp_float(float value, float min, float max);

void taskAppControl(void *argument){
    GoToFixPosition_t received_coordinates ={0};
    float time_to_wait = 0;
    // Init high-level commander
    crtpCommanderHighLevelInit();
    vTaskDelay(M2T(500));
    while(1){
        vTaskDelay(M2T(10));
        //DEBUG_PRINT("taskAppControl: while: started\n");
        if (goto_fix_position_get(&received_coordinates) == pdPASS) {
            //DEBUG_PRINT("taskAppControl: got fix position\n");
            bool goto_successful = false;
            while (!goto_successful){
                if (parameters_get(&current_parameters) == pdPASS) {
                    GoToFixPosition_t current_position = {
                        .x = current_parameters.x,
                        .y = current_parameters.y,
                        .z = current_parameters.z
                    };
                    // DEBUG_PRINT("Current c: x=%f, y=%f, z=%f\n", (double)current_parameters.x, (double)current_parameters.y, (double)current_parameters.z);
                    DEBUG_PRINT("--- taskAppControl moving to: x=%f, y=%f, z=%f\n", (double)received_coordinates.x, (double)received_coordinates.y, (double)received_coordinates.z);
                    time_to_wait = goToFixedCoordinates(current_position, received_coordinates);
                    goto_successful = avoid_collisions(time_to_wait);
                }
            }
        } // Else no new position, avoid
    }
}

/**
* @return float time needed for movement in s
*/
float goToFixedCoordinates(const GoToFixPosition_t start_waypoint, const GoToFixPosition_t end_waypoint){
    float distance = calculateDistance(start_waypoint, end_waypoint);
    // Calculate and check travel time
    float travel_time = distance / MAX_SPEED;
    // if (!isfinite((double)travel_time) || travel_time < 0.01f) {
    //     travel_time = 0.01f;
    // }
    if (travel_time > 60.0f) {
        travel_time = 60.0f;
    }
    if(!supervisorIsFlying()){
        // Arm
        supervisorRequestArming(true);
        vTaskDelay(M2T(500));
        // Takeoff
        crtpCommanderHighLevelTakeoff(0.5, 2);
        vTaskDelay(M2T(2000));
    }
    // Goto coordinates
    DEBUG_PRINT("Current c: x=%f, y=%f, z=%f\n", (double)start_waypoint.x, (double)start_waypoint.y, (double)start_waypoint.z);
    DEBUG_PRINT("Moving to: x=%f, y=%f, z=%f\n", (double)end_waypoint.x, (double)end_waypoint.y, (double)end_waypoint.z);
    crtpCommanderHighLevelGoTo2(end_waypoint.x, end_waypoint.y, end_waypoint.z, 0, travel_time, false, false);
    // vTaskDelay(M2T((uint32_t)(travel_time * 800.0f)));
    return travel_time;
}

/** Wait and check if object close
* @return true if no object detected, false if object detected and moved to safe position
*/
bool avoid_collisions(float duration){
    bool successful = true;
    // Wait non blocking
    GoToFixPosition_t pos_from_params = {0};
    GoToFixPosition_t avoid_position = {0};
    TickType_t startTime = xTaskGetTickCount();
    TickType_t durationTicks = pdMS_TO_TICKS((uint32_t)(duration * 1200.0f));
    while ((xTaskGetTickCount() - startTime) < durationTicks) {
        vTaskDelay(pdMS_TO_TICKS(10));
        if (parameters_get(&current_parameters) == pdPASS) {
            bool trigger_avoid = false;
            pos_from_params.x = current_parameters.x;
            pos_from_params.y = current_parameters.y;
            pos_from_params.z = current_parameters.z;
            avoid_position = pos_from_params;
            // This only works with yaw = 0
            if(current_parameters.front_mr < AVOID_RADIUS){
                avoid_position.x -= (clamp_float((AVOID_RADIUS - current_parameters.front_mr), 0.0f, 0.1f));
                trigger_avoid = true;
            }
            if(current_parameters.back_mr < AVOID_RADIUS){
                avoid_position.x += (clamp_float((AVOID_RADIUS - current_parameters.back_mr), 0.0f, 0.1f));
                trigger_avoid = true;
            }
            if(current_parameters.left_mr < AVOID_RADIUS){
                avoid_position.y -= (clamp_float((AVOID_RADIUS - current_parameters.left_mr), 0.0f, 0.1f));
                trigger_avoid = true;
            }
            if(current_parameters.right_mr < AVOID_RADIUS){
                avoid_position.y += (clamp_float((AVOID_RADIUS - current_parameters.right_mr), 0.0f, 0.1f));
                trigger_avoid = true;
            }
            if(trigger_avoid) {
                DEBUG_PRINT("Obstackle detected: front=%f, back=%f, left=%f, right=%f\n", (double)current_parameters.front_mr, (double)current_parameters.back_mr, (double)current_parameters.left_mr, (double)current_parameters.right_mr);
                float time_to_wait = goToFixedCoordinates(pos_from_params, avoid_position);
                successful = false;
                startTime = xTaskGetTickCount();
                durationTicks = pdMS_TO_TICKS((uint32_t)(time_to_wait * 1200.0f));
            }
        }
    }
    return successful;
}

void land(float absoluteHeight_m, float duration_s){
    // Land
    crtpCommanderHighLevelLand(absoluteHeight_m, duration_s);
    vTaskDelay(M2T(duration_s * 1000));
    supervisorRequestArming(false);
}

// --------------------------------------- HELPER FUNCTIONS -------------------------------------------- //

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

/*
 * Clamp a value to the given range.
 */
static float clamp_float(float value, float min, float max){
    if (value < min) return min;
    if (value > max) return max;
    return value;
}