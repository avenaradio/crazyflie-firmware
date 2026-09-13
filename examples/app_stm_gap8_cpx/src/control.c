#include <float.h>
#include <math.h>
#include <stddef.h>
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

Parameters_t current_parameters = {0};

//Prototypes
int travelTo(const GoToFixPosition_t current_pos, const GoToFixPosition_t new_pos);
int goToFixedCoordinates(const GoToFixPosition_t start_waypoint, const GoToFixPosition_t end_waypoint);
void land(float absoluteHeight_m, float duration_s);
float calculateDistance(GoToFixPosition_t point1, GoToFixPosition_t point2);
GoToFixPosition_t *createLinearWaypoints(GoToFixPosition_t start, GoToFixPosition_t end, float spacing, size_t *outCount);

void taskAppControl(void *argument){
    GoToFixPosition_t received_coordinates ={0};
    // Init high-level commander
    crtpCommanderHighLevelInit();
    vTaskDelay(M2T(500));
    while(1){
        vTaskDelay(M2T(10));
        //DEBUG_PRINT("taskAppControl: while: started\n");
        if (goto_fix_position_get(&received_coordinates) == pdPASS) {
            //DEBUG_PRINT("taskAppControl: got fix position\n");
            if (parameters_get(&current_parameters) == pdPASS) {
                GoToFixPosition_t current_position = {
                    .x = current_parameters.x,
                    .y = current_parameters.y,
                    .z = current_parameters.z
                };
                DEBUG_PRINT("Current c: x=%f, y=%f, z=%f\n", (double)current_parameters.x, (double)current_parameters.y, (double)current_parameters.z);
                DEBUG_PRINT("Moving to: x=%f, y=%f, z=%f\n", (double)received_coordinates.x, (double)received_coordinates.y, (double)received_coordinates.z);
                // goToFixedCoordinates(current_position, received_coordinates);
                travelTo(current_position, received_coordinates);
            }
        }
    }
}

int travelTo(const GoToFixPosition_t current_pos, const GoToFixPosition_t new_pos){
    size_t count;
    GoToFixPosition_t *waypoints = createLinearWaypoints(current_pos, new_pos, WAYPOINT_DISTANCE, &count);
    if (waypoints == NULL) {
        return false;
    }
    for (size_t i = 1; i < count; i++) {
        if (parameters_get(&current_parameters) == pdPASS) {
            // This only works with yaw = 0
            // if(current_params->front_mr < 0.30f){
            //     waypoints[i].x -= (0.30f - current_params->front_mr);
            // }
            // if(current_params->back_mr < 0.30f){
            //     waypoints[i].x += (0.30f - current_params->back_mr);
            // }
            // if(current_params->left_mr < 0.30f){
            //     waypoints[i].y -= (0.30f - current_params->left_mr);
            // }
            // if(current_params->right_mr < 0.30f){
            //     waypoints[i].y += (0.30f - current_params->right_mr);
            // }
            goToFixedCoordinates(waypoints[i-1], waypoints[i]);
        }
    }
    free(waypoints);
    return true;
    // goToFixedCoordinates(new_pos);
    // return true;
}

int goToFixedCoordinates(const GoToFixPosition_t start_waypoint, const GoToFixPosition_t end_waypoint){
    float distance = calculateDistance(start_waypoint, end_waypoint);
    // Calculate and check travel time
    float travel_time = distance / MAX_SPEED;
    if (!isfinite((double)travel_time) || travel_time < 0.1f) {
        travel_time = 0.1f;
    }
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
    int result = crtpCommanderHighLevelGoTo2(end_waypoint.x, end_waypoint.y, end_waypoint.z, 0, travel_time, false, false);
    // Wait non blocking
    TickType_t startTime = xTaskGetTickCount();
    TickType_t durationTicks = pdMS_TO_TICKS((uint32_t)(travel_time * 800.0f));
    while ((xTaskGetTickCount() - startTime) < durationTicks) {
        // CHeck for obstackle and modify next_position if needed
        // if (parameters_get(&current_params) == pdPASS) {
        //     if(current_params->front_mr < 0.30f){
        //         waypoints[i].x -= (0.30f - current_params->front_mr);
        //     }
        //     if(current_params->back_mr < 0.30f){
        //         waypoints[i].x += (0.30f - current_params->back_mr);
        //     }
        //     if(current_params->left_mr < 0.30f){
        //         waypoints[i].y -= (0.30f - current_params->left_mr);
        //     }
        //     if(current_params->right_mr < 0.30f){
        //         waypoints[i].y += (0.30f - current_params->right_mr);
        //     }
        // }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    // vTaskDelay(M2T((uint32_t)(travel_time * 800.0f)));
    return result;
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

/**
 * Creates waypoints from start to end.
 *
 * @param spacing: desired maximum distance between waypoints, in meters
 * @param outCount: receives the number of generated waypoints
 *
 * @return Dynamically allocated waypoint array, or NULL on failure.
 *
 * The caller must free() the returned array.
 */
GoToFixPosition_t *createLinearWaypoints(GoToFixPosition_t start, GoToFixPosition_t end, float spacing, size_t *outCount){
    if (outCount == NULL || spacing <= 0.0f) {
        return NULL;
    }
    float dx = end.x - start.x;
    float dy = end.y - start.y;
    float dz = end.z - start.z;
    float distance = sqrtf(dx * dx + dy * dy + dz * dz);
    // If both points are effectively identical
    if (distance < 1e-6f) {
        GoToFixPosition_t *waypoints = malloc(sizeof(GoToFixPosition_t));
        if (waypoints == NULL) {
            return NULL;
        }
        waypoints[0] = start;
        *outCount = 1;
        return waypoints;
    }
    // Number of intervals. ceil() ensures spacing is never greater
    // than the requested spacing.
    size_t segments = (size_t)ceilf(distance / spacing);
    size_t waypointCount = segments + 1;
    GoToFixPosition_t *waypoints = malloc(waypointCount * sizeof(GoToFixPosition_t));
    if (waypoints == NULL) {
        return NULL;
    }
    for (size_t i = 0; i < waypointCount; i++) {
        float t = (float)i / (float)segments;
        waypoints[i].x = start.x + t * dx;
        waypoints[i].y = start.y + t * dy;
        waypoints[i].z = start.z + t * dz;
    }
    // Make the final point exact, avoiding floating-point accumulation error
    waypoints[waypointCount - 1] = end;
    *outCount = waypointCount;
    return waypoints;
}