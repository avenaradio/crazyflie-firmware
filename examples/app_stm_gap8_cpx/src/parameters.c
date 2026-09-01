#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "log.h"
#include "param.h"
#include "portmacro.h"

#include "app_cpx.h"

#define DEBUG_MODULE "APP_PARAMETERS_C"
#include "debug.h"

#define SAMPLES_FOR_AVERAGE 10
#define SAMPLE_TIME 10 // ms
#define MAX_SPEED 0.5f // m/s

// Parameters
logVarId_t idX;
logVarId_t idY;
logVarId_t idZ;
logVarId_t idBatteryP;
float x = 0.0f;
float y = 0.0f;
float z = 0.0f;
float batteryP = 0.0f;

// Arrays for average
float xa[SAMPLES_FOR_AVERAGE] = {0.0f};
float ya[SAMPLES_FOR_AVERAGE] = {0.0f};
float za[SAMPLES_FOR_AVERAGE] = {0.0f};

// Function Prototypes
void getParameters(void);
void calculateAverages(void);
static int compareFloats(const void *a, const void *b);
float median(const float values[SAMPLES_FOR_AVERAGE]);
void replaceOutliers(float values[SAMPLES_FOR_AVERAGE], float max_deviation);
float average_samples(const float values[SAMPLES_FOR_AVERAGE]);

void taskAppParameters(void *argument){
    idX = logGetVarId("stateEstimate", "x");
    idY = logGetVarId("stateEstimate", "y");
    idZ = logGetVarId("stateEstimate", "z");
    idBatteryP = logGetVarId("pm", "batteryLevel");

    while(1){
        vTaskDelay(M2T(SAMPLE_TIME));
        getParameters();
    }
}

void getParameters(void){
    static int counter = 0;
    if(counter >= SAMPLES_FOR_AVERAGE){
        counter = 0;
        batteryP = logGetFloat(idBatteryP);
        calculateAverages();
        //sendValuesToEsp();
    }
    xa[counter] = logGetFloat(idX);
    ya[counter] = logGetFloat(idY);
    za[counter] = logGetFloat(idZ);
    counter++;
}

void calculateAverages(void){
    float max_deviation = (MAX_SPEED * ((float)SAMPLE_TIME * (float)SAMPLES_FOR_AVERAGE) / 1000.0f) * 1.2f; // Maximum expected travel distance plus 20%
    replaceOutliers(xa, max_deviation);
    replaceOutliers(ya, max_deviation);
    replaceOutliers(za, max_deviation);
    x = average_samples(xa);
    y = average_samples(ya);
    z = average_samples(za);
    // DEBUG_PRINT("\nX: %f m\nY: %f m\nZ: %f m\nBattery Level: %f %%\n",
    //         (double)x,
    //         (double)y,
    //         (double)z,
    //         (double)batteryP);
}

//------------------------------------- MATH FUNCTIONS ----------------------------------------//

static int compareFloats(const void *a, const void *b){
    float x = *(const float *)a;
    float y = *(const float *)b;

    return (x > y) - (x < y);
}

float median(const float values[SAMPLES_FOR_AVERAGE]){
    float sorted[SAMPLES_FOR_AVERAGE];

    // Copy the array so the original is unchanged
    for (int i = 0; i < SAMPLES_FOR_AVERAGE; i++) {
        sorted[i] = values[i];
    }

    qsort(sorted, SAMPLES_FOR_AVERAGE, sizeof(float), compareFloats);

    if (SAMPLES_FOR_AVERAGE % 2 == 0) {
        // Even length: average the two middle values
        return (sorted[SAMPLES_FOR_AVERAGE / 2 - 1] +
                sorted[SAMPLES_FOR_AVERAGE / 2]) / 2.0f;
    } else {
        // Odd length: return the middle value
        return sorted[SAMPLES_FOR_AVERAGE / 2];
    }
}

void replaceOutliers(float values[SAMPLES_FOR_AVERAGE], float max_deviation){
    float med = median(values);

    float lower_bound = med - max_deviation;
    float upper_bound = med + max_deviation;

    for (int i = 0; i < SAMPLES_FOR_AVERAGE; i++) {
        if (values[i] < lower_bound || values[i] > upper_bound) {
            values[i] = med;
        }
    }
}

float average_samples(const float values[SAMPLES_FOR_AVERAGE]){
    float sum = 0.0f;

    for (int i = 0; i < SAMPLES_FOR_AVERAGE; i++) {
        sum += values[i];
    }

    return sum / (float)SAMPLES_FOR_AVERAGE;
}
