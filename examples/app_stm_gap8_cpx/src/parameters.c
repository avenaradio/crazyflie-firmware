
#include "FreeRTOS.h"
#include "task.h"
#include "log.h"
#include "param.h"
#include "portmacro.h"

#define DEBUG_MODULE "APP_PARAMETERS_C"
#include "debug.h"

#define SAMPLES_FOR_AVERAGE 10

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

void taskAppParameters(void *argument){
    idX = logGetVarId("stateEstimate", "x");
    idY = logGetVarId("stateEstimate", "y");
    idZ = logGetVarId("stateEstimate", "z");
    idBatteryP = logGetVarId("pm", "batteryLevel");

    while(1){
        vTaskDelay(M2T(10));
        getParameters();
    }
}

void getParameters(void){
    static int counter = 0;
    xa[counter] = logGetFloat(idX);
    ya[counter] = logGetFloat(idY);
    za[counter] = logGetFloat(idZ);
    if(counter >= SAMPLES_FOR_AVERAGE){
        counter = 0;
        calculateAverages();
    }

    batteryP = logGetFloat(idBatteryP);
    // DEBUG_PRINT("X: %f m\nY: %f m\nZ: %f m\nBattery Level: %f %%\n",
    //         (double)x,
    //         (double)y,
    //         (double)z,
    //         (double)batteryP);
}

void calculateAverages(void){
    // Calculate averages
    // Send to esp32
}