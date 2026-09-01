/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2023 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * App layer application that communicates with the GAP8 on an AI deck.
 */

#include <stddef.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"
#include "app.h"

#include "app_cpx.h"

#include "parameters.h"
#include "control.h"

#define DEBUG_MODULE "APP"
#include "debug.h"

void appMain() {
    DEBUG_PRINT("Hello! I am the stm_esp_cpx app\n");
    DEBUG_PRINT("int: %d, float: %d, uint8_t: %d\n", sizeof(int), sizeof(float), sizeof(uint8_t));
    vTaskDelay(M2T(3000));
    initAppCpx();
    xTaskCreate(taskAppParameters, "taskAppParameters", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(taskAppControl, "taskAppControl", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    while(1) {
      vTaskDelay(M2T(2000));
    }
}