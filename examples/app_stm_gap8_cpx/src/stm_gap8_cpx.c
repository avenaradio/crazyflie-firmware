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


#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "commander.h"
#include "crtp_commander_high_level.h"
#include "supervisor.h"

#include "log.h"
#include "param.h"
#include "cpx.h"
#include "cpx_internal_router.h"

#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "APP"
#include "debug.h"

// Callback that is called when a CPX packet arrives
static void cpxPacketCallback(const CPXPacket_t* cpxRx);

static CPXPacket_t txPacket;

void appMain() {
  DEBUG_PRINT("Hello! I am the stm_gap8_cpx app\n");

  // Register a callback for CPX packets.
  // Packets sent to destination=CPX_T_STM32 and function=CPX_F_APP will arrive here
  cpxRegisterAppMessageHandler(cpxPacketCallback);
  
  vTaskDelay(M2T(3000));

  // Init high-level commander
  crtpCommanderHighLevelInit();
  vTaskDelay(M2T(1000));
  // Arm
  supervisorRequestArming(true);
  vTaskDelay(M2T(1000));
  // Takeoff
  crtpCommanderHighLevelTakeoff(0.5, 1);
  vTaskDelay(M2T(3000));
  // Goto 1,1,1
  crtpCommanderHighLevelGoTo(1, 1, 1, 0, 2, false);
  vTaskDelay(M2T(3000));
  // Land
  crtpCommanderHighLevelLand(0, 1);

  uint8_t counter = 0;
  while(1) {
    vTaskDelay(M2T(2000));

    cpxInitRoute(CPX_T_STM32, CPX_T_GAP8, CPX_F_APP, &txPacket.route);
    txPacket.data[0] = counter;
    txPacket.dataLength = 1;

    cpxSendPacketBlocking(&txPacket);
    DEBUG_PRINT("Sent packet to GAP8 (%u)\n", counter);
    counter++;
  }
}

static void cpxPacketCallback(const CPXPacket_t* cpxRx) {
  DEBUG_PRINT("Got packet from GAP8 (%u)\n", cpxRx->data[0]);
}
