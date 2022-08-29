/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
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
 * hello_world.c - App layer application of a simple hello world debug print every
 *   2 seconds.
 */


#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "HELLOWORLD"
#include "debug.h"

#include "usec_time.h"


// OSQP
#include "workspace.h"
#include "osqp.h"

void appMain() {

  while(1) {

    // Solve Problem
    uint64_t start = usecTimestamp();

    // use osqp_update_P, osqp_update_A, etc. to adjust the problem here

    osqp_solve(&workspace);
    uint64_t end = usecTimestamp();

    // Print status
    DEBUG_PRINT("Status:                %s\n", (&workspace)->info->status);
    // DEBUG_PRINT("Number of iterations:  %d\n", (int)((&workspace)->info->iter));
    // DEBUG_PRINT("Objective value:       %f\n", (&workspace)->info->obj_val);
    // DEBUG_PRINT("Primal residual:       %f\n", (&workspace)->info->pri_res);
    // DEBUG_PRINT("Dual residual:         %f\n", (&workspace)->info->dua_res);
     for (int i = 0; i < 6; ++i) {
      DEBUG_PRINT("result (%d): %f\n", i, (double)((&workspace)->solution->x[i]));
    }
    DEBUG_PRINT("time: %d us\n", (int)(end - start));


    vTaskDelay(M2T(1000));

  }
}
