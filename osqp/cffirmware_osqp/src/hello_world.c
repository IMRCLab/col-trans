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

#include "math3d.h"
// OSQP
#include "workspace.h"
#include "osqp.h"

void appMain() {

  while(1) {

    // Solve Problem
    uint64_t start = usecTimestamp();

    // use osqp_update_P, osqp_update_A, etc. to adjust the problem here 
    // A matrix have [A_equality, A_inequality]
    // A_equality: P martix in the paper (eq. 23) P @ mu == Fd
    // A_inequality: Fixed hyperplanes at angle of 20 degrees with the z-axis
    // our objective is to update the lower and the upper bounds in the first three indices only.
    // l = [Ud[0], Ud[1], Ud[2], -inf, -inf]
    // u = [Ud[0], Ud[1], Ud[2],   0,    0]
    // They are dependent on the desired forces applied on the payload.
   
    // First trial at time = 7.301 [s]
    // Fd = [-0.000687801451762	7.75278822401621E-05	0.171593864239784]
    // Expected result = [-0.005584135823817	0.029758117364301	0.085884672451666	0.004896525877707	-0.02968066880112	0.085663796567353]
    // c_float l_new[6] = {-0.000687801451762,	7.75278822401621E-05,	0.171593864239784, -INFINITY, -INFINITY};
    // c_float u_new[6] =  {-0.000687801451762,	7.75278822401621E-05,	0.171593864239784, 0, 0};
    //////////////////////////////
    // Second Trial at time = 4.589	[s]
    // Fd = [-0.001026575844217	5.48644314921118E-05	0.172234130371821]
    // Expected result = [-0.00577303063058	0.029857674746736	0.086225314351011	0.004746737438521	-0.029802894650152	0.085963251417401]
    c_float l_new[6] = {-0.001026575844217, 5.48644314921118E-05,	0.172234130371821, -INFINITY, -INFINITY};
    c_float u_new[6] =  {-0.001026575844217, 5.48644314921118E-05,	0.172234130371821, 0, 0};


    osqp_update_lower_bound(&workspace, l_new);
    osqp_update_upper_bound(&workspace, u_new);

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
