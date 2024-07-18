/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
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
 */
#ifndef __MFC_CONTROLLER_BETA_PID_H__
#define __MFC_CONTROLLER_BETA_PID_H__

#include "stabilizer_types.h"
#include "math3d.h"

struct mfc_Variables{
    //Helper Variables
    struct vec4 F;
    struct vec F3;
    struct mat44 P;
    struct mat33 P3;
    float u_mfc;
    float prev_ydot;
    float prev_yddot;
    float prev_z_ref;
    float prev_pos_error;
    float prev_vel_error;
    float u_c;
    float prev_u_c;
    float F_z_min;
    float beta_min;
    float F_z_min_prev;
    float beta_min_prev;
};

void controllerOutOfTreeInit();
bool controllerOutofTreeTest(void);
void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const stabilizerStep_t stabilizerStep);

#endif //__MFC_CONTROLLER_BETA_PID_H__
