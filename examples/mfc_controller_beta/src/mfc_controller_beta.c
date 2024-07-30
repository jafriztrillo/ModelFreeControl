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
 */


#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "app.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h> 
#include <time.h> 
#include "usec_time.h"

#define DEBUG_MODULE "MFC_CONTROLLER_BETA_EST"
#include "debug.h"
// #include "controller.h"
// #include "controller_pid.h"
#include "math3d.h"
#include "mfc_controller_beta.h"
#include "position_controller.h"
#include "attitude_controller.h"
#include "log.h"
#include "physicalConstants.h"
#include "platform_defaults.h"
#include "num.h"
#include "param.h"

#include "controller_lee.h"


//Controller Gains
const static int CTRL_RATE = 360;
float kp_z = 31.0f;
float kd_z = 16.0f;
float beta_z = 57.0f;
float w3 = 5.0f;
float w4 = 50.0f;
// float u_mfc = 0;
float S[4] = {0.0f};
float ATTITUDE_UPDATE_DT = (float)(1.0f/ATTITUDE_RATE);
float DT_POS = (float)(1.0f/CTRL_RATE);
float state_body_x, state_body_y, state_body_vx, state_body_vy;
static float pwmToThrustA = 0.091492681f;
static float pwmToThrustB = 0.067673604f;
static int gainFlag = 0;


static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float actuatorThrust;
static float actuatorThrustPID;
float yd_dotLog;
float yd_ddotLog;
static float lpf = 0.13f;
static float posErrorLog;
static float velErrorLog;

//Logging Stuff
static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;
static float r_roll;
static float r_pitch;
static float r_yaw;
static float accelz;

int16_t resetTick;
uint64_t start_time,end_time;

static struct mfc_Variables mfc = {
  .F.v[0] = 0.0f,
  .F.v[1] = 0.0f,
  .F.v[2] = -10.0f,
  .F.v[3] = 57.0f,
  .P.m[0][0] = 1e-6f,
  .P.m[1][1] = 1e-6f,
  .P.m[2][2] = 1e-6f,
  .P.m[3][3] = 1e-6f,
  .u_mfc = 0.0f,
  .prev_ydot = 0.0f,
  .prev_yddot = 0.0f,
  .prev_vel_error = 0.0f,
  .prev_pos_error = 0.0f,
  .u_c = 0.0f,
  .F_z_min = -10.0f,
  .beta_min = 47.0f,
  .F_z_min_prev =  -10.0f,
  .beta_min_prev = 47.0f,
};


static float capAngle(float angle) {
  float result = angle;

  while (result > 180.0f) {
    result -= 360.0f;
  }

  while (result < -180.0f) {
    result += 360.0f;
  }

  return result;
}

void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");

  while(1) {
    vTaskDelay(M2T(2000));
    // DEBUG_PRINT("Hello World!\n");
  }
}


void controllerOutOfTreeInit() {
  // Initialize your controller data here...
  // Call the PID controller instead in this example to make it possible to fly
  // controllerPidInit();
  attitudeControllerInit(ATTITUDE_UPDATE_DT);
  positionControllerInit();
  // mfc.F.v[3] = beta_z;
  // mfc.beta_min = beta_z;
}

bool controllerOutOfTreeTest() {
  bool pass = true;

  pass &= attitudeControllerTest();

  return pass;
}

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const stabilizerStep_t stabilizerStep) {
  control->controlMode = controlModeLegacy;
  //Exactly the Attitude PID controller from Bitcraze
  if (RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)) {
    // Rate-controled YAW is moving YAW angle setpoint
    if (setpoint->mode.yaw == modeVelocity) {
      attitudeDesired.yaw = capAngle(attitudeDesired.yaw + setpoint->attitudeRate.yaw * ATTITUDE_UPDATE_DT);

      float yawMaxDelta = attitudeControllerGetYawMaxDelta();
      if (yawMaxDelta != 0.0f)
      {
      float delta = capAngle(attitudeDesired.yaw-state->attitude.yaw);
      // keep the yaw setpoint within +/- yawMaxDelta from the current yaw
        if (delta > yawMaxDelta)
        {
          attitudeDesired.yaw = state->attitude.yaw + yawMaxDelta;
        }
        else if (delta < -yawMaxDelta)
        {
          attitudeDesired.yaw = state->attitude.yaw - yawMaxDelta;
        }
      }
    } else if (setpoint->mode.yaw == modeAbs) {
      attitudeDesired.yaw = setpoint->attitude.yaw;
    } else if (setpoint->mode.quat == modeAbs) {
      struct quat setpoint_quat = mkquat(setpoint->attitudeQuaternion.x, setpoint->attitudeQuaternion.y, setpoint->attitudeQuaternion.z, setpoint->attitudeQuaternion.w);
      struct vec rpy = quat2rpy(setpoint_quat);
      attitudeDesired.yaw = degrees(rpy.z);
    }

    attitudeDesired.yaw = capAngle(attitudeDesired.yaw);
  }

  // This will enforce the controller to only update at 100Hz
  if (RATE_DO_EXECUTE(POSITION_RATE,stabilizerStep)){
    //XY
    positionController(&actuatorThrustPID, &attitudeDesired, setpoint, state);
    if(gainFlag == 0){
      actuatorThrust = actuatorThrustPID;
      mfc.u_mfc = (powf(((actuatorThrust / UINT16_MAX) * 2.0f * pwmToThrustA + pwmToThrustB),2) - pwmToThrustB*pwmToThrustB) / (4.0f * pwmToThrustA);
      if(state->position.z > 0.85f){gainFlag = 1;}
    }
  }
  // Controller Scheduler (Run this once F has stabilzed)
  if(RATE_DO_EXECUTE(CTRL_RATE, stabilizerStep)) {
    if (setpoint->mode.z == modeAbs){
      //Gain Scheduler 
      // =============== Internal Controller ==================
      float posError = state->position.z - setpoint->position.z;
      float velError = state->velocity.z - setpoint->velocity.z;
      mfc.u_c = kp_z*posError + kd_z*velError;
      mfc.u_c = lpf*mfc.u_c + (1-lpf)*mfc.prev_u_c;
      mfc.prev_u_c = mfc.u_c;
      posErrorLog = posError;
      velErrorLog = velError;

      // =============== Estimation of F ===============
      //Predeclared Constant Matrices
      struct mat44 Q = mdiag44(0.1f, 0.1f, 0.1f, 0.1f);
      struct vec4 H = mkvec4(1.0f, 0.0f, 0.0f, 0.0f);   
      float Harr[4] = {1.0f, 0.0f, 0.0f,0.0f};
      struct mat44 A =  {{{1.0f, DT_POS, DT_POS*DT_POS*0.5f, mfc.u_mfc*DT_POS*DT_POS*0.5f},
                          {0.0f, 1.0f, DT_POS, DT_POS*mfc.u_mfc},
                          {0.0f, 0.0f, 1.0f, 0.0f},
                          {0.0f, 0.0f, 0.0f, 1.0f}}};
      struct mat44 I = meye44();
      float HPHR = 0.0025f*0.0025f; //This is from Bitcraze themselves. The actual stdDev is a function but it varies from 0.0025:0.003 between 0m and 1m
      struct mat44 P_plus_prev = mfc.P;

      //State Prediction
      S[0] = mfc.F.v[0] + mfc.F.v[1]*DT_POS + 0.5f*mfc.F.v[2]*DT_POS*DT_POS + 0.5f*mfc.F.v[3]*DT_POS*DT_POS*mfc.u_mfc;
      S[1] = mfc.F.v[1] + mfc.F.v[2]*DT_POS + mfc.F.v[3]*DT_POS*mfc.u_mfc;
      S[2] = mfc.F.v[2];
      S[3] = mfc.F.v[3];

      //Covariance Prediction
      struct mat44 AT = mtranspose44(A);
      struct mat44 PAT = mmul44(P_plus_prev, AT);
      struct mat44 APAT = mmul44(A,PAT);
      struct mat44 P_minus = madd44(APAT,Q);

      struct vec4 PHT = mvmul44(P_minus,H);
      float PHTarr[4] = {PHT.v[0], PHT.v[1], PHT.v[2], PHT.v[3]};

      //Helper for Scalar Update
      for(int i = 0; i < 4; i++){
          HPHR += Harr[i]*PHTarr[i];
      }
      float try = 1.0f/HPHR;
      
      //Calculate Kalman Gain/Load into Float for Looping
      struct vec4 Kv = vscl4(try, PHT);
      float K[4] = {Kv.v[0], Kv.v[1], Kv.v[2], Kv.v[3]};
      float F_err = state->position.z - S[0];

      //State Measurement Update
      for(int i = 0; i < 4; i++){
          S[i] = S[i] + K[i]*F_err;
      }

      //Covariance Measurement Update
      struct mat44 KH = mvecmult44(Kv,H);
      struct mat44 IKH = msub44(I,KH);
      struct mat44 P_plus = mmul44(IKH,P_minus);

      //Enforce Covariance Boundaries
      for(int i = 0; i < 4; ++i){
        for(int j = i; j < 4;++j){
          float p = 0.5f*P_plus.m[i][j] + 0.5f*P_plus.m[j][i];
          if(isnan(p) || p > 100.0f){
            P_plus.m[i][j] = P_plus.m[j][i] = 100.0f;
          } else if (i==j && p < 1e-6f){
            P_plus.m[i][j] = P_plus.m[j][i] = 1e-6f;
          } else {
            P_plus.m[i][j] = P_plus.m[j][i] = p;
          }
        }
      }

      mfc.P = P_plus;
      mfc.F.v[0] = S[0];
      mfc.F.v[1] = S[1];
      mfc.F.v[2] = S[2];
      mfc.F.v[3] = S[3];

      //Direct Solution
      float yd_ddot_min = state->acc.z; // Direct use from trajectory generation

        //Standard MFC Approach W/ ZoH
      //   if(fabs(yd_ddot) > 2.5){
      //     yd_ddot = mfc.prev_yddot;
      //   }
      //   else{
      //     yd_ddot = lpf*yd_ddot + (1.0f-lpf)*mfc.prev_yddot;
      //   }
      float kw3 = (1.0f/w3);
      float kw4 = (1.0f/w4);

      mfc.F_z_min = (mfc.F_z_min*mfc.u_mfc*mfc.u_mfc*kw3*kw3 - mfc.beta_min*mfc.u_mfc*kw4*kw4 + yd_ddot_min*kw4*kw4)/(mfc.u_mfc*mfc.u_mfc*kw3*kw3 + kw4*kw4);
      mfc.beta_min = (mfc.beta_min*kw4*kw4 - mfc.F_z_min*mfc.u_mfc*kw3*kw3 + kw3*kw3*yd_ddot_min*mfc.u_mfc)/(mfc.u_mfc*mfc.u_mfc*kw3*kw3 + kw4*kw4);

      // mfc.F_z_min = mfc.F_z_min_prev*(1.0f - lpf) + lpf*mfc.F_z_min;
      // mfc.beta_min = mfc.beta_min_prev*(1.0f - lpf) + lpf*mfc.beta_min;

      mfc.F_z_min_prev = mfc.F_z_min;
      mfc.beta_min_prev = mfc.beta_min;
      
    

      //======== Final Controller Calculations ========
      // Compute Acceleration Reference for yd^(v)
      /*
      There's two choices for computing this. With a trajectory planning a UAV has access to all derivitves of the reference setpoint. This
      is only true when we are sending position setpoints
      */

      // mfc.prev_ydot = yd_dot;
      // mfc.prev_yddot = yd_ddot;
      // mfc.prev_z_ref  = setpoint->position.z;
    
      // Control Effort to Thrust
      /*
      There was a design decision here to change the original MATLAB code with a double derivitive to use the acceleration setpoint as the property of differential
      flatness gives us the derivitives of our trajectory generation. A weird consequence is the setpoints don't seem to be too smooth which makes the double derivitive
      zero at some points but otherwise this works well. Need to investigate later
      */
      //Add an lpf at control output
      float yd_ddot = setpoint->acceleration.z; // Direct use from trajectory generation

      if(gainFlag == 1){
        mfc.u_mfc = (yd_ddot - mfc.u_c -  mfc.F.v[2])  /  mfc.beta_min;
        actuatorThrust = (-pwmToThrustB + sqrtf(pwmToThrustB * pwmToThrustB + 4.0f * pwmToThrustA * mfc.u_mfc)) / (2.0f * pwmToThrustA);
        actuatorThrust = constrain(actuatorThrust,0.0f, 0.9f)*UINT16_MAX; //This seems to always saturate, how do we not hit these bounds?
      }
      else
        mfc.u_mfc = constrain(mfc.u_mfc, 0.0f, 3.0f);
      if(setpoint->position.z < 0.06f && state->position.z < 0.06f){actuatorThrust = 0; return;}
    }
  }
  
  if (RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)) {
    // Switch between manual and automatic position control
    if (setpoint->mode.z == modeDisable) {
      actuatorThrust = setpoint->thrust;
    }
    if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable) {
      attitudeDesired.roll = setpoint->attitude.roll;
      attitudeDesired.pitch = setpoint->attitude.pitch;
    }

    attitudeControllerCorrectAttitudePID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw,
                                attitudeDesired.roll, attitudeDesired.pitch, attitudeDesired.yaw,
                                &rateDesired.roll, &rateDesired.pitch, &rateDesired.yaw);

    // For roll and pitch, if velocity mode, overwrite rateDesired with the setpoint
    // value. Also reset the PID to avoid error buildup, which can lead to unstable
    // behavior if level mode is engaged later
    if (setpoint->mode.roll == modeVelocity) {
      rateDesired.roll = setpoint->attitudeRate.roll;
      attitudeControllerResetRollAttitudePID();
    }
    if (setpoint->mode.pitch == modeVelocity) {
      rateDesired.pitch = setpoint->attitudeRate.pitch;
      attitudeControllerResetPitchAttitudePID();
    }

    // TODO: Investigate possibility to subtract gyro drift.
    attitudeControllerCorrectRatePID(sensors->gyro.x, -sensors->gyro.y, sensors->gyro.z,
                             rateDesired.roll, rateDesired.pitch, rateDesired.yaw);

    attitudeControllerGetActuatorOutput(&control->roll,
                                        &control->pitch,
                                        &control->yaw);

    control->yaw = -control->yaw;

    cmd_thrust = control->thrust;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;
    r_roll = radians(sensors->gyro.x);
    r_pitch = -radians(sensors->gyro.y);
    r_yaw = radians(sensors->gyro.z);
    accelz = sensors->acc.z;
  }
  control->thrust = actuatorThrust;
  // Some reset parameters 
  if (control->thrust == 0)
  {
    control->thrust = 0;
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;

    cmd_thrust = control->thrust;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;

    attitudeControllerResetAllPID();
    positionControllerResetAllPID();

    // Reset the calculated YAW angle for rate control
    attitudeDesired.yaw = state->attitude.yaw;
  }

  // Call the PID controller instead in this example to make it possible to fly
  // controllerPid(control, setpoint, sensors, state, tick);
}

//Logging Parameters
LOG_GROUP_START(mfcLogs)
LOG_ADD(LOG_FLOAT, posError, &posErrorLog)
LOG_ADD(LOG_FLOAT, velError, &velErrorLog)
LOG_ADD(LOG_FLOAT, d1, &mfc.prev_ydot)
LOG_ADD(LOG_FLOAT, d2, &yd_ddotLog)
LOG_ADD(LOG_FLOAT, F1, &mfc.F.v[0])
LOG_ADD(LOG_FLOAT, F2, &mfc.F.v[1])
LOG_ADD(LOG_FLOAT, F3, &mfc.F.v[2])
LOG_ADD(LOG_FLOAT, beta_min, &mfc.beta_min)
LOG_ADD(LOG_FLOAT, F_min, &mfc.F_z_min)
LOG_ADD(LOG_FLOAT, S1, &S[0])
LOG_ADD(LOG_FLOAT, S2, &S[1])
LOG_ADD(LOG_FLOAT, S3, &S[2])
LOG_ADD(LOG_FLOAT, u_mfc, &mfc.u_mfc)
LOG_ADD(LOG_FLOAT, u_mfc_PWM, &actuatorThrust)
LOG_ADD(LOG_FLOAT, u_c, &mfc.u_c)
LOG_ADD(LOG_FLOAT, P00, &mfc.P.m[0][0])
LOG_ADD(LOG_FLOAT, P11, &mfc.P.m[1][1])
LOG_ADD(LOG_FLOAT, P22, &mfc.P.m[2][2])
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)

//Some logs about running controllers in parallel
LOG_ADD(LOG_FLOAT, S31, &mfc.F3.x)
LOG_ADD(LOG_FLOAT, S32, &mfc.F3.y)
LOG_ADD(LOG_FLOAT, S33, &mfc.F3.z)
LOG_GROUP_STOP(mfcLogs)

PARAM_GROUP_START(mfcParams)
PARAM_ADD(PARAM_FLOAT, beta, &beta_z)
PARAM_ADD(PARAM_INT16, CTRL_RATE, &CTRL_RATE)
PARAM_ADD(PARAM_FLOAT, alpha, &lpf)
PARAM_ADD(PARAM_FLOAT, kp_z, &kp_z)
PARAM_ADD(PARAM_FLOAT, kd_z, &kd_z)
PARAM_GROUP_STOP(mfcParams)
