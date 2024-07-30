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

#define DEBUG_MODULE "MFC_CONTROLLER_POS"
#include "debug.h"
// #include "controller.h"
// #include "controller_pid.h"
#include "math3d.h"
#include "mfc_controller_pos.h"
#include "position_controller.h"
#include "attitude_controller.h"
#include "log.h"
#include "physicalConstants.h"
#include "platform_defaults.h"
#include "num.h"
#include "param.h"


//Controller Gains
const static int CTRL_RATE = 360;
float ATTITUDE_UPDATE_DT = (float)(1.0f/ATTITUDE_RATE);
static float DT_POS = (float)(1.0f/CTRL_RATE);
float state_body_x, state_body_y, state_body_vx, state_body_vy;
static float pwmToThrustA = 0.091492681f;
static float pwmToThrustB = 0.067673604f;


static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float actuatorThrust;
static float actuatorThrustMFC;
static float lpf;
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

static struct mfc_Variables_s mfc_x = {
  .F.x = 0.0f,
  .F.y = 0.0f,
  .F.z = -1e-6f,
  .P.m[0][0] = 1e-6f,
  .P.m[1][1] = 1e-6f,
  .P.m[2][2] = 1e-6f,
  .u_mfc = 0.0f,
  .prev_ydot = 0.0f,
  .prev_yddot = 0.0f,
  .prev_vel_error = 0.0f,
  .prev_pos_error = 0.0f,
  .u_c = 0.0f,
  .kp = 51.446f,
  .kd = 14.0f,
  .beta = 130.0f,
  .flag = 1,
};

static struct mfc_Variables_s mfc_y = {
  .F.x = 0.0f,
  .F.y = 0.0f,
  .F.z = -1e-6f,
  .P.m[0][0] = 1e-6f,
  .P.m[1][1] = 1e-6f,
  .P.m[2][2] = 1e-6f,
  .u_mfc = 0.0f,
  .prev_ydot = 0.0f,
  .prev_yddot = 0.0f,
  .prev_vel_error = 0.0f,
  .prev_pos_error = 0.0f,
  .u_c = 0.0f,
  .kp = 51.446f,
  .kd = 14.0f,
  .beta = 140.0f,
  .flag = 2.
};

static struct mfc_Variables_s mfc_z = {
  .F.x = 0.0f,
  .F.y = 0.0f,
  .F.z = -1e-6f,
  .P.m[0][0] = 1e-6f,
  .P.m[1][1] = 1e-6f,
  .P.m[2][2] = 1e-6f,
  .u_mfc = 0.0f,
  .prev_ydot = 0.0f,
  .prev_yddot = 0.0f,
  .prev_vel_error = 0.0f,
  .prev_pos_error = 0.0f,
  .u_c = 0.0f,
  .kp = 31.0f,
  .kd = 16.0f,
  .beta = 47.0f,
  .flag = 3,
};






static struct mfc_Variables_s EmptyStruct ={
  .F.x = 0.0f,
  .F.y = 0.0f,
  .F.z = -1e-6f,
  .P.m[0][0] = 1e-6f,
  .P.m[1][1] = 1e-6f,
  .P.m[2][2] = 1e-6f,
  .u_mfc = 0.0f,
  .prev_ydot = 0.0f,
  .prev_yddot = 0.0f,
  .prev_vel_error = 0.0f,
  .prev_pos_error = 0.0f,
  .u_c = 0.0f,
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


void mfcParamReset(){
  mfc_z = EmptyStruct;
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
}

bool controllerOutOfTreeTest() {
  bool pass = true;

  pass &= attitudeControllerTest();

  return pass;
}

void linearKF(mfc_Variables_t *mfc, const state_t *state){
   // =============== Estimation of F ===============
      //Predeclared Constant Matrices
      float S[3] = {0};
      struct mat33 Q = mdiag(0.1f,0.1f,0.1f);
      static struct vec H = {1.0f, 0.0f, 0.0f};   
      float Harr[3] = {1.0f, 0.0f, 0.0f};
      struct mat33 A = {{{1.0f, DT_POS, DT_POS*DT_POS*0.5f},
                         {0.0f, 1.0f, DT_POS},
                         {0.0f, 0.0f, 1.0f}}};
      struct mat33 I = meye();
      float HPHR = 0.0025f*0.0025f; //This is from Bitcraze themselves. The actual stdDev is a function but it varies from 0.0025:0.003 between 0m and 1m
      struct mat33 P_plus_prev = mfc->P;

      // start_time = usecTimestamp();
      //State Prediction
      S[0] = mfc->F.x + mfc->F.y*DT_POS + 0.5f*mfc->F.z*DT_POS*DT_POS + 0.5f*mfc->beta*DT_POS*DT_POS*mfc->u_mfc;
      S[1] = mfc->F.y + mfc->F.z*DT_POS + mfc->beta*DT_POS*mfc->u_mfc;
      S[2] = mfc->F.z;

      //Covariance Prediction
      struct mat33 AT = mtranspose(A);
      struct mat33 PAT = mmul(P_plus_prev, AT);
      struct mat33 APAT = mmul(A,PAT);
      struct mat33 P_minus = madd(APAT,Q);

      struct vec PHT = mvmul(P_minus,H);
      float PHTarr[3] = {PHT.x, PHT.y, PHT.z};

      //Helper for Scalar Update
      for(int i = 0; i < 3; i++){
          HPHR += Harr[i]*PHTarr[i];
      }
      float try = 1.0f/HPHR;
      
      //Calculate Kalman Gain/Load into Float for Looping
      struct vec Kv = vscl(try, PHT);
      float K[3] = {Kv.x, Kv.y, Kv.z};
      float F_err = 0.0f;
      if(mfc->flag == 1){F_err = state->position.x - S[0];}
      else if(mfc->flag == 2){F_err = state->position.y - S[0];}
      else{
        F_err = state->position.z - S[0];
      }
    
      //State Measurement Update
      for(int i = 0; i < 3; i++){
          S[i] = S[i] + K[i]*F_err;
      }

      //Covariance Measurement Update
      struct mat33 KH = mvecmult(Kv,H);
      struct mat33 IKH = msub(I,KH);
      struct mat33 P_plus = mmul(IKH,P_minus);

      //Enforce Covariance Boundaries
      for(int i = 0; i < 3; ++i){
        for(int j = i; j < 3;++j){
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

      mfc->P = P_plus;
      mfc->F.x = S[0];
      mfc->F.y = S[1];
      mfc->F.z = S[2];
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
    positionController(&actuatorThrust, &attitudeDesired, setpoint, state);
    actuatorThrust = 0.0f;
  }

  if(RATE_DO_EXECUTE(CTRL_RATE, stabilizerStep)) {
    if (setpoint->mode.x == modeAbs || setpoint->mode.y == modeAbs || setpoint->mode.z == modeAbs){

      // =============== MFC Controller ==================
      // =============== Internal Controllers ==================
      struct vec posErr = {state->position.x - setpoint->position.x, state->position.y - setpoint->position.y, state->position.z - setpoint->position.z};
      struct vec velErr = {state->velocity.x - setpoint->velocity.x,state->velocity.y - setpoint->velocity.y, state->velocity.z - setpoint->velocity.z};
      struct vec uc = {mfc_x.kp*posErr.x + mfc_x.kd*velErr.x, mfc_y.kp*posErr.y + mfc_y.kd*velErr.y, mfc_z.kp*posErr.z + mfc_z.kd*velErr.z};

      // =============== Estimation of F ===============
      // linearKF(&mfc_x,state);
      // linearKF(&mfc_y,state);
      
         // =============== Estimation of F ===============
      //Predeclared Constant Matrices
      float S[3] = {0};
      struct mat33 Q = mdiag(0.1f,0.1f,0.1f);
      static struct vec H = {1.0f, 0.0f, 0.0f};   
      float Harr[3] = {1.0f, 0.0f, 0.0f};
      struct mat33 A = {{{1.0f, DT_POS, DT_POS*DT_POS*0.5f},
                         {0.0f, 1.0f, DT_POS},
                         {0.0f, 0.0f, 1.0f}}};
      struct mat33 I = meye();
      float HPHR = 0.0025f*0.0025f; //This is from Bitcraze themselves. The actual stdDev is a function but it varies from 0.0025:0.003 between 0m and 1m
      struct mat33 P_plus_prev = mfc_z.P;

      // start_time = usecTimestamp();
      //State Prediction
      S[0] = mfc_z.F.x + mfc_z.F.y*DT_POS + 0.5f*mfc_z.F.z*DT_POS*DT_POS + 0.5f*mfc_z.beta*DT_POS*DT_POS*mfc_z.u_mfc;
      S[1] = mfc_z.F.y + mfc_z.F.z*DT_POS + mfc_z.beta*DT_POS*mfc_z.u_mfc;
      S[2] = mfc_z.F.z;

      //Covariance Prediction
      struct mat33 AT = mtranspose(A);
      struct mat33 PAT = mmul(P_plus_prev, AT);
      struct mat33 APAT = mmul(A,PAT);
      struct mat33 P_minus = madd(APAT,Q);

      struct vec PHT = mvmul(P_minus,H);
      float PHTarr[3] = {PHT.x, PHT.y, PHT.z};

      //Helper for Scalar Update
      for(int i = 0; i < 3; i++){
          HPHR += Harr[i]*PHTarr[i];
      }
      float try = 1.0f/HPHR;
      
      //Calculate Kalman Gain/Load into Float for Looping
      struct vec Kv = vscl(try, PHT);
      float K[3] = {Kv.x, Kv.y, Kv.z};
      float F_err = state->position.z - S[0];
    
      //State Measurement Update
      for(int i = 0; i < 3; i++){
          S[i] = S[i] + K[i]*F_err;
      }

      //Covariance Measurement Update
      struct mat33 KH = mvecmult(Kv,H);
      struct mat33 IKH = msub(I,KH);
      struct mat33 P_plus = mmul(IKH,P_minus);

      //Enforce Covariance Boundaries
      for(int i = 0; i < 3; ++i){
        for(int j = i; j < 3;++j){
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

      mfc_z.P = P_plus;
      mfc_z.F.x = S[0];
      mfc_z.F.y = S[1];
      mfc_z.F.z = S[2];


      // ======== Final Controller Calculations ========
      // Compute Acceleration Reference for yd^(v)

      // float yd_ddot_x = setpoint->acceleration.x; // Direct use from trajectory generation
      // float yd_ddot_y = setpoint->acceleration.y; // Direct use from trajectory generation
      float yd_ddot_z = setpoint->acceleration.z; // Direct use from trajectory generation
    

      // ======== Thrust Control (Z-Direction) ========
      // Control Effort to Thrust
      /*
      There was a design decision here to change the original MATLAB code with a double derivitive to use the acceleration setpoint as the property of differential
      flatness gives us the derivitives of our trajectory generation. A weird consequence is the setpoints don't seem to be too smooth which makes the double derivitive
      zero at some points but otherwise this works well. Need to investigate later
      */
      mfc_z.u_mfc = (yd_ddot_z - uc.z - mfc_z.F.z)  / mfc_z.beta;
      mfc_z.u_mfc = constrain((yd_ddot_z - uc.z - mfc_z.F.z)  / mfc_z.beta, 0.0f, 3.0f);
      if(setpoint->position.z < 0.06f && state->position.z < 0.06f){actuatorThrustMFC = 0; return;}
      else{
        actuatorThrustMFC = (-pwmToThrustB + sqrtf(pwmToThrustB * pwmToThrustB + 4.0f * pwmToThrustA * mfc_z.u_mfc)) / (2.0f * pwmToThrustA);
        actuatorThrustMFC = constrain(actuatorThrustMFC,0.0f, 0.9f)*UINT16_MAX; //This seems to always saturate, how do we not hit these bounds?
      }


      //X Y Control
      // World to Body Transforms
      // float cosyaw = cosf(state->attitude.yaw * (float)M_PI / 180.0f);
      // float sinyaw = sinf(state->attitude.yaw * (float)M_PI / 180.0f);


      // mfc_x.u_mfc = (yd_ddot_x - uc.x - mfc_x.F.z) / mfc_x.beta;
      // mfc_y.u_mfc = (yd_ddot_y - uc.y - mfc_y.F.z) / mfc_y.beta;

      // float gVx = (sinyaw*mfc_x.u_mfc - cosyaw*mfc_y.u_mfc) / mfc_z.u_mfc;
      // float gVy = (cosyaw*mfc_x.u_mfc + sinyaw*mfc_y.u_mfc) / mfc_z.u_mfc;

      // attitudeDesired.roll = asinf(gVx);
      // attitudeDesired.pitch = asinf(gVy/attitudeDesired.roll);
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
  control->thrust = actuatorThrustMFC;
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
    mfcParamReset();

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
LOG_ADD(LOG_FLOAT, d1, &mfc_z.prev_ydot)
LOG_ADD(LOG_FLOAT, d2, &mfc_z.prev_yddot)
LOG_ADD(LOG_FLOAT, F1, &mfc_z.F.x)
LOG_ADD(LOG_FLOAT, F2, &mfc_z.F.y)
LOG_ADD(LOG_FLOAT, F3, &mfc_z.F.z)
LOG_ADD(LOG_FLOAT, u_mfc, &mfc_z.u_mfc)
LOG_ADD(LOG_FLOAT, PID_PWM_Thrust, &actuatorThrust)
LOG_ADD(LOG_FLOAT, u_mfc_PWM, &actuatorThrustMFC)
LOG_ADD(LOG_FLOAT, u_c, &mfc_z.u_c)
LOG_ADD(LOG_FLOAT, P00, &mfc_z.P.m[0][0])
LOG_ADD(LOG_FLOAT, P11, &mfc_z.P.m[1][1])
LOG_ADD(LOG_FLOAT, P22, &mfc_z.P.m[2][2])
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
LOG_GROUP_STOP(mfcLogs)

LOG_GROUP_START(mfcX)
LOG_ADD(LOG_FLOAT, d2, &mfc_x.prev_yddot)
LOG_ADD(LOG_FLOAT, F1, &mfc_x.F.x)
LOG_ADD(LOG_FLOAT, F2, &mfc_x.F.y)
LOG_ADD(LOG_FLOAT, F3, &mfc_x.F.z)
LOG_ADD(LOG_FLOAT, u_mfc, &mfc_x.u_mfc)
LOG_ADD(LOG_FLOAT, u_c, &mfc_x.u_c)
LOG_GROUP_STOP(mfcX)

LOG_GROUP_START(mfcY)
LOG_ADD(LOG_FLOAT, d2, &mfc_y.prev_yddot)
LOG_ADD(LOG_FLOAT, F1, &mfc_y.F.x)
LOG_ADD(LOG_FLOAT, F2, &mfc_y.F.y)
LOG_ADD(LOG_FLOAT, F3, &mfc_y.F.z)
LOG_ADD(LOG_FLOAT, u_mfc, &mfc_y.u_mfc)
LOG_ADD(LOG_FLOAT, u_c, &mfc_y.u_c)
LOG_GROUP_STOP(mfcY)

LOG_GROUP_START(mfcZ)
LOG_ADD(LOG_FLOAT, d2, &mfc_z.prev_yddot)
LOG_ADD(LOG_FLOAT, F1, &mfc_z.F.x)
LOG_ADD(LOG_FLOAT, F2, &mfc_z.F.y)
LOG_ADD(LOG_FLOAT, F3, &mfc_z.F.z)
LOG_ADD(LOG_FLOAT, u_mfc, &mfc_z.u_mfc)
LOG_ADD(LOG_FLOAT, PID_PWM_Thrust, &actuatorThrust)
LOG_ADD(LOG_FLOAT, u_c, &mfc_z.u_c)
LOG_ADD(LOG_FLOAT, P00, &mfc_z.P.m[0][0])
LOG_ADD(LOG_FLOAT, P11, &mfc_z.P.m[1][1])
LOG_ADD(LOG_FLOAT, P22, &mfc_z.P.m[2][2])
LOG_GROUP_STOP(mfcZ)

PARAM_GROUP_START(mfcParams)
PARAM_ADD(PARAM_FLOAT, beta, &mfc_z.beta)
PARAM_ADD(PARAM_INT16, CTRL_RATE, &CTRL_RATE)
PARAM_ADD(PARAM_FLOAT, alpha, &lpf)
PARAM_GROUP_STOP(mfcParams)
