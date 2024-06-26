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
#include "log.h"
#include "math3d.h"
#include "cf_math.h"

#define DEBUG_MODULE "KFTest"
#include "debug.h"
#include "estimator.h"
#include "estimator_kalman.h"

void appMain() {
  DEBUG_PRINT("Estimator Enabled\n");

  while(1) {
    vTaskDelay(M2T(2000));

    // Remove the DEBUG_PRINT.
    // DEBUG_PRINT("Hello World!\n");
  }
}


void estimatorOutOfTreeInit() {
  // Initialize your controller data here...

  // Call the PID controller instead in this example to make it possible to fly
  estimatorKalmanInit();
}

bool estimatorOutOfTreeTest() {
  // Always return true
  return true;
}

void estimatorOutOfTree(state_t *state, const stabilizerStep_t stabilizerStep) {
  // Implement your controller here.
  // Call the PID controller instead in this example to make it possible to fly
  struct mat33 Q = mdiag(0.1f,0.1f,0.1f);

  static struct vec H = {1.0f, 0.0f, 0.0f};   
  float Harr[3] = {1.0f, 0.0f, 0.0f};

  struct mat33 A = {{{1.0f, DT_POS, DT_POS*DT_POS*0.5f},
                      {0.0f, 1.0f, DT_POS},
                      {0.0f, 0.0f, 1.0f}}};
  struct mat33 I = meye();

  // DEBUG_PRINT("Entered Position\n");
  //Preclared Vars
  float HPHR = 1e-6f; 
  struct mat33 P_plus_prev = mfc.P;

  // ============= Estimation o
  if (fabs(this->R[2][2]) > 0.1 && this->R[2][2] > 0){
    float angle = fabsf(acosf(this->R[2][2])) - DEG_TO_RAD * (15.0f / 2.0f);
    if (angle < 0.0f) {
      angle = 0.0f;
    }

  //State Prediction
  float statePredict = state->position.z/cosf(angle);
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

  mfc.P = P_plus;

  mfc.F.x = S[0];
  mfc.F.y = S[1];
  mfc.F.z = S[2];
  estimatorKalman(state, stabilizerStep);
}

LOG_GROUP_START(KFTest)

LOG_GROUP_STOP(KFTest)