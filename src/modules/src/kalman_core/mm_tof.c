/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--'  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2021 Bitcraze AB
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
#include "math3d.h"
#include "mm_tof.h"
#include "usec_time.h"
#include "debug.h"
#include "log.h"
uint64_t start_time,end_time;

static float stateLog[3] = {0};
static float pLog[9] = {0};

void kalmanCoreUpdateWithTof(kalmanCoreData_t* this, tofMeasurement_t *tof)
{
  // Updates the filter with a measured distance in the zb direction using the
  float h[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

  // Only update the filter if the measurement is reliable (\hat{h} -> infty when R[2][2] -> 0)
  if (fabs(this->R[2][2]) > 0.1 && this->R[2][2] > 0){
    float angle = fabsf(acosf(this->R[2][2])) - DEG_TO_RAD * (15.0f / 2.0f);
    if (angle < 0.0f) {
      angle = 0.0f;
    }
    float predictedDistance = this->S[KC_STATE_Z] / cosf(angle);
    float measuredDistance = tof->distance; // [m]

    /*
    The sensor model (Pg.95-96, https://lup.lub.lu.se/student-papers/search/publication/8905295)
    
    h = z/((R*z_b).z_b) = z/cos(alpha)
    
    Here,
    h (Measured variable)[m] = Distance given by TOF sensor. This is the closest point from any surface to the sensor in the measurement cone
    z (Estimated variable)[m] = THe actual elevation of the crazyflie
    z_b = Basis vector in z direction of body coordinate system
    R = Rotation matrix made from ZYX Tait-Bryan angles. Assumed to be stationary
    alpha = angle between [line made by measured point <---> sensor] and [the intertial z-axis] 
    */

    h[KC_STATE_Z] = 1 / cosf(angle); // This just acts like a gain for the sensor model. Further updates are done in the scalar update function below

    start_time = usecTimestamp();
    // MFC Tester
    float S[3] = {this->S[0],this->S[1],this->S[2]};
    float HPHR_mfc = tof->stdDev*tof->stdDev;
    struct vec H_mfc = {0, 0, h[2]};
    struct mat33 I = meye();
    struct mat33 P_minus = {{{(float)this->Pm.pData[0], (float)this->Pm.pData[1], (float)this->Pm.pData[2]},
                              {(float)this->Pm.pData[3], (float)this->Pm.pData[4], (float)this->Pm.pData[5]}, 
                              {(float)this->Pm.pData[6], (float)this->Pm.pData[7], (float)this->Pm.pData[8]}}};
    struct vec PHT = mvmul(P_minus,H_mfc);
    float PHTarr[3] = {PHT.x,PHT.y,PHT.z};
    float Harr[3] = {H_mfc.x, H_mfc.y, H_mfc.z};

    for(int i = 0; i < 3; i++){
      HPHR_mfc += Harr[i]*PHTarr[i];
    }
    float try = 1.0f/HPHR_mfc;

    struct vec Kv = vscl(try, PHT);
    float K[3] = {Kv.x, Kv.y, Kv.z};
    float err = measuredDistance-predictedDistance;
    for(int i = 0; i < 3; i++){
          S[i] = S[i] + K[i]*err;
    }

    struct mat33 KH = mvecmult(Kv,H_mfc);
    struct mat33 IKH = msub(I,KH);
    struct mat33 P_plus = mmul(IKH,P_minus);
    end_time = usecTimestamp();
    // DEBUG_PRINT("Time Taken: %llu \n",end_time-start_time);

    stateLog[0] = S[0];
    stateLog[1] = S[1];
    stateLog[2] = S[2];
    pLog[0] = P_plus.m[0][0];
    pLog[1] = P_plus.m[0][1];
    pLog[2] = P_plus.m[0][2];
    pLog[3] = P_plus.m[1][0];
    pLog[4] = P_plus.m[1][1];
    pLog[5] = P_plus.m[1][2];
    pLog[6] = P_plus.m[2][0];
    pLog[7] = P_plus.m[2][1];
    pLog[8] = P_plus.m[2][2];

    // Scalar update
    kalmanCoreScalarUpdate(this, &H, measuredDistance-predictedDistance, tof->stdDev);
  }
}

LOG_GROUP_START(KFChecker)
LOG_ADD(LOG_FLOAT, state1, &stateLog[0])
LOG_ADD(LOG_FLOAT, state2, &stateLog[1])
LOG_ADD(LOG_FLOAT, state3, &stateLog[2])
LOG_ADD(LOG_FLOAT, P11, &pLog[0])
LOG_ADD(LOG_FLOAT, P12, &pLog[1])
LOG_ADD(LOG_FLOAT, P13, &pLog[2])
LOG_ADD(LOG_FLOAT, P21, &pLog[3])
LOG_ADD(LOG_FLOAT, P22, &pLog[4])
LOG_ADD(LOG_FLOAT, P23, &pLog[5])
LOG_ADD(LOG_FLOAT, P31, &pLog[6])
LOG_ADD(LOG_FLOAT, P32, &pLog[7])
LOG_ADD(LOG_FLOAT, P33, &pLog[8])
LOG_GROUP_STOP(KFChecker)