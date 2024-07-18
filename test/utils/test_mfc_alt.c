#include "num.h"
#include <math3d.h>
#include "unity.h"
#include <stdio.h>
#include <stdlib.h>


// All values are form the 
//
//
//

// void testMatrix(){
//   // Fixture
//   struct mat33 I = meye();
//   float expected = 1.0f;

//   // Test
//   struct vec matReturn = mcolumn(I,0);
//   float actual[3] = {0.0f};
//   vstoref(matReturn, actual);
//   float value = actual[1];

//   // Assert
//   TEST_ASSERT_EQUAL_FLOAT(expected,value);
// }

void testStatePrediction(){
    //Fixture
    float state[3] = {0.0f};
    float expected[3] = {0.549919699137528f, 0.004792292302532f, -10.105618395579837f};
    float beta = 37.0f;
    float DT = 0.01;
    float x1 = 0.549871986891675f;
    float x2 = 0.004750156868237f;
    float x3 = -10.105618395579837f;
    float u = 0.273238701054309f;

    //Test
    state[0] = x1 + x2*DT + 0.5*x3*DT*DT + 0.5*beta*DT*DT*u;
    state[1] = x2 + x3*DT + beta*DT*u;
    state[2] = x3;

    //Assert
    TEST_ASSERT_FLOAT_WITHIN(0.001,expected[0],state[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.001,expected[1],state[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.001,expected[2],state[2]);
}

void testCovariancePrediction(){
    //Fixture
    float DT = 0.01;
    struct mat33 A = {{{1.0f, DT, DT*DT*0.5},{0.0f, 1.0f, DT},{0.0f, 0.0f, 1.0f}}};
    struct mat33 P_plus ={{{9.999901719052559e-7f,1.722069708304241e-6f, 9.913672606304270e-7f},{1.722069708291570e-6f, 17.370871904567103f,10.000126721266822f},{9.913672606322880e-07f, 10.000126721266822f,17.370653406420345f}}};
    struct mat33 Q = mdiag(0.1f,0.1f,0.1f);

    struct mat33 P_minus = {{{0.101748165274514f,0.175219155363945f,0.100870791250250f},{0.175219155363945f,17.672611504333084f,10.173833255331026f},{0.100870791250250f,10.173833255331026f,17.470653406420347f}}};

    //Test
    struct mat33 AT = mtranspose(A);
    struct mat33 PAT = mmul(P_plus, AT);
    struct mat33 APAT = mmul(A,PAT);
    struct mat33 APATQ = madd(APAT,Q);

    struct vec P_minusc1 = mcolumn(P_minus,0);
    struct vec P_minusc2 = mcolumn(P_minus,1);
    struct vec P_minusc3 = mcolumn(P_minus,2);

    struct vec P_plus1 = mcolumn(APATQ,0);
    struct vec P_plus2 = mcolumn(APATQ,1);
    struct vec P_plus3 = mcolumn(APATQ,2);

    //Assert
    TEST_ASSERT_FLOAT_WITHIN(0.001,P_minusc1.x,P_plus1.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001,P_minusc1.y,P_plus1.y);
    TEST_ASSERT_FLOAT_WITHIN(0.001,P_minusc1.z,P_plus1.z);

    TEST_ASSERT_FLOAT_WITHIN(0.001,P_minusc2.x,P_plus2.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001,P_minusc2.y,P_plus2.y);
    TEST_ASSERT_FLOAT_WITHIN(0.001,P_minusc2.z,P_plus2.z);

    TEST_ASSERT_FLOAT_WITHIN(0.001,P_minusc3.x,P_plus3.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001,P_minusc3.y,P_plus3.y);
    TEST_ASSERT_FLOAT_WITHIN(0.001,P_minusc3.z,P_plus3.z);
}

void testKalmanGain(){
    //Fixture
    float HPHR = 1e-6f;
    struct vec H = {1.0f, 0.0f, 0.0f};
    float Harr[3] = {1.0f, 0.0f, 0.0f};
    struct mat33 P_minus = {{{0.101748165274514f,0.175219155363945f,0.100870791250250f},{0.175219155363945f,17.672611504333084f,10.173833255331026f},{0.100870791250250f,10.173833255331026f,17.470653406420347f}}};
   
    struct vec K_expected = {0.999990171909545f, 1.722069708298957f, 0.991367260636541f};
    //Test
    struct vec PHT = mvmul(P_minus,H);
    float PHTarr[3] = {PHT.x, PHT.y, PHT.z};

    for(int i = 0; i < 3; i++){
        HPHR += Harr[i]*PHTarr[i];
    }

    float try = 1.0f/HPHR;
    
    struct vec K = vscl(try, PHT);

    //Assert
    TEST_ASSERT_FLOAT_WITHIN(0.001,K_expected.x ,K.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001,K_expected.y ,K.y);
    TEST_ASSERT_FLOAT_WITHIN(0.001,K_expected.z ,K.z);  
}

void testStateUpdate(){
    //Fixture
    float state[3] = {0.549919699137528f, 0.004792292302532f, -10.105618395579837f};
    float K[3] = {0.999990171909545f, 1.722069708298957f, 0.991367260636541f};
    float error = -0.001436003552351f;

    float state_expected[3] = {0.548483709698351f,0.002319394084019f, -10.107042002487795f};
    //Test
    for(int i = 0; i < 3; i++){
        state[i] = state[i] + K[i]*error;
    }

    //Assert
    TEST_ASSERT_FLOAT_WITHIN(0.001, state_expected[0], state[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.001, state_expected[1], state[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.001, state_expected[2], state[2]);
}

// void testCovarianceUpdate(){
//     //Fixture
//     struct mat33 I = meye();
//     struct vec K = {1.0f, 0.4288f, 0.0881f};
//     struct mat33 P_minus = {{{0.101748165274514f,0.175219155363945f,0.100870791250250f},{0.175219155363945f,17.672611504333084f,10.173833255331026f},{0.100870791250250f,10.173833255331026f,17.470653406420347f}}};
//     struct vec H = {1.0f, 0.0f, 0.0f};
    
//     struct mat33 P_minus_exp ={{{9.999901719052559e-7f,1.722069708304241e-6f, 9.913672606304270e-7f},{1.722069708291570e-6f, 17.370871904567103f,10.000126721266822f},{9.913672606322880e-07f, 10.000126721266822f,17.370653406420345f}}};
//     //Test
//     struct mat33 KH = mvecmult(K,H);
//     struct mat33 IKH = msub(I,KH);
//     struct mat33 P_plus = mmul(IKH,P_minus);

//     //Assert
//     struct vec P_plusc1 = mcolumn(P_plus,0);
//     struct vec P_plusc2 = mcolumn(P_plus,1);
//     struct vec P_plusc3 = mcolumn(P_plus,2);

//     struct vec P_minusc1 = mcolumn(P_minus_exp,0);
//     struct vec P_minusc2 = mcolumn(P_minus_exp,1);
//     struct vec P_minusc3 = mcolumn(P_minus_exp,2);

//     TEST_ASSERT_FLOAT_WITHIN(0.001,P_minusc1.x,P_plusc1.x);
//     TEST_ASSERT_FLOAT_WITHIN(0.001,P_minusc1.y,P_plusc1.y);
//     TEST_ASSERT_FLOAT_WITHIN(0.001,P_minusc1.z,P_plusc1.z);
//     TEST_ASSERT_FLOAT_WITHIN(0.001,P_minusc2.x,P_plusc2.x);
//     TEST_ASSERT_FLOAT_WITHIN(0.001,P_minusc2.y,P_plusc2.y);
//     TEST_ASSERT_FLOAT_WITHIN(0.001,P_minusc2.z,P_plusc2.z);
//     TEST_ASSERT_FLOAT_WITHIN(0.001,P_minusc3.x,P_plusc3.x);
//     TEST_ASSERT_FLOAT_WITHIN(0.001,P_minusc3.y,P_plusc3.y);
//     TEST_ASSERT_FLOAT_WITHIN(0.001,P_minusc3.z,P_plusc3.z);
// }

void testFullAlgo(){
    FILE *file; 
    float sense[1601];
    float control[1601];
    float num;
    int i = 0;
    /* Try Open and exit if there is problem */ 
    file = fopen("sensorvalues.txt", "r");
    /* Read and store EOF is always better */
    while( fscanf(file, "%f,", &num) != EOF ) {
        sense[i++] = num;
    }
    /* Done with File Close it */
    fclose(file);
    i = 0;
    /* Try Open and exit if there is problem */ 
    file = fopen("control.txt", "r");
    /* Read and store EOF is always better */
    while(fscanf(file, "%f,", &num) != EOF ) {
        control[i++] = num;
    }
    /* Done with File Close it */
    fclose(file);


    //Fixture
    struct vec S = {0.263032585590997f,  -0.528299162825243f,  -12.023052799826550f};
    // struct mat33 P = {{{0.000000999990171f, 0.000001714914530f, 0.000000985182944f},{0.000001714914530f, 17.301446590037816f,9.940032985616481f},{0.000000985182944f, 9.940032985616483f,17.301228410680164f}}};
    float state[3] = {0.0f};
    float state_expected[3] = {1.166331880996461f, 0.045182283807592f, -9.959444354561041f};
    float beta = 37.0f;
    float DT = (float)(1/100.0f);
    // float u = 0.250415523547456f;
    // float sense = 0.267879764688580f;
    struct mat33 P_plus = {{{0.000000999990171f, 0.000001714914530f, 0.000000985182944f},{0.000001714914530f, 17.299008381202395f,9.937923609961269f},{0.000000985182944f, 9.937923609961269f,17.299008381202395f}}};
    struct mat33 P_minus_exp = {{{0.0f, 0.0f, 0.0f},{0.0f,17.370871904567103f,10.000126721266826f},{0.0f, 10.000126721266826f,17.370653406420345f}}};
    float Harr[3] = {1.0f, 0.0f, 0.0f};
    struct vec H = {1.0f, 0.0f, 0.0f};   

    struct mat33 A = {{{1.0f, DT, DT*DT*0.5f},{0.0f, 1.0f, DT},{0.0f, 0.0f, 1.0f}}};
    struct mat33 Q = mdiag(0.1f,0.1f,0.1f);

    struct mat33 I = meye();

    for(int i = 0; i < 1600; i++){
        float HPHR = 1e-6f;
        state[0]= S.x + S.y*DT + 0.5f*S.z*DT*DT + 0.5f*beta*DT*DT*control[i];
        state[1] = S.y + S.z*DT + beta*DT*control[i];
        state[2] = S.z;
        // state[0]= state[0] + state[1]*DT + 0.5f*state[2]*DT*DT + 0.5f*beta*DT*DT*u;
        // state[1] = state[1]+ state[2]*DT + beta*DT*u;
        // state[2] = state[2];

        struct mat33 AT = mtranspose(A);
        struct mat33 PAT = mmul(P_plus, AT);
        struct mat33 APAT = mmul(A,PAT);
        struct mat33 P_minus = madd(APAT,Q);

        struct vec PHT = mvmul(P_minus,H);
        float PHTarr[3] = {PHT.x, PHT.y, PHT.z};

        for(int i = 0; i < 3; i++){
            HPHR += Harr[i]*PHTarr[i];
        }

        float try = 1.0f/HPHR;
        
        struct vec Kv = vscl(try, PHT);
        float K[3] = {Kv.x, Kv.y, Kv.z};


        float error = sense[i] - state[0];

        for(int i = 0; i < 3; i++){
            state[i] = state[i] + K[i]*error;
        }

        struct mat33 KH = mvecmult(Kv,H);
        struct mat33 IKH = msub(I,KH);
        // struct mat33 IKH = IKH_tmp;
        P_plus = mmul(IKH,P_minus);

        S.x = state[0];
        S.y = state[1];
        S.z = state[2];
    }
    // P = P_plus_calc;

    //Assert
    struct vec P_plusc1 = mcolumn(P_plus,0);
    struct vec P_plusc2 = mcolumn(P_plus,1);
    struct vec P_plusc3 = mcolumn(P_plus,2);

    struct vec P_minusc1 = mcolumn(P_minus_exp,0);
    struct vec P_minusc2 = mcolumn(P_minus_exp,1);
    struct vec P_minusc3 = mcolumn(P_minus_exp,2);

    TEST_ASSERT_FLOAT_WITHIN(0.001,P_minusc1.x,P_plusc1.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001,P_minusc1.y,P_plusc1.y);
    TEST_ASSERT_FLOAT_WITHIN(0.001,P_minusc1.z,P_plusc1.z);
    TEST_ASSERT_FLOAT_WITHIN(0.001, P_minusc2.x,P_plusc2.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001,P_minusc2.y,P_plusc2.y);
    TEST_ASSERT_FLOAT_WITHIN(0.001,P_minusc2.z,P_plusc2.z);
    TEST_ASSERT_FLOAT_WITHIN(0.001,P_minusc3.x,P_plusc3.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001,P_minusc3.y,P_plusc3.y);
    TEST_ASSERT_FLOAT_WITHIN(0.001,P_minusc3.z,P_plusc3.z);

    TEST_ASSERT_FLOAT_WITHIN(0.001, state_expected[0], S.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001, state_expected[1], S.y);
    TEST_ASSERT_FLOAT_WITHIN(0.001, state_expected[2], S.z);
}

void testFullAlgoComplexTraj(){
    FILE *file; 
    float sense[29999];
    // float control[9991];
    float ref[29999];
    float u_z[29999];
    float num;
    int i = 0;
    /* Try Open and exit if there is problem */ 
    file = fopen("senseCTraj.txt", "r");
    /* Read and store EOF is always better */
    while( fscanf(file, "%f,", &num) != EOF ) {
        sense[i++] = num;
    }
    /* Done with File Close it */
    fclose(file);

    i = 0;
    /* Try Open and exit if there is problem */ 
    file = fopen("ref_z.txt", "r");
    /* Read and store EOF is always better */
    while( fscanf(file, "%f,", &num) != EOF ) {
        ref[i++] = num;
    }
    /* Done with File Close it */
    fclose(file);

    i = 0;
    /* Try Open and exit if there is problem */ 
    file = fopen("uz.txt", "r");
    /* Read and store EOF is always better */
    while( fscanf(file, "%f,", &num) != EOF ) {
        u_z[i++] = num;
    }
    /* Done with File Close it */
    fclose(file);
    // i = 0;
    // /* Try Open and exit if there is problem */ 
    // file = fopen("controlCTraj.txt", "r");
    // /* Read and store EOF is always better */
    // while(fscanf(file, "%f,", &num) != EOF ) {
    //     control[i++] = num;
    // }
    // /* Done with File Close it */
    // fclose(file);


    //Fixture
    struct vec S = {0.001040782171872f, 1.040823801701275e-10f,  5.203858815565599e-13f};
    // struct mat33 P = {{{0.000000999990171f, 0.000001714914530f, 0.000000985182944f},{0.000001714914530f, 17.301446590037816f,9.940032985616481f},{0.000000985182944f, 9.940032985616483f,17.301228410680164f}}};
    float state[3] = {0.0f};
    float state_expected[3] = {-4.522077392752259e-04f, -0.213016516030242f, -9.965021161651567f};
    float beta = 37.0f;
    float DT = (float)(1/100.0f);
    // float u = 0.250415523547456f;
    // float sense = 0.267879764688580f;
    struct mat33 P_plus = {{{9.999900082020056e-07f, 1.000029998405346e-13f, 4.999899997026880e-16f},{1.000029998404189e-13f, 0.100001000099999f,9.999999994999850e-09f},{4.999899996944139e-16f, 9.999999994999850e-09f,0.100001000000000f}}};
    struct mat33 P_minus_exp = {{{9.999901719052559e-07f,1.722069708291570e-06f, 9.913672606322885e-07f},{1.722069708304241e-06f,17.370871904567103f,10.000126721266826f},{9.913672606304270e-07f, 10.000126721266826f,17.370653406420345f}}};
    float Harr[3] = {1.0f, 0.0f, 0.0f};
    struct vec H = {1.0f, 0.0f, 0.0f};   

    struct mat33 A = {{{1.0f, DT, DT*DT*0.5f},{0.0f, 1.0f, DT},{0.0f, 0.0f, 1.0f}}};
    struct mat33 Q = mdiag(0.1f,0.1f,0.1f);

    struct mat33 I = meye();
    float in_dif = 0;
    float control = 0;

    for(int j = 1; j < 29999; j++){
        float HPHR = 1e-6f;
        state[0]= S.x + S.y*DT + 0.5f*S.z*DT*DT + 0.5f*beta*DT*DT*control;
        state[1] = S.y + S.z*DT + beta*DT*control;
        state[2] = S.z;
        // state[0]= state[0] + state[1]*DT + 0.5f*state[2]*DT*DT + 0.5f*beta*DT*DT*u;
        // state[1] = state[1]+ state[2]*DT + beta*DT*u;
        // state[2] = state[2];

        struct mat33 AT = mtranspose(A);
        struct mat33 PAT = mmul(P_plus, AT);
        struct mat33 APAT = mmul(A,PAT);
        struct mat33 P_minus = madd(APAT,Q);

        struct vec PHT = mvmul(P_minus,H);
        float PHTarr[3] = {PHT.x, PHT.y, PHT.z};

        for(int i = 0; i < 3; i++){
            HPHR += Harr[i]*PHTarr[i];
        }

        float try = 1.0f/HPHR;
        
        struct vec Kv = vscl(try, PHT);
        float K[3] = {Kv.x, Kv.y, Kv.z};


        float error = sense[j-1] - state[0];

        for(int i = 0; i < 3; i++){
            state[i] = state[i] + K[i]*error;
        }

        struct mat33 KH = mvecmult(Kv,H);
        struct mat33 IKH = msub(I,KH);
        // struct mat33 IKH = IKH_tmp;
        P_plus = mmul(IKH,P_minus);

        S.x = state[0];
        S.y = state[1];
        S.z = state[2];

        float dif_t1 = (ref[j] - ref[j-1])/DT;
        float dif_t2 = (dif_t1 - in_dif)/DT;
        in_dif = dif_t1;
        control = (dif_t2 - u_z[j-1] - S.z)/beta;   
    }
    // P = P_plus_calc;

    //Assert
    struct vec P_plusc1 = mcolumn(P_plus,0);
    struct vec P_plusc2 = mcolumn(P_plus,1);
    struct vec P_plusc3 = mcolumn(P_plus,2);

    struct vec P_minusc1 = mcolumn(P_minus_exp,0);
    struct vec P_minusc2 = mcolumn(P_minus_exp,1);
    struct vec P_minusc3 = mcolumn(P_minus_exp,2);



    TEST_ASSERT_FLOAT_WITHIN(0.001,P_minusc1.x,P_plusc1.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001,P_minusc1.y,P_plusc1.y);
    TEST_ASSERT_FLOAT_WITHIN(0.001,P_minusc1.z,P_plusc1.z);
    TEST_ASSERT_FLOAT_WITHIN(0.001, P_minusc2.x,P_plusc2.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001,P_minusc2.y,P_plusc2.y);
    TEST_ASSERT_FLOAT_WITHIN(0.001,P_minusc2.z,P_plusc2.z);
    TEST_ASSERT_FLOAT_WITHIN(0.001,P_minusc3.x,P_plusc3.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001,P_minusc3.y,P_plusc3.y);
    TEST_ASSERT_FLOAT_WITHIN(0.001,P_minusc3.z,P_plusc3.z);

    TEST_ASSERT_FLOAT_WITHIN(0.001, state_expected[0], S.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001, state_expected[1], S.y);
    TEST_ASSERT_FLOAT_WITHIN(0.001, state_expected[2], S.z);
}

void testMFCControlCalc(){
    //Fixture
    float dt = 0.01f;
    float ref = 0.548059401999999f;
    float ref_prev = 0.548072447999999f;
    float in_dif = -0.001300200000010f;
    float uz = -0.002381320487904f;
    float F = -10.117838267338007f;
    float beta = 37.0f;

    float expected = 0.273507556427764f;
    //Test
    float dif_t1 = (ref - ref_prev)/dt;
    float dif_t2 = (dif_t1 - in_dif)/dt;
    float u = (dif_t2 - uz - F)/beta;

    //Assert
    TEST_ASSERT_FLOAT_WITHIN(0.00001, expected,u);
}   
// // ============= Kalman Estimation (F) =======
//       // Initalize the vectors and matrices needed
//        struct mat33 P =
//         {{{mfc.P[0][0], mfc.P[0][1],mfc.P[0][2]},
//           {mfc.P[1][0], mfc.P[1][1],mfc.P[1][2]},
//           {mfc.P[2][0], mfc.P[2][1],mfc.P[2][2]}}};

//       struct vec H =  mkvec(1.0f, 0.0f, 0.0f);
//       float Hd[3] = {H.x, H.y, H.z};
//       float K[3] = {0.0f, 0.0f, 0.0f};

//       // ------------- Time Update: Prediction -------------
//       //Step 1: State Prediction 
//       // xhat.x = mfc.S[0] + mfc.S[1]*DT + mfc.S[2]*DT*DT*0.5f + 0.5f*mfc.beta_z*DT*DT*mfc.u_mfc;
//       // xhat.y = mfc.S[1] + mfc.S[2]*DT + mfc.beta_z*DT*mfc.u_mfc;
//       // xhat.z = mfc.S[2]; // Other ways to do this but this is more readable
//       x_error[0] = mfc.S[0] + mfc.S[1]*DT + mfc.S[2]*DT*DT*0.5f + 0.5f*mfc.beta_z*DT*DT*mfc.u_mfc;
//       x_error[1] = mfc.S[1] + mfc.S[2]*DT + mfc.beta_z*DT*mfc.u_mfc;
//       x_error[2] = mfc.S[2];

//       //Step 2: Covariance Error Prediction
//       // Compute F * P * F' + Q
//       struct mat33 FT = mtranspose(F);
//       struct mat33 PFT = mmul(P,FT);
//       P = mmul(F,PFT);
//       //Add some process noise
//       P = madd(P, Q);

//       // ------------- Measurement Update: Correction -------------
//       //Very cheaty way but currently H = [1 0 0] so itll only take the first value of P_z
//       struct vec PHTt = mcolumn(P,0);
//       float PHTd[3] = {PHTt.x, PHTt.y, PHTt.z};
//       float HPHR = R; //Initalize the scalar division variable to the measurement noise

//       // Sequential Kalman Filter - Updates Scalarly
//       // Compute H * P * H' + R
//       for (int i = 0; i < 3; i++){
//         HPHR += Hd[i]*PHTd[i];
//       }

//       //Compute Innovation H * X_hat
//       float error = state->position.z - x_error[0]; //This is the sensor for the z direction - first element in the xhat

//       for (int i = 0; i < 3; i++) {
//         K[i] = PHTd[i]/HPHR;               //Step 3: Compute Kalman Gain
//         x_error[i] = x_error[i] + K[i]*error;  //Step 4: Update State Estimate with Measurement
//       }

//       mfc.S[0] = x_error[0];
//       mfc.S[1] = x_error[1];
//       mfc.S[2] = x_error[2];

//       //Step 5: Update Covariance with Measurement
//       struct vec Kv = vloadf(K); //vloadf loads a vector from a float array
//       struct mat33 KH = mvecmult(Kv, H); // KH
//       struct mat33 tmp1 = msub(I,KH); // I - KH
//       struct mat33 tmp2 = mtranspose(tmp1); // (I - KH)'
//       struct mat33 tmp3 = mmul(tmp1,P); // (I - KH) * P
//       P = mmul(tmp3, tmp2); // (I - KH) * P * (I - KH)'

//       struct vec Ptmp1 = mcolumn(P,0);
//       struct vec Ptmp2 = mcolumn(P,1);
//       struct vec Ptmp3 = mcolumn(P,2);

//       mfc.P[0][0] = Ptmp1.x;
//       mfc.P[1][0] = Ptmp1.y;
//       mfc.P[2][0] = Ptmp1.z;

//       mfc.P[0][1] = Ptmp2.x;
//       mfc.P[1][1] = Ptmp2.y;
//       mfc.P[2][1] = Ptmp2.z;

//       mfc.P[0][2] = Ptmp3.x;
//       mfc.P[1][2] = Ptmp3.y;
//       mfc.P[2][2] = Ptmp3.z;

//       //Enforce PSD
//       // for (int i = 0; i < 3; i++)
//       // {
//       //   for (int j = i; j < 3; j++)
//       //   {
//       //     float p = 0.5f*mfc.P[i][j] + 0.5f*mfc.P[j][i];
//       //     if (i == j && p < 1e-6f) {
//       //       mfc.P[i][j] = 1e-6f;
//       //     }
//       //     else{
//       //       mfc.P[i][j] = mfc.P[j][i] = p;
//       //     }
//       //   }
//       // }
