#include "unity.h"
#include "num.h"
#include "math3d.h"
#include <time.h>
#include "usec_time.h"

// void testDecomp() {
//   // Fixture
//   struct mat77 A = {{{2.0f,0.002f,0.0f,0.0f,1.0f,0.0f,0.0f},
//                      {0.002f,2.0f,0.002f,0.0082f,0.0f,1.0f,0.0f},
//                      {0.0f,0.002f,2.0f,0.0f,0.0f,0.0f,1.0f},
//                      {0.0f,0.0082f,0.0f,2.0f,0.0f,0.0f,4.1172f},
//                      {1.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f},
//                      {0.0f,1.0f,0.0f,0.0f,0.0f,0.0f,0.0f},
//                      {0.0f,0.0f,1.0f,4.1172f,0.0f,0.0f,0.0f}}};


//   struct mat77 LUExp = {{{2.0f,0.002f,0.0f,0.0f,1.0f,0.0f,0.0f},
//                         {0.001f,2.0f,0.002f,0.0082f,-0.001f,1.0f,0.0f},
//                         {0.0f,0.001f,2.0f,0.0f,0.0f,-0.001f,1.0f},
//                         {0.0f,0.0041f,0.0f,2.0f,0.0f,-0.0041f,4.1172f},
//                         {0.5f,-0.0005f,0.0f,0.0f,-0.5f,0.0005f,0.0f},
//                         {0.0f,0.5f,-0.0005f,-0.0021f,-0.001f,-0.5f,0.009f},
//                         {0.0f,0.0f,0.5f,2.0586f,0.0f,-0.0018f,-8.9755f}}};

//   struct vec7 b = mkvec7(2.0085f,-0.0288f,-7.6796f,1.9875f,1.0042f,0.2156f,0.6512f);
//   struct vec7 bExp = mkvec7(1.0042f, 0.2156f, -3.8175f,1.0854f, -0.004f,-0.4633f,-0.0449f);
//   // Test
//   int P[7] = {0};
//   struct mat77 LU = doolittle(A, P);
//   struct vec7 sol = linearSolve(LU, b, P);
//   for(int i = 0; i < 7; i++)
//     printf("The value of solution vector is is: %f\n ",b.v[i]);


//   // Assert
//   TEST_ASSERT_FLOAT_WITHIN(0.0001f, LUExp.m[0][0], LU.m[0][0]);
//   TEST_ASSERT_FLOAT_WITHIN(0.0001f, LUExp.m[0][1], LU.m[0][1]);
//   TEST_ASSERT_FLOAT_WITHIN(0.0001f, LUExp.m[0][2], LU.m[0][2]);
//   TEST_ASSERT_FLOAT_WITHIN(0.0001f, LUExp.m[0][3], LU.m[0][3]);
//   TEST_ASSERT_FLOAT_WITHIN(0.0001f, LUExp.m[0][4], LU.m[0][4]);
//   TEST_ASSERT_FLOAT_WITHIN(0.0001f, LUExp.m[0][5], LU.m[0][5]);


//   TEST_ASSERT_FLOAT_WITHIN(0.0001f, LUExp.m[1][0], LU.m[1][0]);
//   TEST_ASSERT_FLOAT_WITHIN(0.0001f, LUExp.m[1][1], LU.m[1][1]);
//   TEST_ASSERT_FLOAT_WITHIN(0.0001f, LUExp.m[1][2], LU.m[1][2]);
//   TEST_ASSERT_FLOAT_WITHIN(0.0001f, LUExp.m[1][3], LU.m[1][3]);
//   TEST_ASSERT_FLOAT_WITHIN(0.0001f, LUExp.m[1][4], LU.m[1][4]);
//   TEST_ASSERT_FLOAT_WITHIN(0.0001f, LUExp.m[1][5], LU.m[1][5]);

//   TEST_ASSERT_FLOAT_WITHIN(0.0001f, LUExp.m[2][0], LU.m[2][0]);
//   TEST_ASSERT_FLOAT_WITHIN(0.0001f, LUExp.m[2][1], LU.m[2][1]);
//   TEST_ASSERT_FLOAT_WITHIN(0.0001f, LUExp.m[2][2], LU.m[2][2]);
//   TEST_ASSERT_FLOAT_WITHIN(0.0001f, LUExp.m[2][3], LU.m[2][3]);
//   TEST_ASSERT_FLOAT_WITHIN(0.0001f, LUExp.m[2][4], LU.m[2][4]);
//   TEST_ASSERT_FLOAT_WITHIN(0.0001f, LUExp.m[2][5], LU.m[2][5]);

//   TEST_ASSERT_FLOAT_WITHIN(0.0001f, LUExp.m[4][0], LU.m[4][0]);
//   TEST_ASSERT_FLOAT_WITHIN(0.0001f, LUExp.m[4][1], LU.m[4][1]);
//   TEST_ASSERT_FLOAT_WITHIN(0.0001f, LUExp.m[4][2], LU.m[4][2]);
//   TEST_ASSERT_FLOAT_WITHIN(0.0001f, LUExp.m[4][3], LU.m[4][3]);
//   TEST_ASSERT_FLOAT_WITHIN(0.0001f, LUExp.m[4][4], LU.m[4][4]);
//   TEST_ASSERT_FLOAT_WITHIN(0.0001f, LUExp.m[4][5], LU.m[4][5]);

//   TEST_ASSERT_FLOAT_WITHIN(0.0001f, LUExp.m[5][0], LU.m[5][0]);
//   TEST_ASSERT_FLOAT_WITHIN(0.0001f, LUExp.m[5][1], LU.m[5][1]);
//   TEST_ASSERT_FLOAT_WITHIN(0.0001f, LUExp.m[5][2], LU.m[5][2]);
//   TEST_ASSERT_FLOAT_WITHIN(0.0001f, LUExp.m[5][3], LU.m[5][3]);
//   TEST_ASSERT_FLOAT_WITHIN(0.0001f, LUExp.m[5][4], LU.m[5][4]);
//   TEST_ASSERT_FLOAT_WITHIN(0.0001f, LUExp.m[5][5], LU.m[5][5]);

//   TEST_ASSERT_FLOAT_WITHIN(0.001f, bExp.v[0], sol.v[0]);
//   TEST_ASSERT_FLOAT_WITHIN(0.001f, bExp.v[1], sol.v[1]);
//   TEST_ASSERT_FLOAT_WITHIN(0.001f, bExp.v[2], sol.v[2]);
//   TEST_ASSERT_FLOAT_WITHIN(0.001f, bExp.v[3], sol.v[3]);
//   TEST_ASSERT_FLOAT_WITHIN(0.001f, bExp.v[4], sol.v[4]);
//   TEST_ASSERT_FLOAT_WITHIN(0.001f, bExp.v[5], sol.v[5]);
//   TEST_ASSERT_FLOAT_WITHIN(0.001f, bExp.v[6], sol.v[6]);
// }

void testCombination(){
    //Define
    struct mat77 comb = mzero77();
    float DT = (float)(1/1000.0f);
    float u = 1.049477858789893f;
    struct vec4 d = mkvec4(-0.0790f,17.684f,97.9616f,2299.2f);
    struct mat44 A = {{{1.0f, DT, DT*DT*0.5f, u*DT*DT*0.5f},
                        {0.0f, 1.0f, DT, DT*u},
                        {0.0f, 0.0f, 1.0f, 0.0f},
                        {0.0f, 0.0f, 0.0f, 1.0f}}};
    struct mat44 AT = mtranspose44(A);
    //Test
    struct mat44 Qbt = mmul44(AT,A);
    struct mat44 Qb = mscl44(2.0f, Qbt);
    struct vec4 Cb = mvmul44(AT,d);
    Cb = vscl4(-2.0f, Cb);

    //Write Qb into comb (need a better name tbh)
    comb.m[0][0] = Qb.m[0][0]; comb.m[0][1] = Qb.m[0][1]; comb.m[0][2] = Qb.m[0][2]; comb.m[0][3] = Qb.m[0][3];
    comb.m[1][0] = Qb.m[1][0]; comb.m[1][1] = Qb.m[1][1]; comb.m[1][2] = Qb.m[1][2]; comb.m[1][3] = Qb.m[1][3];
    comb.m[2][0] = Qb.m[2][0]; comb.m[2][1] = Qb.m[2][1]; comb.m[2][2] = Qb.m[2][2]; comb.m[2][3] = Qb.m[2][3];
    comb.m[3][0] = Qb.m[3][0]; comb.m[3][1] = Qb.m[3][1]; comb.m[3][2] = Qb.m[3][2]; comb.m[3][3] = Qb.m[3][3];

    //Assert
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, comb.m[0][0]);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, comb.m[3][3]);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0021f, comb.m[1][3]);

    TEST_ASSERT_FLOAT_WITHIN(0.01f, -0.157981067401892f, -Cb.v[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 35.369005085847610f, -Cb.v[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 195.9584992195222f, -Cb.v[2]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 4598.366805884545f, -Cb.v[3]);
}

void testInverse(){
    //Expected
    float fExp = 86.4130f;
    float bExp = 2287.0f;


    //Compute LU Decomp
    //Some Matrix Definitions
      struct mat77 comb = mzero77();
      // Some of these never need to change to they are hard stored 
      comb.m[0][4] = 1.0f;
      comb.m[1][5] = 1.0f;
      comb.m[2][6] = 1.0f;
      comb.m[3][6] = 1.0495f;

      comb.m[4][0] = 1.0f;
      comb.m[5][1] = 1.0f;
      comb.m[6][2] = 1.0f;
      comb.m[6][3] = 1.0495f;

      
      //Write Qb into comb (need a better name tbh)
      comb.m[0][0] = 2.0f; comb.m[0][1] = 0.002f; comb.m[0][2] = 0.0f; comb.m[0][3] = 0.0f;
      comb.m[1][0] = 0.002f; comb.m[1][1] = 2.0f; comb.m[1][2] = 0.002f; comb.m[1][3] = 0.0021f;
      comb.m[2][0] = 0.0f; comb.m[2][1] = 0.002f; comb.m[2][2] = 2.0f; comb.m[2][3] = 0.0f;
      comb.m[3][0] = 0.0f; comb.m[3][1] = 0.0021f; comb.m[3][2] = 0.0f; comb.m[3][3] = 2.0f;

      struct vec7 b = mkvec7(-0.158f,35.369f,195.956f,4598.4f,-0.0790f,0.0793f,2486.6f);

      int P7[7] = {0}; //To keep track of pivots
      struct mat77 LU = doolittle(comb,P7);
      struct vec7 x_min = linearSolve(LU,b,P7);
      float F_z_min = x_min.v[2];
      float beta_min = x_min.v[3];

    TEST_ASSERT_FLOAT_WITHIN(0.1f, fExp, F_z_min);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, bExp, beta_min);
}

void testFullRun(){
    //Fixture
    struct mat77 comb = mzero77();
    float DT = (float)(1/100.0f);
    float u = 0.0;
    struct vec4 d = mkvec4(0.828644030197017f,0.772907896083155f,-21.933998718084000f,32.094800485448190f);
    struct mat44 A = {{{1.0f, DT, DT*DT*0.5f, u*DT*DT*0.5f},
                        {0.0f, 1.0f, DT, DT*u},
                        {0.0f, 0.0f, 1.0f, 0.0f},
                        {0.0f, 0.0f, 0.0f, 1.0f}}};
    struct mat44 AT = mtranspose44(A);
    //Test
    struct mat44 Qbt = mmul44(AT,A);
    struct mat44 Qb = mscl44(2.0f, Qbt);
    struct vec4 Cb = mvmul44(AT,d);
    Cb = vscl4(-2.0f, Cb);
    comb.m[0][4] = 1.0f;
    comb.m[1][5] = 1.0f;
    comb.m[2][6] = 1.0f;
    comb.m[3][6] = u;

    comb.m[4][0] = 1.0f;
    comb.m[5][1] = 1.0f;
    comb.m[6][2] = 1.0f;
    comb.m[6][3] = u;

    //Write Qb into comb (need a better name tbh)
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 4; j++)
            comb.m[i][j] = Qb.m[i][j];
    }

    struct vec7 b = mkvec7(-Cb.v[0],-Cb.v[1],-Cb.v[2],-Cb.v[3],0.8286f,0.1603f,-0.3365f);
    int P7[7] = {0}; //To keep track of pivots
    uint64_t startt = usecTimestamp();
    struct mat77 LU = doolittle(comb, P7);
    struct vec7 x_min = linearSolve(LU,b,P7);
    uint64_t endt = usecTimestamp();
    printf("Time taken is %lld ms \n",(endt-startt)*1000);
    float F_z_min = x_min.v[2];
    float beta_min = x_min.v[3];

    //Test

    //Assert
    TEST_ASSERT_FLOAT_WITHIN(0.001,-22.1912f,F_z_min);
    TEST_ASSERT_FLOAT_WITHIN(0.001,31.9187f,beta_min);


}