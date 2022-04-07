/*
 * Owner: H. Yildiz
 * Brief: KalmanFilter.py ported to C
 * Note: gcc -o KalmanFilter KalmanFilter.c MatrixMath.c -lm
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h> //memcpy()
#include <math.h>   //sin(), cos(), pow()
#include "MatrixMath.h"
#include "KalmanFilter.h"

#define LEN(arr) ((int) (sizeof (arr) / sizeof (arr)[0])) //LEN(arr) = rows, LEN(arr[0]) = columns
#define PI 3.14159265
#define RADIUS_EARTH 6378137 // Radius of the earth is 6378137m

/*
 * Inputs:
 *   t = time interval between itterations
 *   u_k = acceleration
 *   theta = Angle apposed to true North
 *
 * 3 Step process (as seen in the report):
 * Step Prediction (1):
 * 1: X_kp = AX_k-1 + Bu_k + w_k                           // Predict current elements (position/volicity in this example)
 * 2: P_kp = AP_k-1A^T + Q_k                               // Process converiance matrix to calculate the Kalman gain in step 4
 * Step Update (2):
 * 3: K = (P_kpH^T)/(HP_kpH^T + R)                         // Calculate the Kalman Gain
 * 4: Y_k = CY_km + z_m                                    // Convert input to correct form
 * 5: X_k = X_kp + K[Y_k - HX_kp]                          // Update prediction from step 2 with the measurement
 * 6: P_k = (I - KH)P_kp                                   // Calculate new error
 * Step Reload (3):
 * 7: X_k-1 = X_k and P_k-1 = P_k                          // Current becomes previous in the next iteration (current is output)
*/

void prediction(struct KalmanSet *set, float u_k, int theta)
{
    // 1: X_kp = AX_k-1 + Bu_k + w_k
    float A[ROW2][COL2] = {1,0,0,1};
    float B[ROW2][COL1] = {cos(theta * PI/180), sin(theta * PI/180) / cos(set->X_k1[0][0] * PI/180)};
    int w_k = 0;

    // Convert meters to delta latitude
    u_k = (u_k / 3.6) / ((2 * PI/360) * RADIUS_EARTH);

    //self.X_kp = np.dot(A, self.X_k1) + np.dot(B, u_k) + w_k
    float temp[ROW2][COL1];
    float temp2[ROW2][COL1];

    matrix_multiplication(LEN(A), LEN(A[0]), A, LEN(set->X_k1[0]), set->X_k1, temp);
    matrix_dot_vector(LEN(B), LEN(B[0]), B, u_k, temp2);
    matrix_addition(ROW2, COL1, temp, temp2, set->X_kp);

    // 2: P_kp = AP_k-1A^T + Qk
    float temp3[ROW2][COL2];
    float temp4[ROW2][COL2];
    float temp5[ROW2][COL2];

    float Q_k = pow(0.1, 2);

    matrix_multiplication(LEN(A), LEN(A[0]), A, LEN(set->P_k1[0]), set->P_k1, temp3);
    matrix_transpose(LEN(A), LEN(A[0]), A, temp4);
    matrix_multiplication(LEN(set->P_k1), LEN(set->P_k1[0]), set->P_k1, LEN(temp4[0]), temp4, temp5);
    matrix_additionSingle(LEN(temp5), LEN(temp5[0]), temp5, Q_k, set->P_kp);
    matrix_keep_diagonal(LEN(set->P_kp), LEN(set->P_kp[0]), set->P_kp, set->P_kp);
}

void update(struct KalmanSet *set, float Oe, float Mea[][COL1])
{
    // 3: K = (P_kpH^T)/(HP_pkH^T + R) // H = identity matrix
    float temp[ROW2][COL2];
    float temp2[ROW2][COL2];

    float Rdop[ROW2][COL2] = {pow(Oe,2), 0, 0, pow(Oe,2)};

    matrix_addition(LEN(Rdop), LEN(Rdop[0]), Rdop, set->P_kp, temp);
    // Division A/B = A*B^-1, this is also the only place in the code #IND00 error's can occur.
    matrix_inverse(temp, temp2);
    matrix_multiplication(LEN(set->P_kp), LEN(set->P_kp[0]), set->P_kp, LEN(temp2[0]), temp2, set->K_g);

    // 4: Y_k = CY_km + z_m // C = identity matrix and z_m = 0
    matrix_duplicate(LEN(set->Y_k), LEN(set->Y_k[0]), Mea, set->Y_k);

    // 5: X_k = X_kp + K[Y_k - HX_kp] // H = identity matrix
    float temp3[ROW2][COL1];
    float temp4[ROW2][COL1];

    matrix_substraction(LEN(set->Y_k), LEN(set->Y_k[0]), set->Y_k, set->X_kp, temp3);
    matrix_multiplication(LEN(set->K_g), LEN(set->K_g[0]), set->K_g, LEN(temp3[0]), temp3, temp4);
    matrix_addition(LEN(set->X_kp), LEN(set->X_kp[0]), set->X_kp, temp4, set->X_k);

    // 6: P_k = (I - KH)P_kp // I = H = identity matrix
    float identity[ROW2][COL2] = {1,0,0,1};
    matrix_substraction(LEN(identity), LEN(identity[0]), identity, set->K_g, temp);
    matrix_multiplication(LEN(temp), LEN(temp[0]), temp, LEN(set->P_kp[0]), set->P_kp, set->P_k);
}

void reload(struct KalmanSet *set)
{
    // 7: X_k-1 = X_k and P_k-1 = P_k
    matrix_duplicate(LEN(set->X_k1), LEN(set->X_k1[0]), set->X_k, set->X_k1);
    matrix_duplicate(LEN(set->P_k1), LEN(set->P_k1[0]), set->P_k, set->P_k1);
}


// https://stackoverflow.com/questions/12911299/read-csv-file-in-c
const char* getfield(char* line, int num)
{
    const char* tok;
    char *tmp = line;
    for (tok = strtok(tmp, ",");
            tok && *tok;
            tok = strtok(NULL, ",\n"))
    {
        if (!--num)
            return tok;
    }
    return NULL;
}

int main(void)
{
    FILE* input_stream = fopen("../Datasets/kalmansetLondon.csv", "r");
    FILE *output_file = fopen("../Datasets/ksetOutputLondonC.csv", "w");
    
    char line[1024];
    fgets(line, 1024, stream); // Skip first line

    struct KalmanSet package = {.X_k1 = {51.54196,-0.0143366666666667}, .P_k1 = {0.8, 0, 0, 0.8}};
    float timestamp, prevTimestamp;
    float measurements[2][1];
    float hdop;

    int line_number = 1;

    while (fgets(line, 1024, stream))
    {
        // If the second counter goes from 59 to 00, adjust
        timestamp = strtod(getfield(strdup(line), 1), NULL);
        if ((timestamp-prevTimestamp) == 41)
            prevTimestamp += 40;
        else if ((timestamp-prevTimestamp) == 4041)
            prevTimestamp += 4040;

        prediction(&package, (timestamp - prevTimestamp) * strtod(getfield(strdup(line), 6), NULL), strtod(getfield(strdup(line), 4), NULL));

        measurements[0][0] = strtod(getfield(strdup(line), 2), NULL);
        measurements[1][0] = strtod(getfield(strdup(line), 3), NULL);

        hdop = strtod(getfield(strdup(line),5), NULL);
        if (hdop == 0)
            hdop = 1;
        update(&package, hdop, measurements);

        reload(&package);
        fprintf(output_file, "%.13g, %.13g, %.13g, %.13g, %d\n", measurements[0][0], measurements[1][0], package.X_k[0][0], package.X_k[1][0], line_number);

        prevTimestamp = timestamp;

        line_number++;
    }
    fclose(output_file);
    fclose(stream);
}
