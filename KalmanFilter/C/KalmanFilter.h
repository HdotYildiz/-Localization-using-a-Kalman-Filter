/*
 * Owner: H. Yildiz
 */

#ifndef __KalmanFilter
#define __KalmanFilter

struct KalmanSet {
    float X_k1[ROW2][COL1];
    float X_kp[ROW2][COL1];
    float X_k[ROW2][COL1];
    float P_k1[ROW2][COL2];
    float P_kp[ROW2][COL2];
    float P_k[ROW2][COL2];
    float K_g[ROW2][COL2];
    float Y_k[ROW2][COL1];
};

/*
 * 3 Step process:
 * Step Prediction (1):
 * 1: X_kp = AX_k-1 + Bu_k + w_k                                                 // Predict current elements (position/volicity in this example)
 * 2: P_kp = AP_k-1A^T + Q_k                                                     // Process converiance matrix to calculate the Kalman gain in step 4
 * Step Update (2):
 * 3: K = (P_kpH^T)/(HP_kpH^T + R)                                               // Calculate the Kalman Gain
 * 4: Y_k = CY_km + z_m                                                          // Convert input to correct form
 * 5: X_k = X_kp + K[Y_k - HX_kp]                                                // Update prediction from step 2 with the measurement
 * 6: P_k = (I - KH)P_kp                                                         // Calculate new error
 * Step Reload (3):
 * 7: X_k-1 = X_k and P_k-1 = P_k                                                // Current becomes previous in the next iteration (current is output)
*/

/*
 * Brief: The first step in the Kalman process.
 *
 * Inputs:
 *   t = time interval between itterations
 *   u_k = acceleration
 *   theta = Angle apposed to true North
 */
void prediction(struct KalmanSet *set, float u_k, int theta);

/*
 * Brief: The second step in the Kalman process.
 *
 * Inputs:
 *   Oe = Observational errors, matrix of error in measurement
 *   Mea = Location and velocity measurement
 */
void update(struct KalmanSet *set, float Oe, float Mea[][COL1]);

/*
 * Brief: The last step in the Kalman process.
 */
void reload(struct KalmanSet *set);

#endif
