###
# = Kalman-filter
# : H. Yildiz
# /
###

##
#   xInputs:
#   X_k-1 = X_0 = Initial location/velocity
#   P_k-1 = P_0 = Initial error of the Kalman calculation
#   Y_km = Location and velocity measurement
##

##
# 3 Step process (as seen in the report):
# Step Prediction (1):
# 1: X_kp = AX_k-1 + Bu_k + w_k                           // Predict current elements (position/volicity in this example)
# 2: P_kp = AP_k-1A^T + Q_k                               // Process converiance matrix to calculate the Kalman gain in step 4
# Step Update (2):
# 3: K = (P_kpH^T)/(HP_kpH^T + R)                         // Calculate the Kalman Gain
# 4: Y_k = CY_km + z_m                                    // Convert input to correct form
# 5: X_k = X_kp + K[Y_k - HX_kp]                          // Update prediction from step 2 with the measurement
# 6: P_k = (I - KH)P_kp                                   // Calculate new error
# Step Reload (3):
# 7: X_k-1 = X_k and P_k-1 = P_k                          // Current becomes previous in the next iteration (current is output)
##

##
#   xOuputs:
#   X_k = Matrix containing the filtered location and velocity.
#   P_k = Matrix containing the error of the location and velocity
##

from math import sin, cos, pi
import numpy as np

class KalmanFilter:
    # Inputs:
    #   t = time interval between itterations
    #   u_k = acceleration
    #   log = weather or not to log the output
    def __init__(self, X_0, P_0, logger):
        if logger:
            self.logging = True
            self.logger = open("../../logs/kalmanfilterpyIgnore.txt", "w")
            self.logger.write("--- New Run ---")
        else:
            self.logging = False

        self.X_k1 = X_0
        self.P_k1 = np.array([[P_0[0][0]**2, 0],
                              [0, P_0[1][0]**2]])

    def __del__(self):
        if self.logging:
            self.logger.close()

    # Inputs:
    #   t = time interval between itterations
    #   u_k = acceleration
    #   theta = Angle apposed to true North
    def prediction(self, u_k, theta):
        # 1: X_kp = AX_k-1 + Bu_k + w_k
        A = np.array([[1, 0],
                      [0, 1]])
        B = np.array([[cos(theta*pi/180)], # /180 to convert it to radians and /lat to compensate for the earth
                      [sin(theta*pi/180) / cos(self.X_k1[0][0] * pi / 180)]])
        w_k = 0

        if self.logging:
            self.logger.write("u_k: {0}, Theta: {1}, ".format(u_k, theta))

        # Convert meters to delta latitude, radius of the earth is 6378137m
        u_k = (u_k / 3.6) / ((2 * pi / 360) * 6378137)

        self.X_kp = np.dot(A, self.X_k1) + np.dot(B, u_k) + w_k

        if self.logging:
            self.logger.write("B: {0} {1}, X_k1: {2} {3}, u_k2: {4}, ".format(B[0][0], B[1][0], self.X_k1[0][0], self.X_k1[1][0], u_k))

        # 2: P_kp = AP_k-1A^T + Qk
        Q_k = 0.1**2
        self.P_kp = np.dot(np.dot(A, self.P_k1), np.transpose(A)) + Q_k
        self.P_kp = np.diag(np.diag(self.P_kp)) # Only keep diagonal

    # Inputs:
    # Oe = Observational errors, matrix of error in measurement
    # MEA = Location and velocity measurement
    def update(self, Oe, MEA):
        # 3: K = (P_kpH^T)/(HP_pkH^T + R) // H = identity matrix
        R = np.array([[Oe[0][0]**2, 0],
                      [0, Oe[1][0]**2]])

        self.K = np.divide(self.P_kp,(self.P_kp + R))
        self.K = np.nan_to_num(self.K) # Remove nan's from 0/0

        if self.logging:
            self.logger.write("K: {0} {1}".format(self.K[0][0], self.K[1][1]))

        # 4: Y_k = CY_km + z_m // C = identity matrix
        z_m = 0
        self.Y_k = MEA

        # 5: X_k = X_kp + K[Y_k - HX_kp] // H = identity matrix
        self.X_k = self.X_kp + np.dot(self.K, (self.Y_k - self.X_kp))

        # 6: P_k = (I - KH)P_kp // I = H = identity matrix
        self.P_k = np.dot((np.identity(2) - self.K), self.P_kp)


    def reload(self):
        # 7: X_k-1 = X_k and P_k-1 = P_k
        self.X_k1 = self.X_k
        self.P_k1 = self.P_k

    # Outputs:
    # X_kp = Predicted location
    def get_prediction(self):
        return self.X_kp

    # Outputs:
    # X_k = Location and velocity estimate
    def get_output(self):
        return self.X_k

    # Outputs:
    # P_k = Error in estimate
    def get_output_error(self):
        return self.P_k
