import csv
import numpy as np
from plotly import offline
import plotly.graph_objs as go

from KalmanFilter import KalmanFilter

PLOTS_PATH = "../../Plots/"
DATASETS_PATH = "../../Datasets/"

def plot_kalman_results(prediction, kalman_output, measuredX, measuredY):
    xaxis = [i for i in range(kalman_output)]
    result_X = [float(kalman_output[i][0]) for i in range(kalman_output)]
    result_Y = [float(kalman_output[i][1]) for i in range(kalman_output)]

    trace0 = go.Scatter(x = xaxis, y = result_X, mode = 'lines+markers', name = 'Kalman output X')
    trace1 = go.Scatter(x = xaxis, y = measuredX, mode = 'lines+markers', name = 'Measurement')
    trace2 = go.Scatter(x = xaxis, y = prediction, mode = 'lines+markers', name = 'Kalman Prediction')

    trace3 = go.Scatter(x = xaxis, y = result_Y, mode = 'lines+markers', name = 'Kalman output Y')
    trace4 = go.Scatter(x = xaxis, y = measuredY, mode = 'lines+markers', name = 'Measurement')

    #trace3 = go.Scatter(x = outputX, y = outputY, mode = 'lines+markers', name = 'Kalman output')
    # TODO: Plot error
    #trace4 = go.Scatter(x = measuredX, y = measuredY, mode = 'lines+markers', name = 'Measurement')

    fig = [trace0, trace1, trace2]
    fig2 = [trace3, trace4]

    offline.plot(fig, filename=PLOTS_PATH + "plotKalmanLat.html", auto_open=False)
    offline.plot(fig2, filename=PLOTS_PATH + "plotKalmanLong.html", auto_open=False)

    with open(DATASETS_PATH + "ksetOutputLondon.csv", "w") as k:
        for i in range(len(kalmanOutput)):
            k.write("{0}, {1}, {2}, {3}, {4}\r\n".format(measuredX[i], measuredY[i], result_X[i], result_Y[i], xaxis[i]))

##
# Overall input for the filter:
# - Time interval and velocity  (deltaT * v_observation)
# - Angle of velocity           (theta)
# --
# - GPS measurement of location (measurement)
# - GPS errors                  (ObservationalError)
##
if __name__ == "__main__":
    PLOTTING = True

    kalmanOutput = []
    kalmanPrediction = []
    kalmanOutputError = []
    measuredX = []
    measuredY = []

    def fix_second_counter_overflow(new_stamp, old_stamp):
        # If the second counter goes from 59 to 00, adjust
        if (new_stamp - old_stamp) == 41:
            old_stamp += 40
        elif (new_stamp - old_stamp) == 4041:
            old_stamp += 4040
        
        return old_stamp

    # Using historical data to test the filter.
    with open(DATASETS_PATH + "kalmansetLondon.csv", "r") as f:
        next(f) # Skip header row
        reader = csv.reader(f)

        # Row: 0-Timestamp 1-Lat 2-Long 3-Theta 4-HDOP(str) 5-Velocity
        TIMESTAMP = 0
        LAT = 1
        LONG = 2
        THETA = 3
        HDOP = 4
        VEL = 5

        init_done_flag = False
        previous_timestamp = 0

        row = next(reader)
        KF = KalmanFilter(np.array([[float(row[LAT])],[float(row[LONG])]]), np.array([[float(row[HDOP])],[float(row[HDOP])]]), True)
        
        for row in reader:
            # Check if all data is present
            if "NA" in row:
                continue

            previous_timestamp = fix_second_counter_overflow(int(row[TIMESTAMP]), previous_timestamp)

            KF.prediction((int(row[TIMESTAMP]) - previous_timestamp) * float(row[VEL]), float(row[THETA]))

            kalmanPrediction.append(KF.get_prediction()[0][0])

            measurement = np.array([[float(row[LAT])],[float(row[LONG])]])
            ObservationalError = np.array([[float(row[HDOP])],[float(row[HDOP])]])
            KF.update(ObservationalError, measurement)

            # Used to plot output
            kalmanOutput.append(KF.get_output())
            kalmanOutputError.append(KF.get_output_error())

            measuredX.append(row[LAT])
            measuredY.append(row[LONG])

            KF.reload()

            previous_timestamp = int(row[TIMESTAMP])

    if PLOTTING:
        plot_kalman_results(kalmanPrediction, kalmanOutput, measuredX, measuredY)