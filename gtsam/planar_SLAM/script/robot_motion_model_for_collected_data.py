from __future__ import print_function
from tkinter import Y
import pandas as pd
import gtsam
import numpy as np
from gtsam.symbol_shorthand import L, X

# Create noise models
PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))
MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.2]))


from typing import Sequence 
import numpy as np
import numpy.linalg as linalg
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from IPython.core.pylabtools import figsize, getfigs


file_name="/home/tharun/Desktop/Semester_2/SDP/ss22-factor-graph-slam/gtsam/data_collection/Extracted_data_from_bag_files/velocity_sdp_data_collection_look_one_3_marker.csv"

def data_loader():
    data = np.genfromtxt(file_name, delimiter=',',skip_header=1)

    velocity_data = data[:,[0,1,5]]
    return velocity_data

def main():
    graph = gtsam.NonlinearFactorGraph()

    velocity_data= data_loader()

    graph.add(gtsam.PriorFactorPose2(0,gtsam.Pose2(0.37, 0.45, -0.15), PRIOR_NOISE))

    for i in range(1, len(velocity_data), 1):
        odom_data = velocity_data[i]
        x_vel = 0.0178571 * odom_data[0]
        y_vel = 0.0178571 * odom_data[1]
        ang_vel = 0.0178571 * odom_data[2]
        graph.add(gtsam.BetweenFactorPose2(i-1, i, gtsam.Pose2(x_vel, y_vel, ang_vel),ODOMETRY_NOISE))

    initial_estimate = gtsam.Values()
    for i in range(0, len(velocity_data),1):
        initial_estimate.insert(i, gtsam.Pose2(0.0, 0.0, 0.0))

    params = gtsam.LevenbergMarquardtParams()
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial_estimate,
                                                  params)
    result = optimizer.optimize()
    print("\nFinal Result:\n{}".format(result))

    final_data =np.zeros((len(velocity_data),3))
    for i in range(len(velocity_data)):
        final_data[i][0] = result.atPose2(i).x()
        final_data[i][1] = result.atPose2(i).y()
        final_data[i][2] = result.atPose2(i).theta()

    pd.DataFrame(final_data).to_csv('/home/tharun/Desktop/Semester_2/SDP/ss22-factor-graph-slam/gtsam/data_collection/Extracted_data_from_bag_files/final_output_velocity_2_pose.csv',sep=',',index=False)

if __name__ == "__main__":
    main()
