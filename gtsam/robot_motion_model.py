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


def main():
    graph = gtsam.NonlinearFactorGraph()

    graph.add(gtsam.PriorFactorPose2(1,gtsam.Pose2(0.0, 0.0, 0.0), PRIOR_NOISE))
    graph.add(gtsam.BetweenFactorPose2(1, 2, gtsam.Pose2(2.0, 0.0, 0.0),ODOMETRY_NOISE))
    graph.add(gtsam.BetweenFactorPose2(2, 3, gtsam.Pose2(2.0, 0.0, 0.0),ODOMETRY_NOISE))

    initial_estimate = gtsam.Values()
    initial_estimate.insert(1, gtsam.Pose2(0.5, 0.0, 0.2))
    initial_estimate.insert(2, gtsam.Pose2(3.5, 0.1, -0.2))
    initial_estimate.insert(3, gtsam.Pose2(7.8, 0.1, 0.1))

    params = gtsam.LevenbergMarquardtParams()
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial_estimate,
                                                  params)
    result = optimizer.optimize()
    print("\nFinal Result:\n{}".format(result))

if __name__ == "__main__":
    main()
