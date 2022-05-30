"""
GTSAM Copyright 2010-2018, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
Authors: Frank Dellaert, et al. (see THANKS for the full author list)

See LICENSE for the license information

Simple robotics example using odometry measurements and bearing-range (laser) measurements
Author: Alex Cunningham (C++), Kevin Deng & Frank Dellaert (Python)
"""
# pylint: disable=invalid-name, E1101

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

file_name = '/home/tharun/Desktop/Semester_2/SDP/GTSAM/gtsam/python/gtsam/examples/csv_files/sdp_look_two_3_marker.csv'

x_distance_step = None
y_distance_step = None


def get_RX(theta: float) -> np.ndarray:
    
    cos = np.cos(theta)
    sin = np.sin(theta)
    Rx = np.array(((1,0,0),
                   (0,cos,-sin),
                   (0,sin,cos)))

    return Rx

def get_RY(theta: float) -> np.ndarray:
    
    cos = np.cos(theta)
    sin = np.sin(theta)
    Ry = np.array(((cos , 0  ,sin),
                   ( 0  , 1  ,0),
                   (-sin, 0  ,cos)))

    return Ry

def get_RZ(theta: float) -> np.ndarray:
    
    cos = np.cos(theta)
    sin = np.sin(theta)
    Rz = np.array(((cos , -sin  ,0),
                   ( sin, cos  ,0),
                   (0, 0  ,1)))

    return Rz
    
def get_homogeneous_transform(input):

    rotation_angles =np.array(input[3:6])
    translation=np.array(input[0:3])
    W=input[6]

    rot = rotation_angles

    Rx = get_RX(rot[2]) # rotation about X axis
    Ry = get_RY(rot[1]) # rotation about Y axis
    Rz = get_RZ(rot[0]) # rotation about Z axis
 
    h_stack = np.array([[0],[0],[0]])
    v_stack = np.array([0,0,0,W])
    Rotational_matrix = np.dot(Rz, np.dot(Ry, Rx)) 
    
    r = Rotational_matrix
    r = np.hstack((r,h_stack))
    r = np.vstack((r,v_stack))
    
    trans = np.array(([translation[0]],
                      [translation[1]],
                      [translation[2]]))
    
    Translational_matrix = np.array(((1,0,0),(0,1,0),(0,0,1)))

    t = Translational_matrix
    t = np.hstack((t,trans))
    t = np.vstack((t,v_stack))
    
    H_matrix = t.dot(r)
    
    return H_matrix

def transforms():
    input = [0.239, 0.0, 0.071, 0.0, 0.0,0.0,1]
    b_T_apl = get_homogeneous_transform(input)

    input = [0,0,0,0,0,-0.38268343236488267, 0.9238795325113726]
    apl_T_arl = get_homogeneous_transform(input)

    input = [-0.022,0.0,0.037,0,0,0,1]
    arl_T_a0 = get_homogeneous_transform(input)

    input = [0.024,0.0,0.096,0.0,0.0,0.37421911645649647,0.9273403112549993]
    a0_T_a1 = get_homogeneous_transform(input)

    input = [0.033,0.0,0.019,0,0.528847175314683, 0.0, 0.8487170701486337]
    a1_T_a2 = get_homogeneous_transform(input)

    input = [0,0,0.155,0,-0.18396078610708805,0,0.9829335832979063]
    a2_T_a3 = get_homogeneous_transform(input)

    input = [0.0,0.0,0.135,0.0,0.722613797908215, 0, 0.6912519794348986]
    a3_T_a4 = get_homogeneous_transform(input)

    input = [-0.002, 0, 0.13 , 0,0, 0.7120631548522023,  0.7021154203561754]
    a4_T_a5 = get_homogeneous_transform(input)
    
    input = [-0.005, -0.008,0.009,  -0.49999999999755174, -0.5,-0.5,  0.5000000000024483]
    a5_T_cl = get_homogeneous_transform(input)
    
    input = [-0.000208392972126603, 0.014731519855558872,  0.00020157775725238025, -0.014435658231377602, -0.00020775734446942806, -0.0008906595758162439,  0.9998953938484192]
    cl_T_ccf = get_homogeneous_transform(input)

    b_T_ccf = cl_T_ccf.dot(a5_T_cl).dot(a4_T_a5).dot(a3_T_a4).dot(a2_T_a3).dot(a1_T_a2).dot(a0_T_a1).dot(arl_T_a0).dot(apl_T_arl).dot(b_T_apl)

    return b_T_ccf

def data_loader():
    
    global file_name
    global x_distance_step
    global y_distance_step

    my_data = np.genfromtxt(file_name, delimiter=',',skip_header=1)
    
    x_distance_step = np.average(my_data[:,10]) / len(my_data[0])
    y_distance_step = np.average(my_data[:,11]) / len(my_data[0])

    my_data = np.delete(my_data,[1,2,10,11,12,13,14,15],1)

    return my_data

def yaw_angle_calculator(idx):

    b_T_ccf = transforms()
    data = data_loader()
    my_data = np.delete(data,0,1)
    ccf_T_F_seq = get_homogeneous_transform(my_data[idx])
    b_T_F_seq = ccf_T_F_seq.dot(b_T_ccf)
    rotation = R.from_matrix(b_T_F_seq[0:3,0:3]) # rotation matrix of frame F relative to frame B
    yaw_angle = rotation.as_euler('xyz', degrees=True)[2]

    return yaw_angle

def dict_generator():

    our_data = data_loader()
    ids_in_data = our_data[:,0]
    LM_dict ={}
    cnt=0
    for i in ids_in_data:
        if(i not in list(dict.keys(LM_dict))):
            LM_dict[i]=cnt
            cnt+=1
    print(LM_dict)

    return LM_dict, ids_in_data





def main():
    """Main runner"""
    
    # Getting Landmarks:

    landmarks , ids_in_data = dict_generator()

    # Create an empty nonlinear factor graph
    graph = gtsam.NonlinearFactorGraph()

    # Create the keys corresponding to unknown variables in the factor graph

    #First node with prior
    graph.add(gtsam.PriorFactorPose2(X(0),gtsam.Pose2(0.0, 0.0, 0.0), PRIOR_NOISE))
    angle = yaw_angle_calculator(0)
    graph.add(gtsam.BearingRangeFactor2D(X(0), L(landmarks[ids_in_data[0]]), gtsam.Rot2.fromDegrees(angle),0, MEASUREMENT_NOISE))
    
    #Then connecting consecutive nodes using between factor pose 2
    for i in range(1, len(ids_in_data), 1):
        graph.add(gtsam.BetweenFactorPose2(X(i-1), X(i), gtsam.Pose2(0.00944, -0.09073, 0.0),ODOMETRY_NOISE))
        angle = yaw_angle_calculator(i)
        graph.add(gtsam.BearingRangeFactor2D(X(i), L(landmarks[ids_in_data[i]]), gtsam.Rot2.fromDegrees(angle),0, MEASUREMENT_NOISE))
        

    # Print graph
    # print("Factor Graph:\n{}".format(graph))

    # Create (deliberately inaccurate) initial estimate
    initial_estimate = gtsam.Values()

    for i in range(len(ids_in_data)):
        initial_estimate.insert(X(i), gtsam.Pose2(x_distance_step*i, y_distance_step*i, 0.0))
    
    for i in range(len(list(dict.keys(landmarks)))):
        initial_estimate.insert(L(i), gtsam.Point2(0.95, 0.1))

    # Print
    # print("Initial Estimate:\n{}".format(initial_estimate))

    # Optimize using Levenberg-Marquardt optimization. The optimizer
    # accepts an optional set of configuration parameters, controlling
    # things like convergence criteria, the type of linear system solver
    # to use, and the amount of information displayed during optimization.
    # Here we will use the default set of parameters.  See the
    # documentation for the full set of parameters.
    params = gtsam.LevenbergMarquardtParams()
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial_estimate,
                                                  params)
    result = optimizer.optimize()

    final_data =np.zeros((len(ids_in_data),3))
    for i in range(len(ids_in_data)):
        final_data[i][0] = result.atPose2(X(i)).x()
        final_data[i][1] = result.atPose2(X(i)).y()
        final_data[i][2] = result.atPose2(X(i)).theta()

    # final_data.tofile('/home/tharun/Desktop/Semester_2/SDP/GTSAM/gtsam/python/gtsam/examples/csv_files/final_data.csv',sep=',',format='%10.5f')
    
    pd.DataFrame(final_data).to_csv('/home/tharun/Desktop/Semester_2/SDP/GTSAM/gtsam/python/gtsam/examples/csv_files/final_data.csv',sep=',',index=False)
    print("\nFinal Result:\n{}".format(result))

    # Calculate and print marginal covariances for all variables
    # marginals = gtsam.Marginals(graph, result)
    # for (key, s) in [(X1, "X1"), (X2, "X2"), (X3, "X3"), (L1, "L1"),
    #                  (L2, "L2")]:
    #     print("{} covariance:\n{}\n".format(s,
    #                                         marginals.marginalCovariance(key)))
    # graphviz_formatting = gtsam.GraphvizFormatting()
    # graph.dot(result,writer=graphviz_formatting)


if __name__ == "__main__":
    main()


