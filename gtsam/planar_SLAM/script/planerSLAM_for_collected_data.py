# pylint: disable=invalid-name, E1101
from __future__ import print_function
import math
from tkinter import Y
from turtle import color, distance
from unittest.result import failfast
import pandas as pd
import gtsam
import numpy as np
from gtsam.symbol_shorthand import L, X
from sympy import O
import networkx as nx
from networkx.drawing.nx_pydot import write_dot
from typing import Sequence, final
import numpy as np
import numpy.linalg as linalg
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from IPython.core.pylabtools import figsize, getfigs
import time
import numpy as np
import matplotlib.pyplot as plt

# Create noise models
PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.0005, 0.0003, 0.0001]))
MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.2]))
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.0, 0.0, 0.0]))

# file paths
file_name = "/ubuntu_disk/ravi/SDP/Sdp_factor_graphs/gtsam/data_collection/Specific_data_files/sim_full_loop_tf.csv"
output_path = "/ubuntu_disk/ravi/SDP/sim_test/output_files/"+'gtsam_output_'+'sim_full_loop.csv'

loaded_data = None

def data_loader():
    
    global file_name
    global x_distance_step
    global y_distance_step
    global loaded_data

    my_data = np.genfromtxt(file_name, delimiter=',', skip_header=1)
    
    # delete the row if the value in second column is nan
    my_data = my_data[~np.isnan(my_data[:,2])]

    # delete first and second column
    my_data = np.delete(my_data, [0,1], axis=1) # delete first column sno and frame_id

    # ["aruco_id", "X","Y","Z","o_x","o_y","o_z"]

    #X,Y,Z are in Aruco position
    #o_x,o_y,o_z are in baseframe position from odom

    np.delete(my_data, list(range(0, my_data.shape[0], 2)), axis=0)

    loaded_data = my_data
    return my_data

def angle_calculator(idx):

    global loaded_data
    data = loaded_data
    
    x = data[idx][1]
    z = data[idx][3]
    o_x = data[idx][4]
    o_z = data[idx][6]

    x_arc = o_x - x
    z_arc = o_z - z

    angle = np.arctan2(z_arc,x_arc)

    return angle


def distance_calculator(idx):


    global loaded_data
    data = loaded_data

    x = data[idx,1]
    y = data[idx,2]
    z = data[idx,3]

    o_x = data[idx,4]
    o_y = data[idx,5]
    o_z = data[idx,6]

    x_arc = o_x - x
    y_arc = o_y - y
    z_arc = o_z - z

    distance = np.sqrt(x_arc**2 + y_arc**2 + z_arc**2)

    return distance

def initial_estimate_landmark(idx):
    
        estimate = { 0 :[2.457349769	,-2.476124194	,-0.005418868368],
                    1 : [2.807765644	,-2.355438329	,-0.4103379515],
                    3 : [1.771994398	,-4.437056738	,-1.002728486],
                    2 : [3.084205077	,-3.829282619	,-0.7043568445],
                    4 : [1.071734559	,-3.162029276	,-0.7192004394]}

        return [estimate[idx][0], estimate[idx][1]]

def dict_generator():

    global loaded_data
    our_data = loaded_data
    ids_in_data = our_data[:,0] #`get the ids in the data
    ids_in_data = [x for x in ids_in_data if str(x) != 'nan']
    LM_dict ={}
    cnt=0

    for i in ids_in_data:
        if (np.isnan(i)):
            continue
        if(i not in list(dict.keys(LM_dict))):
            LM_dict[i]=cnt
            cnt+=1
    return LM_dict, ids_in_data


def plotting(factor_data, odom_data, lbl_s, lbl_r,  clr_s, clr_r,  title):
    
    x_f = factor_data[:,0]
    y_f = factor_data[:,1]

    x_o = odom_data[:,1]
    y_o = odom_data[:,2]

  
    fig, ax = plt.subplots(1,1,figsize=(20,20))
    ax.scatter(x_o, y_o, label = lbl_r,color = 'orange',edgecolors='orange')
    ax.scatter(x_f, y_f, label = lbl_s, color = 'black',edgecolors='black')
    
    ax.scatter(x_f[0], y_f[0], label = "Starting_position_FACTOR", marker="x", color='red', edgecolors='black')
    ax.scatter(x_o[0], y_o[0], label = "Starting_position_ODOM", marker="x", color='cyan', edgecolors='orange')

    ax.scatter(x_f[-1], y_f[-1], label = "Ending_position_FACTOR", marker="x", color='green', edgecolors='black')
    ax.scatter(x_o[-1], y_o[-1], label = "Ending_position_ODOM", marker="x", color='magenta', edgecolors='orange')

    plt.axis("equal")
    plt.xlabel("X-axis (cm)",fontsize = 9)
    plt.ylabel("Y-axis (cm)",fontsize =9)
    plt.grid()
    plt.legend(loc=1, prop={'size': 9})
    
    print("plotting done and saving the figure")
    
    plt.savefig('/ubuntu_disk/ravi/SDP/sim_test/Images/'+title +'_'+ str(time.time()) + ".png")

    plt.show(block=False)
    plt.pause(2)
    plt.close()


def main():
    """Main runner"""
    
    print("Loading data")
    G = nx.Graph()

    # Create an empty nonlinear factor graph
    graph = gtsam.NonlinearFactorGraph()

    odom_data= data_loader()
    odom_data= odom_data[:,[-3,-2,-1]]

    # Create the keys corresponding to unknown variables in the factor graph
    # Getting Landmarks:
    landmarks , ids_in_data = dict_generator()

    # finding the closest point to the end point
    odom_data = data_loader()[:,[-3,-2,-1]]
    min_distance = 1

    for i in odom_data[0:200]:
        if (np.linalg.norm(i-odom_data[-1]) < min_distance):
            min_distance = np.linalg.norm(i-odom_data[-1])
            closest_point = i

    #First node with prior
    print("Creating graph")

    odom_initial = odom_data[0] #initial pose
    G.add_edges_from([ ["X0", "L"+str(landmarks[ids_in_data[0]]) ] ])
    graph.add(gtsam.PriorFactorPose2(X(0),gtsam.Pose2(odom_initial[0],odom_initial[1], odom_initial[2]), PRIOR_NOISE))

    angle = angle_calculator(0)
    distance = distance_calculator(0)
    graph.add(gtsam.BearingRangeFactor2D(X(0), L(landmarks[ids_in_data[0]]), gtsam.Rot2.fromDegrees(angle),distance, MEASUREMENT_NOISE))
    
    #Then connecting consecutive nodes using between factor pose 2
    for i in range(1, len(ids_in_data)):
        if all(odom_data[i] == closest_point):
            closest_point_idx = i
            print("Closest point found at index: ", closest_point_idx)

        odom=odom_data[i]-odom_data[i-1]
        graph.add(gtsam.BetweenFactorPose2(X(i-1), X(i), gtsam.Pose2(odom[0],odom[1], odom[2]),ODOMETRY_NOISE))
        
        angle = angle_calculator(i)
        distance = distance_calculator(i)
        graph.add(gtsam.BearingRangeFactor2D(X(i), L(landmarks[ids_in_data[i]]), gtsam.Rot2.fromDegrees(angle),distance, MEASUREMENT_NOISE))

        G.add_edges_from([ ["X"+str(i-1),"X"+str(i)] ])
        G.add_edges_from([ ["X"+str(i),"L"+str(landmarks[ids_in_data[i]]) ] ])


    # Loop closure
    odom = None
    odom = odom_data[0] - odom_data[i] 
    graph.add(gtsam.BetweenFactorPose2(X(len(ids_in_data)-1), X(closest_point_idx), gtsam.Pose2(0.0,0.0,0.0),ODOMETRY_NOISE))
    
    # Create an initial estimate to the solution
    print("Creating initial estimate")
    initial_estimate = gtsam.Values()

    odom_x = odom_data[0][0]
    odom_y = odom_data[0][1]
    odom_theta = odom_data[0][2]

    initial_estimate.insert(X(0), gtsam.Pose2(odom_x, odom_y, odom_theta))  

    for i in range(1,len(ids_in_data)):
        odom=odom_data[i]
        initial_estimate.insert(X(i), gtsam.Pose2(odom[0],odom[1],odom[2]))
    
    for i in range(len(list(dict.keys(landmarks)))):
        initial_estimate.insert(L(i), gtsam.Point2(initial_estimate_landmark(i)[0],initial_estimate_landmark(i)[1]))

    epochs = 1
    for epoch in range(epochs):

        print("Optimizing")
        params = gtsam.LevenbergMarquardtParams()
        optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial_estimate,
                                                    params)
        result = optimizer.optimize()

        final_data =np.zeros((len(ids_in_data),3))

        for i in range(len(ids_in_data)):

            final_data[i][0] = result.atPose2(X(i)).x()
            final_data[i][1] = result.atPose2(X(i)).y()
            final_data[i][2] = result.atPose2(X(i)).theta()


        initial_estimate = gtsam.Values()
        initial_estimate.insert(X(0), gtsam.Pose2(odom_x, odom_y, odom_theta))

        for i in range(1,len(ids_in_data)):
            initial_estimate.insert(X(i), gtsam.Pose2(final_data[i][0],final_data[i][1], final_data[i][2]))
            
        for i in range(len(list(dict.keys(landmarks)))):
            initial_estimate.insert(L(i), gtsam.Point2(initial_estimate_landmark(i)[0],initial_estimate_landmark(i)[1]))

    print("saving data")
    pd.DataFrame(final_data).to_csv(output_path,sep=',',index=False)
    nx.draw(G, with_labels=True, cmap = plt.get_cmap('jet'))
    write_dot(G, '/ubuntu_disk/ravi/SDP/sim_test/output_files/factor_graph_sim.dot')
    plt.show()
    print("Done")

if __name__ == "__main__":
    main()
    gtsam_file = "/ubuntu_disk/ravi/SDP/sim_test/output_files/gtsam_output_sim_full_loop.csv"
    odom_sim_data = "/ubuntu_disk/ravi/SDP/sim_test/extracted_data/sim_full_loop_odom.csv"


    factor_data = np.genfromtxt(gtsam_file, skip_header=1, delimiter=',')
    odom_data = np.genfromtxt(odom_sim_data, skip_header=1, delimiter=',')

    plotting(factor_data, odom_data,
            "factor motion", "odom motion",
            clr_s = "blue", clr_r= "orange", 
            title="factor vs odom")


