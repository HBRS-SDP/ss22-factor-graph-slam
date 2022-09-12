#!/usr/bin/env python

from __future__ import print_function
import time
import rospy
import os
from nav_msgs.msg import Odometry
from csv import writer
import tf2_msgs.msg 
import numpy as np
from tkinter import Y
import pandas as pd
import gtsam
import numpy as np
from gtsam.symbol_shorthand import L, X
from sympy import O
import networkx as nx
from networkx.drawing.nx_pydot import write_dot
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import time
import numpy as np
import matplotlib.pyplot as plt
import touch


source_data_file = "extracted_data.csv"
output_path = "factor_graph_data.csv"
result_file_path = "./"

loaded_data = None

print("Creating the data holding files")
# intialize csv file empty every time
with open(source_data_file, 'w') as my_new_csv_file:
   pass
with open(output_path, 'w') as my_new_csv_file:
   pass

cmd_cnt = 0
count = 0
flag = True
rostime = None
data = []

data =  [count,'-','-','-','-','-','-','-','-']
# data =  ["S.no","frame_id", "child_frame_id", "X","Y","Z"]

def data_writer(data_to_write):

    global data, count


    if not flag:
        count+=1
        data_to_write[0] = count

    # create a file to write the data if it does not exist
    if not os.path.exists(source_data_file):
        with open(source_data_file, 'w+') as f_object:
            writer_object = writer(f_object)
            writer_object.writerow(data_to_write)
            f_object.close()
    else:
        with open(source_data_file, 'a+') as f_object:
            writer_object = writer(f_object)
            writer_object.writerow(data_to_write)
            f_object.close()


def callback_tf(msg):

    global flag, count, rostime, data

    if flag:
        data_header = ["S.no","frame_id", "child_frame_id", "X","Y","Z","o_x","o_y","o_z"]
        data_writer(data_header)
        flag = False

    if(msg.transforms[0].child_frame_id[0]=="b"): # if the child frame is the base_link

        data[1] = "-"
        data[2] = "-"
        data[3] = "-"
        data[4] = "-"
        data[5] = "-"

    elif(msg.transforms[0].child_frame_id[0]=="F"): # if the child frame is the Frame_<aruco id>

        data[1] =  msg.transforms[0].header.frame_id
        data[2] =  msg.transforms[0].child_frame_id[6:8] # aruco id
        data[3] =  msg.transforms[0].transform.translation.x
        data[4] = msg.transforms[0].transform.translation.y
        data[5] = msg.transforms[0].transform.translation.z

        data_writer(data)
 
def callback_odom(msg):

    global flag , count, rostime, data

    if flag:
        data_header = ["S.no","frame_id", "child_frame_id", "X","Y","Z","o_x","o_y","o_z"]
        data_writer(data_header)
        flag = False

    data[-3] =(msg.pose.pose.position.x)
    data[-2] =(msg.pose.pose.position.y)
    data[-1] =( msg.pose.pose.orientation.z )

    data_writer(data)


def data_loader():

    """
    This function loads the data from the csv file and returns it as a numpy array
    The data description is as follows:
    ["aruco_id", "X","Y","Z","o_x","o_y","o_z"]
    X,Y,Z are in Aruco position
    o_x,o_y,o_z are in baseframe position from odom
    """
    
    global source_data_file , loaded_data
    my_data = np.genfromtxt(source_data_file, delimiter=',', skip_header=1)

    my_data = my_data[~np.isnan(my_data[:,2])] # delete the row if the value in second column is nan
    my_data = np.delete(my_data, [0,1], axis=1) # delete first column sno and frame_id
    np.delete(my_data, list(range(0, my_data.shape[0], 2)), axis=0)
    loaded_data = my_data
    return my_data

def angle_calculator(idx):

    """
    This function calculates the angle between the aruco frame and the base_link frame
    """
    global loaded_data
    data = loaded_data
    
    x = data[idx][1]
    z = data[idx][3]
    o_x = data[idx][4]
    o_z = data[idx][6]
    x_arc = o_x - x
    z_arc = o_z - z

    # we need pitch because of the aruco marker orientation
    #Calculate angle of a point with respect to the robot base frame
    angle = np.arctan2(z_arc,x_arc)
    return angle

def distance_calculator(idx):

    """
    This function calculates the distance between the aruco frame and the base_link frame
    """
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

    """
    This function returns the initial estimate of the landmark
    """
    estimate = { 0 :[2.457349769	,-2.476124194	,-0.005418868368],
                1 : [2.807765644	,-2.355438329	,-0.4103379515],
                2 : [3.084205077	,-3.829282619	,-0.7043568445],
                3 : [1.771994398	,-4.437056738	,-1.002728486],
                4 : [1.071734559	,-3.162029276	,-0.7192004394]}

    return [estimate[idx][0], estimate[idx][1]] # x,y coordinates of the landmark

def dict_generator():

    """
    This function generates a dictionary of the landmarks 
    """

    global loaded_data
    our_data = loaded_data
    ids_in_data = our_data[:,0] #`get the ids in the data
    ids_in_data = [x for x in ids_in_data if str(x) != 'nan'] # remove nan values
    LM_dict ={}
    cnt=0
    for i in ids_in_data:
        if (np.isnan(i)):
            continue
        if(i not in list(dict.keys(LM_dict))):
            LM_dict[i]=cnt
            cnt+=1
    return LM_dict, ids_in_data

def result_saver(final_data, graph):

    """
    This function saves the result in a csv file
    """
    print("saving data")
    pd.DataFrame(final_data).to_csv(output_path,sep=',',index=False)
    nx.draw(graph, with_labels=True, cmap = plt.get_cmap('jet'))
    write_dot(graph, result_file_path+'/factor_graph_sim.dot')
    plt.show(block = False)
    plt.pause(3)
    plt.close()
    print("Done")

def odom_factorgraph_comaprator():

    print("Starting the odom factor graph comparision")

    global source_data_file, output_path

    factor_data = np.genfromtxt(output_path, skip_header=1, delimiter=',')
    odom_data = np.genfromtxt(source_data_file, skip_header=1, delimiter=',')

    #only take last 3 columns from odom data
    odom_data = odom_data[:,6:9]

    lbl_s = "factor motion"
    lbl_r = "odom motion"
    title = "factor vs odom"

    x_f = factor_data[:,0]
    y_f = factor_data[:,1]

    x_o = odom_data[:,0]
    y_o = odom_data[:,1]

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
    
    # save the figure with timestamp
    
    print("plotting done and saving the figure")

    plt.savefig(title +'_'+ str(time.time()) + ".png")
    plt.show(block=False)
    plt.pause(2)
    plt.close()


def factor_graph_generator():

    """
    This function generates the factor graph for the landmarks 
    """
    
    # Create noise models
    PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.0005, 0.0003, 0.0001]))
    MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.2]))
    ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.0, 0.0, 0.0]))


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
    closest_point = odom_data[0]


    for i in odom_data[0:int(len(odom_data)/2)]: # search only in the first half of the data
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

    closest_point_idx = 0
    for i in range(1, len(ids_in_data)):
        if all(odom_data[i] == closest_point):
            closest_point_idx = i

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
    for _ in range(epochs):

        print("Optimizing")
        params = gtsam.LevenbergMarquardtParams()
        optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial_estimate,params)
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

    result_saver(final_data, G)

def main():

    global odom, transform

    print("Starting_node_gtsam")
    rospy.init_node('gtsam_slam', anonymous=False)
    print("Node initialized")
    odom = rospy.Subscriber('/odom',Odometry, callback_odom,queue_size=25)
    transform = rospy.Subscriber('/tf', tf2_msgs.msg.TFMessage, callback_tf,queue_size=25)
    print("Subscribers initialized")

    Max_time_for_subscriber = 65
    print("Node will run for {} seconds".format(Max_time_for_subscriber))
    rospy.sleep(Max_time_for_subscriber)
    print("Node terminated")
    factor_graph_generator()
    odom_factorgraph_comaprator()
    

if __name__ == '__main__':
    main()
