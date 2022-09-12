from cmath import nan
from pickle import FALSE
import time

from pytz import common_timezones
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import os

from vision_msgs.msg import Detection2DArray
from nav_msgs.msg import Odometry
from datetime import datetime
from csv import writer

import tf2_msgs.msg 


path = "/ubuntu_disk/ravi/SDP/Sdp_factor_graphs/gtsam/data_collection/tf_data/sim_full_loop_tf.csv"
cmd_cnt = 0
count = 0
flag = True
rostime = None
data = []

data =  [count,'-','-','-','-','-','-','-','-']
# data =  ["S.no","frame_id", "child_frame_id", "X","Y","Z"]

def data_writer(data_to_write):

    global data
    global count

    if not flag:
        count+=1
        data_to_write[0] = count
    with open(path, 'a+') as f_object:

        writer_object = writer(f_object)
        writer_object.writerow(data_to_write)
        f_object.close()


def callback_tf(msg):

    global flag
    global count
    global rostime
    global data

    if flag:
        data_header = ["S.no","frame_id", "child_frame_id", "X","Y","Z","o_x","o_y","o_z"]
        data_writer(data_header)
        flag = False

    if(msg.transforms[0].child_frame_id[0]=="b"):

        data[1] = "-"
        data[2] = "-"
        data[3] = "-"
        data[4] = "-"
        data[5] = "-"

    elif(msg.transforms[0].child_frame_id[0]=="F"):

        data[1] =  msg.transforms[0].header.frame_id
        data[2] =  msg.transforms[0].child_frame_id[6:8]
        data[3] =  msg.transforms[0].transform.translation.x
        data[4] = msg.transforms[0].transform.translation.y
        data[5] = msg.transforms[0].transform.translation.z

        data_writer(data)
 
def callback_odom(msg):

    global flag
    global count
    global rostime
    global data

    if flag:
        data_header = ["S.no","frame_id", "child_frame_id", "X","Y","Z","o_x","o_y","o_z"]
        data_writer(data_header)
        flag = False

    data[-3] =(msg.pose.pose.position.x)
    data[-2] =(msg.pose.pose.position.y)
    data[-1] =( msg.pose.pose.orientation.z )

    data_writer(data)



def listener():

    global odom
    global transform

    rospy.init_node('getter', anonymous=False)
    odom = rospy.Subscriber('/odom',Odometry, callback_odom,queue_size=25)
    transform = rospy.Subscriber('/tf', tf2_msgs.msg.TFMessage, callback_tf,queue_size=25)
    rospy.spin()

if __name__ == '__main__':

    listener()


# Reference
# https://www.codegrepper.com/code-examples/python/python+convert+time+to+timestamp
# https://pythonguides.com/python-dictionary-to-csv/



