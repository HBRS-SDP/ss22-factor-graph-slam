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


path = "/home/tharun/Desktop/Semester_2/SDP/ss22-factor-graph-slam/gtsam/data_collection/tf_data/sim_full_loop_tf.csv"
cmd_cnt = 0
count = 0
flag = True
rostime = None
data = []

data =  ["S.no","frame_id", "child_frame_id", "X","Y","Z"]

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
        data_header = ["S.no","frame_id", "child_frame_id", "X","Y","Z"]
        data_writer(data_header)
        flag = False

    data[1] =  msg.transforms[0].header.frame_id
    data[2] =  msg.transforms[0].child_frame_id
    data[3] =  msg.transforms[0].transform.translation.x
    data[4] = msg.transforms[0].transform.translation.y
    data[5] = msg.transforms[0].transform.translation.z

    data_writer(data)
 
def listener():

    # global fiducial
    global cmd_vel

    rospy.init_node('getter', anonymous=False)
    cmd_vel = rospy.Subscriber('/tf', tf2_msgs.msg.TFMessage, callback_tf,queue_size=25)
    rospy.spin()

if __name__ == '__main__':

    listener()


# Reference
# https://www.codegrepper.com/code-examples/python/python+convert+time+to+timestamp
# https://pythonguides.com/python-dictionary-to-csv/



