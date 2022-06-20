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


path = "/media/ravi/ubuntu_disk/ravi/SDP/Sdp_factor_graphs/gtsam/data_collection/odom_data/sdp_data_collection_"+"two_full_loop_marker"+"_odom"+".csv"
cmd_cnt = 0
count = 0
flag = True
rostime = None
data = []

data =  ["S.no","X","Y","Z_theta"]

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


def callback_odom(msg):

    global flag
    global count
    global rostime
    global data

    if flag:
        data_header = ["S.no","X","Y","Z_theta"]
        data_writer(data_header)
        flag = False

    data[1] =(msg.pose.pose.position.x)/(msg.pose.pose.orientation.w)
    data[2] =(msg.pose.pose.position.y)/(msg.pose.pose.orientation.w)
    data[3] =( msg.pose.pose.orientation.z )/(msg.pose.pose.orientation.w)

    data_writer(data)
 
def listener():

    # global fiducial
    global cmd_vel

    rospy.init_node('getter', anonymous=False)
    cmd_vel = rospy.Subscriber('/odom',Odometry, callback_odom,queue_size=25)
    rospy.spin()

if __name__ == '__main__':

    listener()


# Reference
# https://www.codegrepper.com/code-examples/python/python+convert+time+to+timestamp
# https://pythonguides.com/python-dictionary-to-csv/



