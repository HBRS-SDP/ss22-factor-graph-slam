from cmath import nan
from pickle import FALSE
import time

from pytz import common_timezones
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import os

from nav_msgs.msg import Odometry
from vision_msgs.msg import Detection2DArray
from datetime import datetime
from csv import writer


path = "/home/ravi/Desktop/Work/semester_2/sdp/Sdp_factor_graphs/gtsam/data_collection/Extracted_data_from_bag_files/sdp_data_collection_"+"look_two_full_loop"+"_cmd_vel"+".csv"
count = 0
cmd_cnt = 0
flag = True
rostime = None
data = []

data =  ["S.no","time_stamp","X","Y","Z","X_O","Y_O","Z_O"]

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
    data_to_write = []

def callback_cmd_vel(msg):

    global flag
    global count
    global rostime
    global data


    if flag:

        data_header =  ["S.no","time_stamp","X","Y","Z","X_O","Y_O","Z_O"]
        data_writer(data_header)

        flag = False

    data[1] = str(datetime.fromtimestamp(rospy.get_rostime().secs))+"."+str(rospy.get_rostime().nsecs)

    data[-6] = msg.linear.x
    data[-5] = msg.linear.y
    data[-4] = msg.linear.z

    data[-3] = msg.angular.x
    data[-2] = msg.angular.y
    data[-1] = msg.angular.z

    data_writer(data)
 
def listener():

    # global fiducial
    global cmd_vel

    rospy.init_node('getter', anonymous=False)
    cmd_vel = rospy.Subscriber('/cmd_vel',Twist, callback_cmd_vel,queue_size=25)
    rospy.spin()

if __name__ == '__main__':

    listener()


# Reference
# https://www.codegrepper.com/code-examples/python/python+convert+time+to+timestamp
# https://pythonguides.com/python-dictionary-to-csv/


