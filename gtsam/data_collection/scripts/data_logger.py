from cmath import nan
from pickle import FALSE
import time

from pytz import common_timezones
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import os
from vision_msgs.msg import Detection2DArray
from datetime import datetime
from csv import writer



path = "/home/ravi/Desktop/Work/semester_2/sdp/Sdp_factor_graphs/gtsam/data_collection/Extracted_data_from_bag_files/sdp_data_collection_"+"look_one_3_marker"+".csv"
count = 0
cmd_cnt = 0
flag = True
rostime = None


data =  [count,'-',rostime,'-','-','-','-','-','-','-','-','-','-','-','-','-','-']


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



def callback_fiducial_transform(msg):

    global count
    global flag
    global rostime

    global data 

    detections = msg.detections
    header = msg.header
    rostime = datetime.fromtimestamp(header.stamp.secs)

    if flag:
        data_header = ['S.no', 'aruco_id', 'Timestamp', 'score', 'p_x', 'p_y', 'p_z', 'o_x', 'o_y', 'o_z', 'o_w','x_linear','y_linear','z_linear','x_angular','y_angular','z_angular']
        data_writer(data_header)
        flag = False


    if detections == []:

        data[1] = "-"
        data[2] = rostime
        data[3] = "-"
        data[4] = "-"
        data[5] = "-"
        data[6] = "-"
        data[7] = "-"
        data[8] = "-"
        data[9] = "-"
        data[10] = "-"

    elif detections[0].results[0].score*100 > 10:

        data[1] = (detections[0].results[0].id)
        data[2] = rostime
        data[3] = detections[0].results[0].score
        data[4] = detections[0].results[0].pose.pose.position.x
        data[5] = detections[0].results[0].pose.pose.position.y
        data[6] = detections[0].results[0].pose.pose.position.z
        data[7] = detections[0].results[0].pose.pose.orientation.x
        data[8] = detections[0].results[0].pose.pose.orientation.y
        data[9] = detections[0].results[0].pose.pose.orientation.z
        data[10] = detections[0].results[0].pose.pose.orientation.w

        data_writer(data)

def callback_cmd_vel(msg):

    global flag
    global count
    global rostime
    global data


    if flag:

        data_header = ['S.no', 'aruco_id', 'Timestamp', 'score', 'p_x', 'p_y', 'p_z', 'o_x', 'o_y', 'o_z', 'o_w','x_linear','y_linear','z_linear','x_angular','y_angular','z_angular']
        data_writer(data_header)

        flag = False

    data[-6] = msg.linear.x
    data[-5] = msg.linear.y
    data[-4] = msg.linear.z

    data[-3] = msg.angular.x
    data[-2] = msg.angular.y
    data[-1] = msg.angular.z

    data_writer(data)
 
def listener():

    global fiducial
    global cmd_vel

    rospy.init_node('getter', anonymous=False)
    cmd_vel = rospy.Subscriber('/cmd_vel',Twist, callback_cmd_vel,queue_size=25)
    fiducial = rospy.Subscriber("/fiducial_transforms", Detection2DArray, callback_fiducial_transform,queue_size = 25)

    rospy.spin()


if __name__ == '__main__':

    listener()




# Reference
# https://www.codegrepper.com/code-examples/python/python+convert+time+to+timestamp
# https://pythonguides.com/python-dictionary-to-csv/


