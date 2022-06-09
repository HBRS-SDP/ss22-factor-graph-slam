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



path = "/home/ravi/Desktop/Work/semester_2/sdp/Sdp_factor_graphs/gtsam/data_collection/Extracted_data_from_bag_files/sdp_data_collection_"+str(time.time())+".csv"
count = 0
cmd_cnt = 0
flag_fidu = True
flag_cmd = True
rostime = None
# common_access_flag = [True,True]
data_to_write = []


def data_writer(data_to_write):

    with open(path, 'a+') as f_object:

        writer_object = writer(f_object)
        writer_object.writerow(data_to_write)
        f_object.close()

    data_to_write = []


def callback_fiducial_transform(msg):

    global count
    global value
    global flag_fidu
    global data_to_write
    global fiducial
    global rostime

    count += 1

    data_dict = {'S.no' : [],
                'aruco_id': [],
                'Timestamp': [],
                'score':[],
                'p_x':[],
                'p_y':[],
                'p_z':[],
                'o_x':[],
                'o_y':[],
                'o_z':[],
                'o_w':[]}

    detections = msg.detections
    header = msg.header
    rostime = datetime.fromtimestamp(header.stamp.secs)

    if flag_fidu:
        data = list(data_dict.keys())
        data_writer(data)
        data = []
        flag_fidu = False


    if detections == []:

        data = [count,0,rostime,0,0,0,0,0,0,0,0]
        data_writer(data)


    elif detections[0].results[0].score*100 > 10:

        data = []

        data.append(count)
        data.append(detections[0].results[0].id)
        data.append(rostime)
        data.append(detections[0].results[0].score)
        data.append(detections[0].results[0].pose.pose.position.x)
        data.append(detections[0].results[0].pose.pose.position.y)
        data.append(detections[0].results[0].pose.pose.position.z)
        data.append(detections[0].results[0].pose.pose.orientation.x)
        data.append(detections[0].results[0].pose.pose.orientation.y)
        data.append(detections[0].results[0].pose.pose.orientation.z)
        data.append(detections[0].results[0].pose.pose.orientation.w)

        data_writer(data)
        data = []

def callback_cmd_vel(msg):

    global flag_cmd
    global cmd_cnt

    data = []
    if flag_cmd:

        data = [count,0,rostime,0,0,0,0,0,0,0,0,'S.no','x_linear','y_linear','z_linear','x_angular','y_angular','z_angular']
        flag_cmd = False
        data_writer(data)
        data = []

    data = [count,0,rostime,0,0,0,0,0,0,0,0]
    cmd_cnt += 1
    data.append(cmd_cnt)
    data.append(msg.linear.x)
    data.append(msg.linear.y)
    data.append(msg.linear.z)

    data.append(msg.angular.x)
    data.append(msg.angular.y)
    data.append(msg.angular.z)

    data_writer(data)
    data = []


 
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


