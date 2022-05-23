from pytz import common_timezones
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import os
from vision_msgs.msg import Detection2DArray
from datetime import datetime
from csv import writer



path = "sdp_look_one.csv"
count = 0
flag_fidu = True
flag_cmd = True
common_access_flag = [False,False]
data_to_write = []


def data_writer():

    global data_to_write
    global cmd_vel
    global common_access_flag


    if common_access_flag[0] and common_access_flag[1] == True:

        print("writing ....")

        with open(path, 'a') as f_object:
            writer_object = writer(f_object)
            writer_object.writerow(data_to_write)
            f_object.close()
        common_access_flag = [False,False]
        data_to_write = []


def callback_fiducial_transform(msg):

    global count
    global value
    global flag_fidu
    global data_to_write
    global fiducial

   
    data_dict = {#'frame_id' : [],
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

    if detections != [] and common_access_flag[0] == False :

        if detections[0].results[0].score*100 > 95:

            data = []

            if flag_fidu:
                data = list(data_dict.keys())
                flag_fidu = False

            else:

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

            for datum in data:
                data_to_write.append(datum)

            data = []
            common_access_flag[0] = True



def callback_cmd_vel(msg):

    global flag_cmd
    global data_to_write

    if common_access_flag[0] == True and common_access_flag[1] == False:
        data = []
        if flag_cmd:

            data = ['x_linear','y_linear','z_linear','x_angular','y_angular','z_angular']
            flag_cmd = False
        
        else:

            data.append(msg.linear.x)
            data.append(msg.linear.y)
            data.append(msg.linear.z)

            data.append(msg.angular.x)
            data.append(msg.angular.y)
            data.append(msg.angular.z)

        for datum in data:
            data_to_write.append(datum)

        data = []
        
        common_access_flag[1] = True
        data_writer()

 
def listener():

    global fiducial
    global cmd_vel

    rospy.init_node('getter', anonymous=False)
    cmd_vel = rospy.Subscriber('/cmd_vel',Twist, callback_cmd_vel)
    fiducial = rospy.Subscriber("/fiducial_transforms", Detection2DArray, callback_fiducial_transform)

    rospy.spin()


if __name__ == '__main__':

    listener()




# Reference
# https://www.codegrepper.com/code-examples/python/python+convert+time+to+timestamp
# https://pythonguides.com/python-dictionary-to-csv/


