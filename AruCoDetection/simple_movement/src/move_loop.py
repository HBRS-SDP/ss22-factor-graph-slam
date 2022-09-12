#! /usr/bin/env python

from calendar import c
from re import X
from tkinter import Y
import rospy
from geometry_msgs.msg import Point, Twist
from tf.transformations import euler_from_quaternion
from math import atan2
import sys


class square():

    def __init__(self):
        rospy.on_shutdown(self.cleanup)

        self.pub = rospy.Publisher('cmd_vel', Twist)

        rospy.sleep(1)
        r = rospy.Rate(5.0)

        while not rospy.is_shutdown():
            twist = Twist()
            twist.linear.x = 0.15
            for i in range(61):
                self.pub.publish(twist)
                r.sleep()

            twist = Twist()
            twist.angular.z = -1.615/2
            for i in range(10):
                self.pub.publish(twist)
                r.sleep()
    def cleanup(self):
        twist = Twist()
        self.pub.publish(twist)
if __name__ =="__main__":
    rospy.init_node('square')
    square()





# rospy.init_node('topic_publisher')
# pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
# rate = rospy.Rate(2)
# move = Twist()
# move.linear.x = 0.1
# move.linear.z = 0.1

# while not rospy.is_shutdown():
#     pub.publish(move)
#     rate.sleep()



# x = 0.0
# y = 0.0
# theta = 0.0

# def newOdom(msg):
#     global x
#     global y
#     global theta

#     x = msg.pose.pose.position.x
#     y = msg.pose.pose.position.y


#     rot_q = msg.pose.pose.orientation
#     (roll,pitch,theta)= euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


# rospy.init_node("speed_controller")

# sub = rospy.Subscriber("/odom", newOdom)
# pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

# speed = Twist()

# r = rospy.Rate(4)

# goal = Point()
# goal.x = 2
# goal.y = 2

# while not rospy.is_shutdown():
#     inc_x = goal.x - x
#     inc_y = goal.y - y

#     angle_to_goal = atan2 (inc_y, inc_x)
#     if abs(angle_to_goal - theta) > 0.1:
#         speed.linear.x(0.0)
#         speed.angular.z(0.3)
#     else:
#         speed.linear.x(0.5)
#         speed.angular.z(0.0)
    
#     pub.publish(speed)
#     r.sleep()