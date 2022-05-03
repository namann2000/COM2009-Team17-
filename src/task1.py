#!/usr/bin/env python3
import time
from xml.etree.ElementTree import PI
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
 
 
class Circle:
 
    def __init__(self):
        # When setting up the publisher, the "cmd_vel" topic needs to be specified
        # and the Twist message type needs to be provided
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber("odom", Odometry, self.callback)
        rospy.init_node('move_circle', anonymous=True)
        self.rate = rospy.Rate(10) # hz
 
        self.vel_cmd = Twist()
        self.counter = 0
 
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)
 
        rospy.loginfo("the 'move_circle' node is active...")
 
    def callback(self, topic_data):
        self.orientation_x = topic_data.pose.pose.orientation.x
        self.orientation_y = topic_data.pose.pose.orientation.y
        self.orientation_z = topic_data.pose.pose.orientation.z
        self.orientation_w = topic_data.pose.pose.orientation.w
 
        self.position_x = topic_data.pose.pose.position.x
        self.position_y = topic_data.pose.pose.position.y
        self.position_z = topic_data.pose.pose.position.z
 
        self.roll, self.pitch, self.yaw = euler_from_quaternion([self.orientation_x, 
                                self.orientation_y, self.orientation_z, self.orientation_w],
                                'sxyz')
 
    def shutdownhook(self):
        self.vel_cmd.linear.x = 0.0 # m/s
        self.vel_cmd.angular.z = 0.0 # rad/s
 
        print("stopping the robot")
 
        # publish to the /cmd_vel topic to make the robot stop
        self.pub.publish(self.vel_cmd)
 
        self.ctrl_c = True
 
    def main_loop(self):
         self.position_x = 0.00
         self.position_y = 0.00
         self.yaw = 0.0
         print(f" x=" + format(float(self.position_x),'.2f') + "[m], y=" + format(float(self.position_y),'.2f')+ " [m], yaw= " + format(float(math.degrees(self.yaw)),'.1f') + " [degrees]")
         
         startTime = time.time()
         while not self.ctrl_c :
            if time.time() - startTime < 31:
                #radius of circle 1:
                path_rad = 0.5 # m
 
                #set linear velocity (below 0.26m/s)
                lin_vel = (math.pi / 30) #m/s
 
                self.vel_cmd.linear.x = lin_vel
                self.vel_cmd.angular.z = lin_vel / path_rad # rad/s 
 
                self.pub.publish(self.vel_cmd)
            elif time.time() - startTime < 61:
                #radius of circle 2:
                path_rad = 0.5 # m
 
                #set linear velocity (below 0.26 m/s)
                lin_vel = (math.pi /30) #m/s
 
                self.vel_cmd.linear.x = lin_vel
                self.vel_cmd.angular.z = -lin_vel / path_rad 
 
                self.pub.publish(self.vel_cmd)
            else:
                self.ctrl_c = True
 
            if self.counter > 10:
                self.counter = 0
                print(f" x=" + format(float(self.position_x),'.2f') + "[m], y=" + format(float(self.position_y),'.2f')+ " [m], yaw= " + format(float(math.degrees(self.yaw)),'.1f') + " [degrees]")

            else:
                self.counter += 1
 
            self.rate.sleep()
 
if __name__ == '__main__':
    vel_ctlr = Circle()
    try:
        
        vel_ctlr.main_loop()
    except rospy.ROSInterruptException:
        pass