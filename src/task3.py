#!/usr/bin/env python3

from statistics import mean
import rospy
# import the Twist message for publishing velocity commands:
from geometry_msgs.msg import Twist
# import the Odometry message for subscribing to the odom topic:
from nav_msgs.msg import Odometry
# import the function to convert orientation from quaternions to angles:
from tf.transformations import euler_from_quaternion
# import some useful mathematical operations (and pi), which you may find useful:

from sensor_msgs.msg import LaserScan

class Square:
    def __init__(self):
        node_name = "move_square"


        # a flag if this node has just been launched
        self.startup = True

        # This might be useful in the main_loop() to switch between turning and moving forwards...?
        self.turn = False

        # setup a cmd_vel publisher and an odom subscriber:
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("odom", Odometry, self.callback_function)
        self.laser = rospy.Subscriber("scan", LaserScan, self.laser_function)

        rospy.init_node(node_name, anonymous=True)
        self.rate = rospy.Rate(300) # hz

        # define the robot pose variables and set them all to zero to start with:
        # variables to use for the "current position":
        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        # variables to use for the "reference position":
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0
        
        # define a Twist instance, which can be used to set robot velocities
        self.vel = Twist()

        # define scan variables
        self.frontDistance=0.0
        self.rightDistance=0.0
        self.leftDistance=0.0
        self.turnDirection="NONE"


        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo(f"the {node_name} node has been initialised...")

    def shutdownhook(self):
        # publish an empty twist message to stop the robot
        # (by default all velocities will be zero):
        self.pub.publish(Twist())
        self.ctrl_c = True
    

    def laser_function(self, laser_data):
        self.frontDistance = min(laser_data.ranges[:20] + laser_data.ranges[340:359])
        self.leftDistance = mean(laser_data.ranges[70:120])
        self.rightDistance = mean(laser_data.ranges[240:290])

        self.minRight = min(laser_data.ranges[230:300])
        self.minLeft = min(laser_data.ranges[60:130])


    def callback_function(self, odom_data):
        # obtain the orientation co-ords:
        or_x = odom_data.pose.pose.orientation.x
        or_y = odom_data.pose.pose.orientation.y
        or_z = odom_data.pose.pose.orientation.z
        or_w = odom_data.pose.pose.orientation.w

        # obtain the position co-ords:
        pos_x = odom_data.pose.pose.position.x
        pos_y = odom_data.pose.pose.position.y

        # convert orientation co-ords to roll, pitch & yaw (theta_x, theta_y, theta_z):
        (roll, pitch, yaw) = euler_from_quaternion([or_x, or_y, or_z, or_w], 'sxyz')
        
        # We are only interested in the x, y and theta_z odometry data for this
        # robot, so we only assign these to class variables (so that we can 
        # access them elsewhere within our Square() class):
        self.x = pos_x
        self.y = pos_y
        self.theta_z = yaw 

        # If this is the first time that this callback_function has run, then 
        # obtain a "reference position" (used to determine how far the robot has moved
        # during its current operation)
        if self.startup:
            # don't initialise again:
            self.startup = False
            # set the reference position:
            self.x0 = self.x
            self.y0 = self.y
            self.theta_z0 = self.theta_z

    
    
    def print_stuff(self, a_message):
        # a function to print information to the terminal (use as you wish):
        # print the message that has been passed in to the method via the "a_message" input:
        print(a_message)
        
        # you could use this to print the current velocity command:
        #print(f"current velocity: lin.x = {self.vel.linear.x:.1f}, ang.z = {self.vel.angular.z:.1f}")
        # you could also print the current odometry to the terminal here, if you wanted to:
        #print(f"current odometry: x = {self.x:.3f}, y = {self.y:.3f}, theta_z = {self.theta_z:.3f}")


    def main_loop(self):
        while not self.ctrl_c:
            # here is where your code would go to control the motion of your 
            # robot. Add code here to make your robot move in a square of
            # dimensions 0.5x0.5m...

            
            ##########################


            if self.frontDistance > 0.535:
                self.turnDirection="NONE"
                self.print_stuff("Forward distance {}".format(self.frontDistance))


                # more space in front
                
                self.vel.linear.x=0.25
                self.vel.angular.z = 0


            else:
                self.vel.linear.x=0

                if  self.turnDirection == "NONE":
                    if self.rightDistance > self.leftDistance:
                        self.turnDirection="RIGHT"
                        self.print_stuff("TURN DECISION RIGHT")

                    else:
                        self.turnDirection="LEFT"
                        self.print_stuff("TURN DECISION LEFT")

                elif self.turnDirection == "RIGHT":
                    self.vel.angular.z = -0.6
                elif self.turnDirection == "LEFT":
                    self.vel.angular.z = 0.6




            

            ##########################
            
            
            # publish whatever velocity command has been set in your code above:
            self.pub.publish(self.vel)
            # call a function which prints some information to the terminal:
            #self.print_stuff("this is a message that has been passed to the 'print_stuff()' method")
            # maintain the loop rate @ 10 hz
            self.rate.sleep()

if __name__ == '__main__':
    movesquare_instance = Square()
    try:
        movesquare_instance.main_loop()
    except rospy.ROSInterruptException:
        pass