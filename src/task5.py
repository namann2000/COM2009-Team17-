#!/usr/bin/env python3
from os import system
import numpy as np
import rospy
import roslaunch
import pathlib
import cv2
from cv_bridge import CvBridge, CvBridgeError
import sys
from sensor_msgs.msg import Image, LaserScan
from statistics import mean
from geometry_msgs.msg import Twist

class Square:
    def __init__(self):
        node_name = "task5_node"
        self.epochs=0
        rospy.init_node(node_name)

        self.target_colour = sys.argv[0]
        self.mapped=False

        self.base_image_path = pathlib.Path.home().joinpath("catkin_ws/src/team17/src/snaps/")
        self.base_image_path.mkdir(parents=True, exist_ok=True)

        self.startup = True
        self.turn = False

        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.laser = rospy.Subscriber("scan", LaserScan, self.laser_function)
        self.camera = rospy.Subscriber("/camera/color/image_raw", Image, self.camera_cb)

        self.rate = rospy.Rate(20) # hz
        

        self.vel = Twist()

        self.frontDistance=0.0
        self.rightDistance=0.0
        self.leftDistance=0.0
        self.turnDirection="NONE"
        self.leftHazard=0.0
        self.rightHazard=0.0
        self.frontRight = 0.0
        self.frontLeft = 0.0



        self.ctrl_c = False

        self.cvbridge_interface = CvBridge()

        self.waiting_for_image = True

        rospy.on_shutdown(self.shutdownhook)
        rospy.loginfo(f"the {node_name} node has been initialised...")


    def show_and_save_image(self, img, img_name):
        self.full_image_path = self.base_image_path.joinpath(f"{img_name}.jpg")

        cv2.imshow(img_name, img)
        cv2.waitKey(0)

        cv2.imwrite(str(self.full_image_path), img)
        print(f"Saved an image to '{self.full_image_path}'\n"
            f"image dims = {img.shape[0]}x{img.shape[1]}px\n"
            f"file size = {self.full_image_path.stat().st_size} bytes")

    def camera_cb(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            pass

        if self.waiting_for_image == True:
            height, width, channels = cv_img.shape

            print(f"Obtained an image of height {height}px and width {width}px.")

            self.show_and_save_image(cv_img, img_name = "the_beacon")

            self.waiting_for_image = False
        
    def shutdownhook(self):
        self.pub.publish(Twist())
        self.ctrl_c = True
    
    def save_map(self):
        system("rosrun map_server map_saver -f ~/catkin_ws/src/team17/maps/task5_map ")
        

    def laser_function(self, laser_data):

        for i, value in enumerate(laser_data.ranges):
            if value==0:
                laser_data.ranges[i]=1000

        self.frontDistance = min(laser_data.ranges[:20] + laser_data.ranges[340:359])
        self.leftDistance = mean(laser_data.ranges[70:120])
        self.rightDistance = mean(laser_data.ranges[240:290])
        self.frontRight = (min(laser_data.ranges[320:340]) + mean(laser_data.ranges[320:340]))/2
        self.frontLeft = (min(laser_data.ranges[20:40]) + mean(laser_data.ranges[20:40]))/2


        self.minRight = min(laser_data.ranges[230:300])
        self.minLeft = min(laser_data.ranges[60:130])   

        self.rearRight=mean(laser_data.ranges[165:180])
        self.rearLeft=mean(laser_data.ranges[180:195])

        self.leftHazard=self.rearRight > 2*self.frontDistance
        self.rightHazard=self.rearLeft > 2*self.frontDistance

        self.dangerTotal=0
        clone=[]
        for i in range(360):
            clone.append(laser_data.ranges[i])
        clone.sort()
        self.dangerTotal=mean(clone[0:30])
        



    def main_loop(self):
        brexit=rospy.get_time()
        while not self.ctrl_c:
            if rospy.get_time() - brexit >100 and self.mapped == False:
                for i in range(20):
                    print("MAP")
                self.save_map()
                print("7")
                self.mapped=True
            if self.frontDistance > 0.6 and self.dangerTotal>0.12:
                if self.frontLeft > 0.6:
                    self.vel.angular.z = -0.1
                if self.frontRight > 0.6:
                    self.vel.angular.z = 0.1

                self.turnDirection="NONE"


                # more space in front
                
                self.vel.linear.x=0.085
                self.vel.angular.z = 0.01


            else:
                self.vel.linear.x=0

                if  self.turnDirection == "NONE":
                    if self.rightDistance > self.leftDistance:
                        self.turnDirection="RIGHT"

                    else:
                        self.turnDirection="LEFT"

                elif self.turnDirection == "RIGHT":
                    self.vel.angular.z = -0.6

                elif self.turnDirection == "LEFT":
                    self.vel.angular.z = 0.6

            self.epochs+=1
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
    #implement moveto in the morning xx