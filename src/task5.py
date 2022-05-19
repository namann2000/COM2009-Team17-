#!/usr/bin/env python3

import rospy
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
        rospy.init_node(node_name)

        self.target_colour = sys.argv[0]

        self.base_image_path = pathlib.Path.home().joinpath("catkin_ws/src/team17/src/snaps/")
        self.base_image_path.mkdir(parents=True, exist_ok=True)

        self.startup = True
        self.turn = False

        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.laser = rospy.Subscriber("scan", LaserScan, self.laser_function)
        self.camera = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_cb)

        self.rate = rospy.Rate(300) # hz
        
        self.vel = Twist()

        self.frontDistance=0.0
        self.rightDistance=0.0
        self.leftDistance=0.0
        self.turnDirection="NONE"

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
            print(e)

        if self.waiting_for_image == True:
            height, width, channels = cv_img.shape

            print(f"Obtained an image of height {height}px and width {width}px.")

            self.show_and_save_image(cv_img, img_name = "the_beacon")

            self.waiting_for_image = False
        
    def shutdownhook(self):
        self.pub.publish(Twist())
        self.ctrl_c = True
    

    def laser_function(self, laser_data):
        self.frontDistance = min(laser_data.ranges[:20] + laser_data.ranges[340:359])
        self.leftDistance = mean(laser_data.ranges[70:120])
        self.rightDistance = mean(laser_data.ranges[240:290])

        self.minRight = min(laser_data.ranges[230:300])
        self.minLeft = min(laser_data.ranges[60:130])    


    def print_stuff(self, a_message):
        print(a_message)


    def main_loop(self):
        while not self.ctrl_c:
            if self.frontDistance > 0.4:

                self.turnDirection="NONE"
                self.print_stuff("Forward distance {}".format(self.frontDistance))
                
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
                    self.vel.angular.z = -0.3
                elif self.turnDirection == "LEFT":
                    self.vel.angular.z = 0.3          
            
            self.pub.publish(self.vel)
            self.rate.sleep()

if __name__ == '__main__':
    movesquare_instance = Square()
    try:
        movesquare_instance.main_loop()
    except rospy.ROSInterruptException:
        pass
    #implement moveto in the morning xx