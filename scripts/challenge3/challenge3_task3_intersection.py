#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, cv2

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

from cam_detection import LineDetection, PointRallyingSpeed, T_Detection, obstacle_detection

from math import pi

flag=0
door_flag=0
choix_flag = 0
t0_flag = 0
t_final = 0



def emergency_stop(msg):
    twist=Twist()
    pub=rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    global flag
    
    if msg.ranges[0]<rospy.get_param("distance_stop"):
        if flag==0:
            print("Arret immediat, l'objet est à proximité")
            twist.linear.x=0
            pub.publish(twist)
            flag = 1
            #open_door(img)
    else:
        flag = 0

            


def set_speed(linear,angular):
    # Publish the linear and angular speed wanted in the cmd_vel topic
    pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
    twist = Twist()
    twist.linear.x = linear
    twist.angular.z = angular
    pub.publish(twist)
    
    
def open_door(img):
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(img, "bgr8")
    hsv=cv2.cvtColor(image,cv2.COLOR_RGB2HSV)
    image = cv2.GaussianBlur(hsv,(5,5),0)
    blue_pts = LineDetection(image,'blue',colorformat="hsv")
    
    global door_flag
    global as_opened
    
    if len(blue_pts)>=5 and flag == 1 and as_opened == 1:
        pub=rospy.Publisher('/Garage_Door_Opener',Bool,queue_size=3)
        var = True
        pub.publish(var)
        print("Ouverture Porte")
        door_flag = 1

    

def line_follower(img):
    global flag
    global choix_flag
    global t0_flag
    global t_final
    # using OpenCV
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(img, "bgr8")
    hsv=cv2.cvtColor(image,cv2.COLOR_RGB2HSV)
    image = cv2.GaussianBlur(hsv,(5,5),0) # we blur the image to reduce noise
	
    # detecting red, yellow and green points of the image
    yellow_pts = LineDetection(image,'yellow',colorformat="hsv")
    red_pts = LineDetection(image,'red',colorformat="hsv")
    green_pts = LineDetection(image,'green',colorformat="hsv")
   
    max_points = max([yellow_pts, red_pts, green_pts], key=len) # detect the dominant color of the image
    
    #print(T_Detection(image))
    
    
    if T_Detection(image) == True or choix_flag == 1:
        #choix_chemin(image)
        if choix_flag ==0:
            t0_flag = rospy.get_time()
            print("0 seconde")
            choix_flag = 1
        t_final = rospy.get_time()
        if t_final - t0_flag < 1:
            set_speed(0,pi/4)
            #rospy.sleep(1)
        else:
            print("1 seconde\n")
            set_speed(0,0)
            #print("choix_flag =   0 a 1\n")
            
            if obstacle_detection(image) == True:
                pass
                #set_speed(0,-pi/2)
                #rospy.sleep(1)
            #set_speed(1,0)
            #rospy.sleep(0.3)
                choix_flag = 0
            else:
                set_speed(2,0)
                choix_flag = 0
            
                print("choix_flag =   1 a 0\n")
            
 
    elif len(max_points)>=5 and flag==0:
        if max_points == yellow_pts:
            linear,angular = PointRallyingSpeed(yellow_pts[0],yellow_pts[4], rospy.get_param("speed_factor"),image.shape) # if the line is yellow : set speed to the speed parameter
            set_speed(linear,angular)
        elif max_points == red_pts:
            linear,angular = PointRallyingSpeed(red_pts[0],red_pts[4], rospy.get_param("speed_factor")/2, image.shape) # if the line is red : set speed to half the speed parameter
            set_speed(linear,angular)
        elif max_points == green_pts:
            if len(yellow_pts) < 2: # to make sure the robot stops in the green zone
                set_speed(0,0)
        
            
    else:
        set_speed(0,0) # the robot won't move if there is no line detected
    
        
    
def choix_chemin(image):
    global choix_flag
    
    if choix_flag ==0:
        print("0 seconde")
        set_speed(0,pi/4)
        rospy.sleep(1)
        print("1 seconde\n")
        set_speed(0,0)
        choix_flag = 1
        print("choix_flag =   0 a 1\n")
    else:
        if obstacle_detection(image) == True:
            pass
            #set_speed(0,-pi/2)
            #rospy.sleep(1)
        #set_speed(1,0)
        #rospy.sleep(0.3)
        choix_flag = 0
        print("choix_flag =   1 a 0\n")
        
    







if __name__ == '__main__':
    global as_opened
    as_opened = 1
    rospy.init_node('final_follower')
    sub_cam = rospy.Subscriber("/camera/image_raw",Image,line_follower, queue_size=1)
    sub_cam2 = rospy.Subscriber("/camera/image_raw",Image,open_door)
    #sub_cam3 = rospy.Subscriber("/camera/image_raw",Image,choix_chemin)
    sub_laser = rospy.Subscriber('/scan',LaserScan,emergency_stop)
    rospy.spin()
