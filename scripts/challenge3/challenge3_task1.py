#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, cv2

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

from cam_detection import LineDetection, PointRallyingSpeed

Stop_flag=0
door_flag=0



def emergency_stop(msg):
    #reprise de l'emergency_stop task1_challenge2
    twist=Twist()
    pub=rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    global Stop_flag
    
    #si mur detecté devant à la distance de sécurité on arrête le suiv dse ligne et immobilise le robot
    if msg.ranges[0]<rospy.get_param("distance_stop"):
        if Stop_flag==0:
            print("Mur detecté, arret d'urgence bip bip bip")
            twist.linear.x=0
            pub.publish(twist)
            Stop_flag = 1
            
    else:
        Stop_flag = 0

            


def set_speed(linear,angular):
    # Publish the linear and angular speed wanted in the cmd_vel topic
    pub = rospy.Publisher("/cmd_vel",Twist,queue_size=5)
    twist = Twist()
    twist.linear.x = linear
    twist.angular.z = angular
    pub.publish(twist)
    
    
def open_door(img):
    #fonction d'ouverture de la porte qu se souscrit à la caméra
    
    #traitement de l'image
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(img, "bgr8")
    hsv=cv2.cvtColor(image,cv2.COLOR_RGB2HSV)
    image = cv2.GaussianBlur(hsv,(5,5),0)
    blue_pts = LineDetection(image,'blue',colorformat="hsv")
    
    global door_flag
    #as_opened = 1 si la porte n 'a pas été ouverte
    
    #si l'on detecte une ligne bleue, cela signifie que l'on est devant la porte
    if len(blue_pts)>=5 and Stop_flag == 1:
        
        #envoi du message approprié sur le topic d'ouverture de la porte bleue
        pub=rospy.Publisher('/Garage_Door_Opener',Bool,queue_size=3)
        var = True
        pub.publish(var)
        print("Ouverture Porte")
        door_flag = 1
        #as_opened = 0

    

def line_follower(img):
    global Stop_flag
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
    
    if len(max_points)>=5 and Stop_flag==0:
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







if __name__ == '__main__':

    global door_flag 
    door_flag = 1
    rospy.init_node('line_and_obstacle')
    sub_cam = rospy.Subscriber("/camera/image_raw",Image,line_follower)
    sub_cam2 = rospy.Subscriber("/camera/image_raw",Image,open_door)
    sub_laser = rospy.Subscriber('/scan',LaserScan,emergency_stop)
    rospy.spin()
