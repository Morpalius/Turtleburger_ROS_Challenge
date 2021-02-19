#!/usr/bin/env python
# -*- coding: utf-8 -*-



import math

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import rospy
import sys, termios, tty
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy
from numpy import isinf 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from cam_detection import LineDetection, PointRallyingSpeed

pos_cam = ((0.03,0,0.135),(0,0.610865238,0))
fov_cam=1.3962634
height = 800
width = 800

def emergency_stop(msg):
    twist=Twist()
    pub=rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    global flag
    if msg.ranges[0]<rospy.get_param("distance_stop"):
        if flag==0:
            print("Mur détecté arret d'urgence bip bip bip")
            twist.linear.x=0
            pub.publish(twist)
            flag = 1
            
    else:
        flag = 0


def set_speed(linear,angular):
    # Publish the linear and angular speed wanted in the cmd_vel topic
    pub = rospy.Publisher("/cmd_vel",Twist,queue_size=5)
    twist = Twist()
    twist.linear.x = linear
    twist.angular.z = angular
    pub.publish(twist)


def line_follower(img):
    global flag
    global dans_tunnel
    # using OpenCV
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(img, "bgr8")
    hsv=cv2.cvtColor(image,cv2.COLOR_RGB2HSV)
    image = cv2.GaussianBlur(hsv,(5,5),0) # we blur the image to reduce noise
	
    # detecting red, yellow and green points of the image
    yellow_pts = LineDetection(image,'yellow',colorformat="hsv")
    red_pts = LineDetection(image,'red',colorformat="hsv")
    green_pts = LineDetection(image,'green',colorformat="hsv")
    blue_pts = LineDetection(image,'blue',colorformat="hsv")
    max_points = max([yellow_pts, red_pts, green_pts, blue_pts], key=len) # detect the dominant color of the image
    
    #print(flag)
    if len(max_points)>=5 and flag==0:
        #print("suivi de ligne en cours") 
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
    
    #le flag "dans_tunnel" permet de verifier si le robot n'est pas dans le tunnel. Car si il est dedans --> pas de ligne jaune , 
    #mais on ne veux pas que le robot s'arrete pour autant
        
        if dans_tunnel==0:
              twist = Twist()
              twist.linear.x = 0
              twist.angular.z=0
              pub2=rospy.Publisher('/cmd_vel',Twist,queue_size=3)
              pub2.publish(twist)
       # the robot won't move if there is no line detected

def open_door(img):
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(img, "bgr8")
    hsv=cv2.cvtColor(image,cv2.COLOR_RGB2HSV)
    image = cv2.GaussianBlur(hsv,(5,5),0)
    blue_pts = LineDetection(image,'blue',colorformat="hsv")
    
    global door_flag
    if len(blue_pts)>=5 and flag == 1:
        pub=rospy.Publisher('/Garage_Door_Opener',Bool,queue_size=3)
        var = True
        pub.publish(var)
        if door_flag == 1 : print("Ouverture Porte")
        door_flag = 0


def wall_follower(msg):  #Fonction du subscriber "sub_wall" utilisant les informations du lidar pour un suivi de murs dans un tunnel
  global flag #voir initialisation du flag dans le main
   
  global dans_tunnel #bool de controle d'etat initialisé dans la fonctioin main
  global door_flag #bool de controle de passage de la porte bleue
  twist=Twist()
  
  

  
  #detection des bords droit(-60°) et gauche (60°) indiquant le début du tunnel
  if msg.ranges[60]<0.7 and msg.ranges[300]<0.7 and door_flag == 0:
      dans_tunnel=1 # flag informant  de l'entrée du tunnel quand il est à 1
      print("entre dans le tunnel")
      door_flag=1 #reset du flag de passage de porte bleue
      

  #algorithme de mouvement dans le tunnel via lidar
  if dans_tunnel ==1:
      flag=0 #flag à 0 pour inhiber l'action depuis la fonction de suivi de ligne 
      print(msg.ranges[300]," "," ",msg.ranges[60], "door_flag =", door_flag)
      twist.linear.x=0.5
      pub2=rospy.Publisher('/cmd_vel',Twist,queue_size=3)
      
      #tourne a gauche si le coté droit est trop proche du mur
      if (msg.ranges[60])>(msg.ranges[300]):
        
        twist.angular.z=0.5
        pub2.publish(twist)

      #tourne a droite si le coté gauche est trop proche du mur  
      elif (msg.ranges[60])<(msg.ranges[300]):
        
        twist.angular.z=-0.5
        pub2.publish(twist)
      
      #avance tout droit si equidistance avec chaque mur respectée
      else :
        twist.angular.z=0
        pub2.publish(twist)
        print('tout droit')
      pub2.publish(twist)
      
      if isinf(msg.ranges[60]) and isinf(msg.ranges[300]) : 
      #detection de fin de tunnel par abscence des 2 murs sur les cotés
          print('fin du tunnel reprise du suivi de ligne')
          twist.linear.x=0
          dans_tunnel= 0
          pub2.publish(twist)
          flag = 0 #REPRISE DU SUIVI DE LIGNE
          
  if msg.ranges[0]>=rospy.get_param("distance_stop") and dans_tunnel == 1:
      
      #utilisé pour inhiber l'action du suivi de ligne et faire uniquement un suivi de murs
      flag = 1
      


if __name__ =='__main__':
  dans_tunnel=0 # flag qui permet de savoir si on a passé la tache 1 cad si on rentre dans le tunnel et qu'on arrete le suivi de trajectoire
  
  flag=0 #flag qui est = 1 si le robot doit bouger et =0 si il doit s'arreter
  door_flag = 1 #flag de controle de passage de la porte bleue 
  
  rospy.init_node('final_follower')
  sub_cam = rospy.Subscriber("/camera/image_raw",Image,line_follower)
  sub_cam2 = rospy.Subscriber("/camera/image_raw",Image,open_door)
  sub_laser = rospy.Subscriber('/scan',LaserScan,emergency_stop)
  sub_wall = rospy.Subscriber('/scan',LaserScan,wall_follower)
  rospy.spin()
