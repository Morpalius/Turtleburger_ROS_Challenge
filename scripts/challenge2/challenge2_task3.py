#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

twist = Twist()
def omni_distance_servo(msg):
    global twist
    distance=rospy.get_param("distance")
    K_p = 1
    angle_wall = np.argmin(msg.ranges) #determination de l'angle du mur par rapport à la direction du robot
    print("angle distance minimale : ",angle_wall)
    print("distance au robot : ",angle_wall)
    if (angle_wall<90 or angle_wall>269):
        
        #selon la valeur de l'angle trouvé, on choisit de tourner vers la droite ou vers la gauche au cas le robot n'est pas face au mur 
        if angle_wall < 180:
            twist.angular.z = 1    
        if angle_wall >= 180:
            twist.angular.z = -1
        pub.publish(twist)
        
        #reprise de l'asservissement en vitesse utilisé dans le task2 avec la distance minimale trouvée avec le mur cette fois ci
        if distance != np.min(msg.ranges):
            twist.linear.x=K_p *(np.min(msg.ranges)-distance)
        pub.publish(twist) 



rospy.init_node('lidar_servo_omni')
sub=rospy.Subscriber('/scan',LaserScan,omni_distance_servo)
pub=rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rospy.spin()

