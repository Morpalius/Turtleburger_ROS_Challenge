#!/usr/bin/env python
# -*- coding: utf-8 -*-

#author 3704957
#code emergency_stop repris du tp2

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def emergency_stop(msg):
    twist=Twist()
    global stop_flag

    # si le robot n'est pas à l'arret il avance à vitesse constante
    if stop_flag==0:
        twist.linear.x=rospy.get_param("linear_scale")
        pub.publish(twist)
    #condition de detection du mur en face du robot
    if msg.ranges[0]<rospy.get_param("distance_stop"):
	#s'il n'est pas à l'arret on l'arrete par changement du stop_flag, sinon non ne fait rien car déjà à l'arret
        if stop_flag == 0 :
            print("Obstacle détecté, arret d'urgence")
            stop_flag=1
            twist.linear.x=0
            pub.publish(twist)


stop_flag=0 # booléen flag qui informe de l'etat d'arret du robot (1 = arret, 0 = peut bouger)

rospy.init_node('stop_turtlebot')
pub=rospy.Publisher('/cmd_vel', Twist, queue_size=10)

sub=rospy.Subscriber('/scan',LaserScan,emergency_stop)
rospy.spin()
