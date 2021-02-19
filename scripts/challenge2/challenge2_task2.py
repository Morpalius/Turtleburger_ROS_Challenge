#!/usr/bin/env python
# -*- coding: utf-8 -*-

#auteur 3704957

#lidar distance servoing

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

twist = Twist()
def distance_serv(msg):
    """
    the main idea of this code is to implement a simple P corrector in a first place to make sure
    that this algoprithm is working , once the security distance is reached, the robot corrects its front position
    in order to maintain that distance
    """

    global twist
    #command gain for proportional controler used in the later distance servoing
    Kp = 2
    distance=rospy.get_param("distance")

    #distance servoing used if distance between robot and wall are not the same 
    if distance != msg.ranges[0]:
        print(msg.ranges[0])
	#proportional correction of robot's speed based on the gap between the real and desired distance
        twist.linear.x= Kp *(msg.ranges[0]-distance) 
        pub.publish(twist)



rospy.init_node('lidar_servo_turtlebot')
sub=rospy.Subscriber('/scan',LaserScan,distance_serv)
pub=rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rospy.spin()
