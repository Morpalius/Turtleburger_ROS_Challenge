#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

from cam_detection import LineDetection, PointRallyingSpeed


def set_speed(linear,angular):
    """ Publie la vitesse linéaire et angulaire dans le paramètres /cmd_vel """
    pub = rospy.Publisher("/cmd_vel",Twist,queue_size=5)
    twist = Twist()
    twist.linear.x = linear
    twist.angular.z = angular
    pub.publish(twist)

def line_follower(img):
    """ Fonction utilisée pour le suivi de ligne, prends en paramètre l'image et envoie les commandes de vitesse à l'aide de set_speed """
    # utilisation de OpenCV
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(img, "bgr8")
    hsv=cv2.cvtColor(image,cv2.COLOR_RGB2HSV) # conversion en hsv
    image = cv2.GaussianBlur(hsv,(5,5),0) # on floute l'image pour réduire le bruit (floutage gaussien en hsv)
	
    # détection des points rouges, jaunes et verts
    yellow_pts = LineDetection(image,'yellow',colorformat="hsv")
    red_pts = LineDetection(image,'red',colorformat="hsv")
    green_pts = LineDetection(image,'green',colorformat="hsv")
    max_points = max([yellow_pts, red_pts, green_pts], key=len) # on prend la couleur dominante de l'image
    
    if len(max_points)>=5: # seuil d'exécution des fonctions (nombre total de points = 20)
        if max_points == yellow_pts:
            linear,angular = PointRallyingSpeed(yellow_pts[0],yellow_pts[4], rospy.get_param("speed_factor"),image.shape) # si la ligne est jaune, on va à la vitesse définie dans le launch file
            set_speed(linear,angular)
        if max_points == red_pts:
            linear,angular = PointRallyingSpeed(red_pts[0],red_pts[4], rospy.get_param("speed_factor")/2, image.shape) # si la ligne est jaune, on va à la moitié de la vitesse définie
            set_speed(linear,angular)
        if max_points == green_pts:
            if len(yellow_pts) < 2: # on s'arrête si la couleur est verte, une fois qu'on ne voit plus de jaune pour être sûr d'être dans la zone
                set_speed(0,0)
    else:
        set_speed(0,0) # si on ne voit pas de ligne, on ne bouge pas


if __name__ == '__main__':
    rospy.init_node('line_follow')
    sub = rospy.Subscriber("/camera/image_raw",Image,line_follower)
    rospy.spin()
