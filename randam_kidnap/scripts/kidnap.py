#! /usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs/PointStamped import point

class RandamKidnap:
    def __init__(self):
        rospy.init_node('RandamKidnap',anonymous==True)
        rospy.Subscriber("clicked_point", point, callback)

def callback(data):
    rospy.INFO("%lf",data.poitn.x)

if __name__ =='__main__':
    kidnap = RandamKidnap()
