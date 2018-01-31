#!/usr/bin/env python2
import rospy
import tf

# Custom messages
from leg_tracker.msg import Person, PersonArray, Leg, LegArray 
from visualization_msgs.msg import Marker

from geometry_msgs.msg import PoseWithCovarianceStamped, Polygon, Point32

import collections
import numpy as np

class ExtractorBoundingBox:    
    def __init__(self):

        self.points = collections.deque(maxlen=2)
        self.tf_listener = tf.TransformListener()
        self.marker_pub = rospy.Publisher('distance_Legs', Marker, queue_size=300)

        #self.f = open("/home/airlab/Desktop/file.txt", "w")

        # ROS subscribers         
        self.sub = rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.initialPoseCallback)      

        # ROS publishers
        self.pub = rospy.Publisher('bounding_box', Polygon, queue_size=1)

        rospy.spin() # So the node doesn't immediately shut down

    
    def initialPoseCallback(self, msg):
        self.points.append(msg)
        rospy.logwarn("#points in buffer: {}/2 (drops as in FIFO)".format(len(self.points)))
    
        if len(self.points) == 2:
            
            polygon = Polygon()
            p1 = Point32();
            p1.x = self.points[0].pose.position.x
            p1.y = self.points[0].pose.position.y
            p2.x = self.points[1].pose.position.x
            p2.y = self.points[1].pose.position.y

            polygon.points.append(p1)
            p12 = Point32();
            p12.x = 
            polygon.points.append(p1)




if __name__ == '__main__':
    rospy.init_node('leg_distance_extractor_bounding_box', anonymous=True)
    bd = ExtractorBoundingBox()