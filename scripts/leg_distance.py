#!/usr/bin/env python2
import rospy
import tf
import math
import copy

# Custom messages
from leg_tracker.msg import Person, PersonArray, Leg, LegArray 
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PolygonStamped, Point32

from munkres import Munkres, print_matrix # For the minimum matching assignment problem. To install: https://pypi.python.org/pypi/munkres 
import numpy as np

class LegDistance: 

    '''Calculates the distance between LEG clusters and saves to file.''' 
    
    def __init__(self, x_min, x_max, y_min, y_max, logfile="/home/airlab/Desktop/file.txt"):

        self.tf_listener = tf.TransformListener()
        self.marker_pub = rospy.Publisher('detected_legs_in_bounding_box', Marker, queue_size=300)

        self.isSaveToFile = logfile != ''
        self.fixed_frame = rospy.get_param('fixed_frame')

        if self.isSaveToFile:
            self.f = open(logfile, "w")
        

        # accepted area
        self.x_min_ = x_min
        self.x_max_ = x_max
        self.y_min_ = y_min
        self.y_max_ = y_max

        # ROS subscribers         
        self.detected_clusters_sub = rospy.Subscriber('detected_leg_clusters', LegArray, self.detected_clusters_callback)      

        # ROS publishers
        self.pub = rospy.Publisher('bounding_box', PolygonStamped, queue_size=1)

        #rospy.spin() # So the node doesn't immediately shut down

    
    def publish(self):
        '''Publish poligon for the bouding box'''
            
        polygon = PolygonStamped()
        polygon.header.stamp = rospy.get_rostime()
        polygon.header.frame_id = 'base_link'
        p1 = Point32()
        p1.x = self.x_min_
        p1.y = self.y_min_
        p2 = Point32()
        p2.x = self.x_max_
        p2.y = self.y_max_

        polygon.polygon.points.append(p1)
        p11 = Point32();
        p11.x = p1.x
        p11.y = p1.y + (p2.y - p1.y)
        p12 = Point32();
        p12.x = p1.x + (p2.x - p1.x)
        p12.y = p1.y
        polygon.polygon.points.append(p1)
        polygon.polygon.points.append(p11)
        polygon.polygon.points.append(p2)
        polygon.polygon.points.append(p12)
        polygon.polygon.points.append(p1)

        self.pub.publish(polygon)


    def detected_clusters_callback(self, detected_clusters_msg):    
        """
        Callback for every time detect_leg_clusters publishes new sets of detected clusters. 
        It will try to match the newly detected clusters with tracked clusters from previous frames.
        """

        now = detected_clusters_msg.header.stamp
       
        accepted_clusters = []

        for i,cluster in enumerate(detected_clusters_msg.legs):
            in_bounding_box = cluster.position.x > self.x_min_ and \
                              cluster.position.x < self.x_max_ and \
                              cluster.position.y > self.y_min_ and \
                              cluster.position.y < self.y_max_
            
            if in_bounding_box:
                # publish rviz markers       
                marker = Marker()
                marker.header.frame_id = self.fixed_frame
                marker.header.stamp = now
                marker.id = i
                marker.ns = "detected_legs"                       
                marker.type = Marker.SPHERE
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.2
                marker.color.r = 0
                marker.color.g = 1
                marker.color.b = 0
                marker.color.a = 1
                marker.pose.position.x = cluster.position.x
                marker.pose.position.y = cluster.position.y        
                marker.pose.position.z = 0.1                        
                marker.lifetime = rospy.Duration(0.1)

                # Publish to rviz and /people_tracked topic.
                self.marker_pub.publish(marker)

                # save cluster
                accepted_clusters.append(cluster)
        
        z = np.array([[complex(c.position.x, c.position.y) for c in accepted_clusters]]) # notice the [[ ... ]]

        
        try: 

            out = abs(z.T-z)
            out[np.triu_indices(out.shape[0])] = 500     # 500 is a big weight that replaces inf.
            out_backup = copy.deepcopy(out)
            print np.array_str(out, precision=2, suppress_small=True)

            # Run munkres on match_dist to get the lowest cost assignment
            munkres = Munkres()
            indexes = munkres.compute(out_backup)

            if self.isSaveToFile:
                N_people_hat = int(math.floor(len(accepted_clusters)/2))         # estimated number of people from number of legs inside bounding box.   
                    
                for row, column in indexes[1:N_people_hat+1]:
                    rospy.logerr('{}-{}'.format(row, column))
                    self.f.write(str(out[row][column])+'\n')
                    self.f.flush()

        except Exception as e:
            rospy.logwarn('Distance matrix shape: {}'.format(out.shape))
        


if __name__ == '__main__':
    rospy.init_node('leg_distance_node', anonymous=True)
    rospy.logwarn('THE BOUDING BOX PARAMETERS SHOULD BE THE SAME AS THE ONES IN "extract_positive*.launch" file!')
    ldistance = LegDistance(x_min=-2.76, x_max=2.47, y_min=-2.37, y_max=1.49)

    r = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        ldistance.publish()
        r.sleep()