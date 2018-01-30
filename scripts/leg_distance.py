#!/usr/bin/env python2
import rospy
import tf

# Custom messages
from leg_tracker.msg import Person, PersonArray, Leg, LegArray 
from visualization_msgs.msg import Marker

from munkres import Munkres, print_matrix # For the minimum matching assignment problem. To install: https://pypi.python.org/pypi/munkres 
import numpy as np

class LegDistance:    
    def __init__(self):

        self.tf_listener = tf.TransformListener()
        self.marker_pub = rospy.Publisher('distance_Legs', Marker, queue_size=300)

        self.f = open("/home/airlab/Desktop/file.txt", "w")

        # ROS subscribers         
        self.detected_clusters_sub = rospy.Subscriber('detected_leg_clusters', LegArray, self.detected_clusters_callback)      

        rospy.spin() # So the node doesn't immediately shut down

    
    def getRobotPose(self, player_tf='/player_filtered_link'):
        """
        Gets player position.
        OUTPUTS:
        @ a 3D-numpy array defined as: [x, y, theta] w.r.t /map.
        """
        try:
            self.tf_listener.waitForTransform('/map',player_tf, rospy.Time(0), rospy.Duration(1.0))
            trans, rot = self.tf_listener.lookupTransform('/map',player_tf, rospy.Time(0))
            # transform from quaternion to euler angles
            euler = tf.transformations.euler_from_quaternion(rot)
 
            return np.array([trans[0], trans[1], euler[2]])   # [xR,yR,theta]

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Navigation node: " + str(e))


    def detected_clusters_callback(self, detected_clusters_msg):    
        """
        Callback for every time detect_leg_clusters publishes new sets of detected clusters. 
        It will try to match the newly detected clusters with tracked clusters from previous frames.
        """

        now = detected_clusters_msg.header.stamp
       
        for cluster in detected_clusters_msg.legs:
            # publish rviz markers       
            marker = Marker()
            marker.header.frame_id = '/map'
            marker.header.stamp = now
            marker.ns = "detected_legs"                       
            marker.type = Marker.SPHERE
            marker.scale.x = cluster.position.x
            marker.scale.y = cluster.position.y
            marker.scale.z = 0.2                
            marker.pose.position.z = 1.5                        
            

            # Publish to rviz and /people_tracked topic.
            self.marker_pub.publish(marker)
        
        z = np.array([[complex(c.position.x, c.position.y) for c in detected_clusters_msg.legs]]) # notice the [[ ... ]]
        out = abs(z.T-z)
        out[out == 0] = float("inf")
        #print np.array_str(out, precision=2, suppress_small=True)

        # Run munkres on match_dist to get the lowest cost assignment
        #munkres = Munkres()
        #indexes = munkres.compute(out)
        for i in list(out[:,0])[1:]:
            self.f.write(str(i)+'\n')
        
        


if __name__ == '__main__':
    rospy.init_node('leg_distance_node', anonymous=True)
    ldistance = LegDistance()