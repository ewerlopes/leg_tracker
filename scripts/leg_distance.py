#!/usr/bin/env python2
import rospy
import tf
import math
import copy
import itertools
import numpy as np
from scipy import spatial

# Custom messages
from player_tracker.msg import Person, PersonArray, Leg, LegArray, PersonEvidence, PersonEvidenceArray, TowerArray
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PolygonStamped, Point32
from munkres import Munkres, print_matrix # For the minimum matching assignment problem. To install: https://pypi.python.org/pypi/munkres 
from sklearn.externals import joblib

class LegContextProcessor: 
    '''Process pairs of leg clusters and weight them by their
    probability of being a human using a generative model trained
    by data..''' 
    
    def __init__(self, x_min, x_max, y_min, y_max, pub_bounding_box=False, logfile=''):

        self.model = joblib.load(rospy.get_param("leg_dist_model"))

        self.tf_listener = tf.TransformListener()

        self.pub_bounding_box = pub_bounding_box
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
        self.person_evidence_pub = rospy.Publisher('person_evidence_array', PersonEvidenceArray, queue_size=1)
        self.marker_pub = rospy.Publisher('detected_legs_in_bounding_box', Marker, queue_size=5)
        self.pub_towers = rospy.Publisher('estimated_tower_positions', TowerArray, queue_size=5)

        #rospy.spin() # So the node doesn't immediately shut down
        self.tower_positions, _ = self.get_tower_distances()
        self.tower_target_distances = set([])
        self.tower_triangle_areas = []
        for perm in itertools.permutations(self.tower_positions, r=3):
            dist1_2 = spatial.distance.euclidean(np.array([perm[0].x, perm[0].y]),np.array([perm[1].x, perm[1].y]))
            dist1_3 = spatial.distance.euclidean(np.array([perm[0].x, perm[0].y]),np.array([perm[2].x, perm[2].y]))
            dist2_3 = spatial.distance.euclidean(np.array([perm[1].x, perm[1].y]),np.array([perm[2].x, perm[2].y]))
            self.tower_target_distances.add(dist1_2)
            self.tower_target_distances.add(dist1_3)
            self.tower_target_distances.add(dist2_3)
            p = (dist1_2 + dist1_3 + dist2_3) / 2.0     # perimeter
            area = np.sqrt(p*(p-dist1_2)*(p-dist1_3)*(p-dist2_3))
            self.tower_triangle_areas.append(area)

        self.tower_triangle_areas = list(set(self.tower_triangle_areas))
        self.tower_target_distances = list(set(self.tower_target_distances))
        rospy.loginfo("Tower triangle areas: {}".format(self.tower_triangle_areas))
        rospy.loginfo(self.tower_target_distances)

    def publish_poligon(self, pts):
        '''Publish poligon for the bouding box'''
            
        polygon = PolygonStamped()
        polygon.header.stamp = rospy.get_rostime()
        polygon.header.frame_id = 'map'
        polygon.polygon.points = pts
        self.pub.publish(polygon)

    def get_trans_wrt_robot(self, tower):
        """
        Gets tower position with respect to base_link. That is, performs a TF transformation from 'tower_link' to /base_link and returns
        x,y and theta.
        Param:
            @tower the name of the tower tf.
        Returns:
            @ a 3D-numpy array defined as: [x, y, theta] w.r.t /map.
        """
        try:
            self.tf_listener.waitForTransform('/base_link', tower, rospy.Time(0), rospy.Duration(1.0))
            trans, rot = self.tf_listener.lookupTransform('/base_link', tower, rospy.Time(0))
            # transform from quaternion to euler angles
            euler = tf.transformations.euler_from_quaternion(rot)
 
            return np.array([trans[0], trans[1], euler[2]])   # [xR,yR,theta]

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Navigation node: " + str(e))

    def get_tower_distances(self, num_towers=4):
        """Calculate tower distances from each other in the robot frame"""
        # TODO: get num of towers from param server
        towers = []
        for t in range(1,num_towers+1):
            tower = self.get_trans_wrt_robot("tower_"+str(t))
            towers.append(Point(tower[0],tower[1],0))

        distances = []
        for t in range(num_towers):
            if t != num_towers-1:
                distances.append(round(spatial.distance.euclidean((towers[t].x,towers[t].y) , (towers[t+1].x,towers[t+1].y)),3))
            else:
                distances.append(round(spatial.distance.euclidean((towers[t].x,towers[t].y), (towers[0].x,towers[0].y)),3))
        return towers, distances

    def getPersonProbability(self,distance):
        '''Uses self.model to calculate the probability of the pair being a person'''
        return np.exp(self.model.score_samples(distance))

    def is_close_enough(self, num1, num2, tol=0.1):
        return abs(num1-num2) < tol

    def detected_clusters_callback(self, detected_clusters_msg):    
        """
        Callback for every time detect_leg_clusters publishes new sets of detected clusters. 
        It will try to match the newly detected clusters with tracked clusters from previous frames.
        """

        now = detected_clusters_msg.header.stamp
       
        accepted_clusters = []

        for i,cluster in enumerate(detected_clusters_msg.legs):
            
            in_bounding_box = True
            
            if self.pub_bounding_box:
                in_bounding_box = cluster.position.x > self.x_min_ and \
                                  cluster.position.x < self.x_max_ and \
                                  cluster.position.y > self.y_min_ and \
                                  cluster.position.y < self.y_max_
            
            if in_bounding_box:
                marker = Marker()
                marker.header.frame_id = self.fixed_frame
                marker.header.stamp = now
                marker.id = i
                # Text showing person's ID number 
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0
                marker.color.a = 1.0
                marker.type = Marker.TEXT_VIEW_FACING
                marker.text = "t: {}".format(str(now.to_sec()))
                marker.scale.z = 0.1         
                marker.pose.position.x = cluster.position.x
                marker.pose.position.y = cluster.position.y        
                marker.pose.position.z = 0.5 
                marker.lifetime = rospy.Duration(0.1)
                self.marker_pub.publish(marker)

                # publish rviz markers       
                marker = Marker()
                marker.header.frame_id = self.fixed_frame
                marker.header.stamp = now
                marker.id = i+100
                marker.ns = "detected_legs"                       
                marker.type = Marker.CYLINDER
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.01
                marker.color.r = 0
                marker.color.g = 1
                marker.color.b = 0
                marker.color.a = 1
                marker.pose.position.x = cluster.position.x
                marker.pose.position.y = cluster.position.y        
                marker.pose.position.z = 0.01                        
                marker.lifetime = rospy.Duration(0.1)

                # Publish to rviz and /people_tracked topic.
                self.marker_pub.publish(marker)
                
                # save cluster
                accepted_clusters.append(cluster)

        if len(accepted_clusters) <= 1:
            return
        
        tree = spatial.KDTree(np.array([[c.position.x, c.position.y] for c in accepted_clusters]))

        points_in_tree = len(tree.data)

        pair_set = set([])

        pos_dis_distances = [ Point(c.position.x, c.position.y, 0) for c in accepted_clusters]

        tower_array_msg = TowerArray()

        for j, pts in enumerate(tree.data):
            nearest_point = tree.query(pts, k=2)
            distance = nearest_point[0][1]

            prob =  self.getPersonProbability(distance)[0]

            pair = list(copy.deepcopy(nearest_point[1]))
            pair.sort()
            pair.append(prob) 
            pair = tuple(pair)
            pair_set.add(pair)
            
            # Save to file
            if self.isSaveToFile:   
                self.f.write(str(distance) +'\n')
                self.f.flush()

            for i, pt in enumerate([pts]):

                # publish rviz markers 
                marker = Marker()
                marker.header.frame_id = self.fixed_frame
                marker.header.stamp = now
                marker.id = j + i * 10
                marker.ns = "person"                       
                marker.type = Marker.SPHERE
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.2
                marker.color.r = prob
                marker.color.g = 0#poss_tower
                marker.color.b = 0
                marker.color.a = 1
                marker.pose.position.x = pt[0]
                marker.pose.position.y = pt[1]        
                marker.pose.position.z = 0.1                        
                marker.lifetime = rospy.Duration(0.1)

                # Publish to rviz and /people_tracked topic.
                self.marker_pub.publish(marker)
        

        if len(pos_dis_distances) > 3:
            for perm in itertools.permutations(pos_dis_distances, r=3):
                dist1_2 = spatial.distance.euclidean(np.array([perm[0].x, perm[0].y]),np.array([perm[1].x, perm[1].y]))
                dist1_3 = spatial.distance.euclidean(np.array([perm[0].x, perm[0].y]),np.array([perm[2].x, perm[2].y]))
                dist2_3 = spatial.distance.euclidean(np.array([perm[1].x, perm[1].y]),np.array([perm[2].x, perm[2].y]))
                triangle_distances = [dist1_2, dist1_3, dist2_3]
                p = (dist1_2 + dist1_3 + dist2_3) / 2.0     # perimeter
                
                area = np.sqrt(p*(p-dist1_2)*(p-dist1_3)*(p-dist2_3))
                
                for a in self.tower_triangle_areas:
                    if self.is_close_enough(a, area, tol=0.1):
                        to_pub = True

                        for side in triangle_distances:
                            if(not self.is_compatible_with_playground(side)):
                                to_pub = False
                                break

                        if to_pub:
                            tower_array_msg.towers.append(perm)
                            self.publish_poligon(perm)

        if len(tower_array_msg.towers) != 0:
            self.pub_towers.publish(tower_array_msg)

        evid_msg = PersonEvidenceArray()
        evid_msg.header.frame_id = self.fixed_frame
        evid_msg.header.stamp = now

        for item in pair_set:
            msg = PersonEvidence()
            msg.leg1.x = tree.data[item[0]][0]
            msg.leg1.y = tree.data[item[0]][1]
            msg.leg2.x = tree.data[item[1]][0]
            msg.leg2.y = tree.data[item[1]][1]
            msg.probability = item[2]
            evid_msg.evidences.append(msg)

        self.person_evidence_pub.publish(evid_msg)

    def is_compatible_with_playground(self, side_to_compare):
        for d in self.tower_target_distances:
            if(self.is_close_enough(side_to_compare, d, tol=0.1)):
                return True
        return False

if __name__ == '__main__':
    rospy.init_node('leg_distance_node', anonymous=True)
    rospy.logwarn('THE BOUDING BOX PARAMETERS SHOULD BE THE SAME AS THE ONES IN "extract_positive*.launch" file!')
    ldistance = LegContextProcessor(x_min=-2.76, x_max=2.47, y_min=-2.37, y_max=1.49)# logfile='/home/airlab/Desktop/newFile.txt')

    r = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        #ldistance.publish_poligon()
        r.sleep()