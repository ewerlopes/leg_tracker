#!/usr/bin/python
import rospy
import numpy as np
import cv2

from nav_msgs.msg import OccupancyGrid

class BRIEFWrapper(object):
    def __init__(self):
        self.star = cv2.xfeatures2d.StarDetector_create() # Initiate FAST detector
        self.brief = cv2.xfeatures2d.BriefDescriptorExtractor_create() # Initiate BRIEF extractor

    def detectAndCompute(self, image):
        keypoints = self.star.detect(image, None)  # find the keypoints with STAR
        keypoints, descriptors = self.brief.compute(image, keypoints) # compute the descriptors with BRIEF
        return keypoints, descriptors


class PerceptionGridProjector(object):
    keypoints = {}
    descriptors = {}

    def __init__(self, detector_name='ORB'):
        if detector_name == 'ORB':
            self.detector = cv2.ORB_create()
            match_distance = cv2.NORM_HAMMING
        elif detector_name == 'BRIEF':
            self.detector = BRIEFWrapper()
            match_distance = cv2.NORM_HAMMING
        elif detector_name == 'SIFT':
            self.detector = cv2.xfeatures2d.SIFT_create()
            match_distance = cv2.NORM_L2
        elif detector_name == 'SURF':
            self.detector = cv2.xfeatures2d.SURF_create()
            match_distance = cv2.NORM_L2
        elif detector_name == 'FAST':
            self.detector = cv2.FastFeatureDetector()
            match_distance = cv2.NORM_L2

        # create BFMatcher object
        self.matcher = cv2.BFMatcher(match_distance, crossCheck=True)


    def compute_descriptors(self, image):
        kp, des = self.detector.detectAndCompute(image, None)
        return kp, des

    def compute_matches(self, des1, des2):
        matches = self.matcher.match(des2, des1) # inverted order beacuse the second is considered to be the ground-truth or traning
        return sorted(matches, key=lambda x: x.distance)

    def compute_transformation(self, matches, keypoints1, keypoints2):
        pts1, pts2 = self.extract_points(matches, keypoints1, keypoints2)
        return cv2.getAffineTransform(pts1, pts2)

    def project_points(self, M, points):
        rows = len(points)
        pts = np.reshape(np.float32(points), (rows, 2)) # reshape to be a matrix
        return cv2.warpAffine(pts, M)

    def project_point(self, M, point):
        pt = np.float32([point])
        return cv2.warpAffine(pt, M)

    def project_image(self, M, image):
        return cv2.warpAffine(image, M, image.shape)

    def extract_points(self, matches, kp1, kp2):
        pts1 = None
        pts2 = None

        for match in matches:
            pts1 = self.append_point_to(kp1[match.trainIdx], pts1)
            pts2 = self.append_point_to(kp2[match.queryIdx], pts2)
        return pts1[:3, :], pts2[:3, :]

    def append_point_to(self, keypoint, matrix):
        pt = list(keypoint.pt)
        if matrix is None:
            return np.float32([pt])
        else:
            return np.append(matrix, np.float32([pt]), axis=0)

    def store(self, image, kp, des):
        self.keypoints[image] = kp
        self.descriptors[image] = des

    def clear(self):
        self.keypoints = {}
        self.descriptors = {}

    def delete(self, image):
        del self.keypoints[image]
        del self.descriptors[image]

    def get_rotation_matrix(self, map_kp, map_desc, perc_kp, perc_desc):
        matches = self.compute_matches(map_desc, perc_desc)
        rot_matrix = self.compute_transformation(matches, map_kp, perc_kp)
        return rot_matrix

        
