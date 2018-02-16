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
        return cv2.warpAffine(image, M, (4000,4000))

    def extract_points(self, matches, kp1, kp2):
        pts1 = None
        pts2 = None

        for match in matches:
            pts1 = self.append_point_to(kp1[match.trainIdx], pts1)
            pts2 = self.append_point_to(kp2[match.queryIdx], pts2)
        return pts1[3:6, :], pts2[3:6, :]

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

if __name__ == "__main__":

    perception_grid_projector = PerceptionGridProjector()

    # Load an color image in grayscale 
    ### CHANGE PATH FOR FILES ### 
    map_img = cv2.imread('/home/airlab/catkin_ws/src/player_tracker/imagesimage.png',0)

    cv2.namedWindow('mapimage', cv2.WINDOW_NORMAL)
    cv2.imshow('mapimage',map_img)
    cv2.waitKey(0)

    # map descriptor
    map_kp, map_desc = perception_grid_projector.compute_descriptors(map_img)

    # image descriptor
    img = cv2.imread('/home/airlab/catkin_ws/src/player_tracker/images/image_grid.png',0)

    cv2.namedWindow('image_grid', cv2.WINDOW_NORMAL)
    cv2.imshow('image_grid', img)
    cv2.waitKey(0)

    # map descriptor
    perc_kp, perc_desc = perception_grid_projector.compute_descriptors(img)

    rotation_mtx = perception_grid_projector.get_rotation_matrix(map_kp, map_desc, perc_kp, perc_desc)
    trans_ = perception_grid_projector.project_image(rotation_mtx, img)

    cv2.namedWindow('trans_', cv2.WINDOW_NORMAL)
    cv2.imshow('trans_', trans_)
    cv2.waitKey(0)

    matches = perception_grid_projector.compute_matches(map_desc, perc_desc)

    #draw_params = dict(matchColor=(0,255,0), singlePointColor=(255,0,0), flags=0)
    #img3 = cv2.drawMatchesKnn(map_img, map_kp, img, perc_kp, matches, None, flags=2)
    print len(matches)
    print len(map_kp)
    print len(perc_kp)
    img3 = cv2.drawMatches(img, perc_kp, map_img, map_kp, matches[:10], None, flags=2)

    cv2.namedWindow('img3', cv2.WINDOW_NORMAL)
    cv2.imshow('img3',img3)
    cv2.waitKey(0)

    print np.array_str(trans_, precision=2, suppress_small=True)
    print np.where(trans_ > 0)

    print "image: {}".format(np.where(img > 0))
    print "map: {}".format(np.where(map_img > 0))
    print "rotation: \n{}".format(np.array_str(rotation_mtx, precision=2, suppress_small=True))

    #cv2.imwrite('/home/airlab/Scrivania/trans_.png',trans_)
