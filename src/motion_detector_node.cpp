/*******************************************************
    Motion Detector main ros
********************************************************/

#include <omp.h>
#include <ros/ros.h>
#include <player_tracker/motion_detector.h>

int main(int argc, char **argv) {
    omp_set_num_threads(OMP_THREADS);

    ros::init(argc, argv, "motion_detection_algorithm");

    ros::NodeHandle nh;
    std::string scan_topic;
    nh.param("scan_topic", scan_topic, std::string("scan"));
    MotionDetector md(nh, scan_topic, 1, "/home/airlab/Scrivania/log_file.txt");

    //MotionDetector md(nh, scan_topic, 1);

    ros::spin();
    return 0;
}
