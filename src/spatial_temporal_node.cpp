#include <omp.h>
#include <ros/ros.h>
#include <player_tracker/spatial_temporal.h>

#define OMP_THREADS 8

int main(int argc, char **argv) {
    omp_set_num_threads(OMP_THREADS);
    ros::init(argc, argv, "motion_detection_algorithm");

    ros::NodeHandle nh;
    std::string scan_topic, filename;
    nh.param<std::string>("scan_topic", scan_topic, "scan");
    nh.param<std::string>("filename", filename, "log_file.txt");
    spatial_temporal::Extractor ext(nh, scan_topic, 1, filename);

    //spatial_temporal::Extractor ext(nh, scan_topic, 1);

    ros::spin();
    return 0;
}