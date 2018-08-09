#include <omp.h>
#include <ros/ros.h>
#include <player_tracker/spatial_temporal.h>

#define OMP_THREADS 8

int main(int argc, char **argv) {
    omp_set_num_threads(OMP_THREADS);
    ros::init(argc, argv, "motion_detection_algorithm");

    ros::NodeHandle nh, privateNH("~");
    std::string scan_topic, filename;
    privateNH.param<std::string>("scan_topic", scan_topic, "scan");
    privateNH.param<std::string>("filename", filename, "log_file");
    spatial_temporal::Extractor ext(nh, scan_topic, 1, filename);

    //spatial_temporal::Extractor ext(nh, scan_topic, 1);

    ros::spin();
    return 0;
}