
#include <ros/ros.h>
#include <player_tracker/ClusterLogger.hpp>
#include <omp.h>

#define OMP_THREADS 8

int main(int argc, char **argv) {

    omp_set_num_threads(OMP_THREADS);

    ros::init(argc, argv, "cluster_logger");
    ros::NodeHandle nh;
    ros::NodeHandle privateNH("~");

    std::string scan_topic, filename;
    privateNH.param<std::string>("scan_topic", scan_topic, "scan");
    privateNH.param<std::string>("filename", filename, "log_file.txt");

    logging::ClusterLogger log(nh, scan_topic, filename);

    ros::spin();

    return 0;
}