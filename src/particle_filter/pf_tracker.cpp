#include "particle_filter/ParticleFilter.h"
#include <ros/ros.h>

int main(int argc, char** argv){
    
    ros::init(argc, argv, "particle_filter");
    ParticleFilter pf;
    ros::spin();
    return 0;
}