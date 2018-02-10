
#include <ros/ros.h>

#include <particle_filter/BlobPublisher.h>

int main(int argc, char** argv){
    
    ros::init(argc, argv, "player_blob_transformer");

    ros::NodeHandle nh;
    BlobPublisher blobPub(nh, "/player_blob_transformer");
    ros::spin();

    return 0;
}
