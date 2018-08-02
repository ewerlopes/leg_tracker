
#include <ros/ros.h>

#include <tf/transform_broadcaster.h>

#include <geometry_msgs/PoseStamped.h>

#include <boost/bind.hpp>

#include <fstream>

void convertOptitrackDataToTF(const geometry_msgs::PoseStamped::ConstPtr &msg, std::string &optitrackFrame);
void saveOptitrackToFileAndPublish(const geometry_msgs::PoseStamped::ConstPtr &msg, std::string &optitrackFrame, std::string &file);

int main(int argc, char **argv) {

    ros::init(argc, argv, "optitrack_to_tf");
    ros::NodeHandle nh;

    std::string optitrackTopic, optitrackFrame;
    bool saveInFile;
    nh.param<std::string>("player_topic", optitrackTopic, "/robot_markerset/pose");
    nh.param<std::string>("player_pose", optitrackFrame, "/player_pose");
    nh.param<bool>("save", saveInFile, false);

    ros::Subscriber poseSub;

    if (saveInFile) {
        poseSub = nh.subscribe<geometry_msgs::PoseStamped>(optitrackTopic, 10, boost::bind(&convertOptitrackDataToTF, _1, optitrackFrame));
    } else {
        std::string file;
        nh.param<std::string>("log_file", file, "player_log.txt");
        std::ofstream out(file);
        out.close();    // clear file
        poseSub = nh.subscribe<geometry_msgs::PoseStamped>(optitrackTopic, 10, boost::bind(&saveOptitrackToFileAndPublish, _1, optitrackFrame, file));
    }

    ros::spin();

    return 0;
}

void convertOptitrackDataToTF(const geometry_msgs::PoseStamped::ConstPtr &msg, std::string &optitrackFrame)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
    tf::Quaternion q;
    q.setRPY(0, 0, 0);  // irrelevant the orientation, and not reliable!
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "map", optitrackFrame));
}

void saveOptitrackToFileAndPublish(const geometry_msgs::PoseStamped::ConstPtr &msg, std::string &optitrackFrame, std::string &file)
{
    convertOptitrackDataToTF(msg, optitrackFrame);
    std::ofstream out(file, std::ofstream::app);
    out << (msg->header.stamp).toSec() << "," << msg->pose.position.x << "," << msg->pose.position.y << "," << msg->pose.position.z << std::endl << std::flush;
    out.close();
}


