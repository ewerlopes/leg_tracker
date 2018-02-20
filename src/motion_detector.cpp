#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <laser_assembler/AssembleScans.h>

// ROS messages
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

// basic file operations
#include <iostream>
#include <fstream>

/**
* @basic The motion detector algorithm in Shen, Xiaotong, Seong-Woo Kim, and
* Marcelo H. Ang.
* "Spatio-temporal motion features for laser-based moving objects detection and
* tracking. In
* IEEE/RSJ Internation Conference on Intelligent Robots and Systems (IROS 2014).
* September 14-18,
* Chicago, IL, USA.
*/
class MotionDetector {
public:
    /**
    * @basic Constructor
    * @param nh A nodehandle
    * @param scan_topic The topic for the scan we would like to map
    */
    MotionDetector(ros::NodeHandle nh, std::string scan_topic, float window_duration, std::string log_filename)
        : nh_(nh), scan_topic_(scan_topic), window_duration_(window_duration), on_window(false) {
        ros::NodeHandle nh_private("~");
        base_frame_ = "odom";
        scan_sub_ = nh_.subscribe("/scan", 1, &MotionDetector::laserCallback, this);
        merged_cloud_pub = nh_.advertise<sensor_msgs::PointCloud>("merged_cloud", 10);
        cloud_pub = nh_.advertise<sensor_msgs::PointCloud>("laser_cloud", 10);

        ros::service::waitForService("assemble_scans");
        client = nh_.serviceClient<laser_assembler::AssembleScans>("assemble_scans");

        log_file.open (log_filename);
    }

    ~MotionDetector(){
        log_file.close();
    }

private:
    std::string scan_topic_;
    std::string fixed_frame_;
    std::string base_frame_;

    bool on_window;

    std::ofstream log_file;

    ros::Duration window_duration_;// delta_t
    ros::Time start_time;          // t_s

    // static_pts_set;             // S_t
    // movint_pts_set;             // M_t
    // partition_prob_measure;     // P_t
    // PoseStamped robot_pose;     // q_t
    // pts_in_odom;

    laser_geometry::LaserProjection projector_;

    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher cloud_pub;
    ros::Publisher merged_cloud_pub;
    tf::TransformListener tf_listener;

    laser_assembler::AssembleScans srv;
    ros::ServiceClient client;

    /**
    * @brief gets the pointcloud from laserScan
    */
    void getPointCloud(const sensor_msgs::LaserScan::ConstPtr &scan_msg, sensor_msgs::PointCloud &cloud){
        projector_.transformLaserScanToPointCloud(base_frame_,*scan_msg, cloud,tf_listener);
    }

    void publishMergedCloud(ros::Time from, ros::Time to){
        sensor_msgs::PointCloud cloud;

        if (mergeCloud(from, to, cloud)){
            merged_cloud_pub.publish(cloud);
        }else{
            ROS_ERROR("Failed to publish merged cloud!");
        }
    }

    int mergeCloud(ros::Time from, ros::Time to, sensor_msgs::PointCloud &cloud){
        srv.request.begin = from;
        srv.request.end   = to;
        
        ROS_WARN("Time diff: %f", (srv.request.end.toSec() - start_time.toSec()));

        if (client.call(srv)){
            ROS_INFO("Got cloud with %lu points\n", srv.response.cloud.points.size());
            cloud = srv.response.cloud;
            return 1;
        }else{
            ROS_ERROR("Service call to assemble_scans failed\n");
            return 0;
        }
    }

    void setTemporal(sensor_msgs::PointCloud &cloud, double time){
        #pragma omp parallel for    
        for (int i=0; i < cloud.points.size(); i++){
            cloud.points[i].z = time;
        }
    }

    void saveToLog(sensor_msgs::PointCloud &cloud, double time){
        #pragma omp parallel for    
        for (int i=0; i < cloud.points.size(); i++){
            log_file << cloud.points[i].x << "," << cloud.points[i].y << "," << cloud.points[i].z << std::endl;
        }
    }

    /**
    * @brief callback for laser scan message
    */
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {

        // check whether we can transform data
        if (!tf_listener.waitForTransform(scan_msg->header.frame_id,
            base_frame_, scan_msg->header.stamp + ros::Duration().fromSec(scan_msg->ranges.size() * 
            scan_msg->time_increment), ros::Duration(1.0))) {
            return;
        }

        sensor_msgs::PointCloud cloud;

        if (!on_window){
            ROS_WARN("Starting windows...");
            on_window = true;
            start_time = scan_msg->header.stamp;
            getPointCloud(scan_msg,cloud);

        }else if ((scan_msg->header.stamp.toSec() - start_time.toSec()) < window_duration_.toSec()){
            ROS_INFO("Inside windows...");
            ROS_INFO("Time: %f", (scan_msg->header.stamp.toSec() - start_time.toSec()));
            getPointCloud(scan_msg,cloud);
        }else{
            ROS_WARN("Resetting...");
            log_file << "--" << std::endl;
            on_window = false;
            return;
        }

        // set time dimension
        setTemporal(cloud, (scan_msg->header.stamp.toSec() - start_time.toSec()));
        
        //publishMergedCloud(start_time, ros::Time::now());
        publishMergedCloud(ros::Time(0.0), ros::Time::now());
        
        // save to file
        saveToLog(cloud, (scan_msg->header.stamp.toSec() - start_time.toSec()));

        // publich point cloud
        cloud_pub.publish(cloud);

    }

    void run() {}
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "motion_detection_algorithm");

    ros::NodeHandle nh;
    std::string scan_topic;
    nh.param("scan_topic", scan_topic, std::string("scan"));
    MotionDetector md(nh, scan_topic, 1, "/home/airlab/Scrivania/log_file.txt");

    ros::spin();
    return 0;
}
