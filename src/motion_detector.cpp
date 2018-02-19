#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <eigen3/Eigen/Dense>

// ROS messages
#include <geometry_msgs/PoseStamped.h>
#include <laser_geometry/laser_geometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

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
    MotionDetector(ros::NodeHandle nh, std::string scan_topic, float window_duration)
        : nh_(nh), scan_topic_(scan_topic), window_duration_(window_duration) {
        ros::NodeHandle nh_private("~");
        base_frame_ = "odom";
        scan_sub_ = nh_.subscribe("/scan", 1, &MotionDetector::laserCallback, this);
        cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("laser_cloud", 10);
    }

    ~MotionDetector(){
        clearPoints();
    }



private:
    std::string scan_topic_;
    std::string fixed_frame_;
    std::string base_frame_;

    ros::Duration window_duration_;// delta_t
    long double start_time;      // t_s

    std::vector< sensor_msgs::PointCloud2*> points;

    // static_pts_set;             // S_t
    // movint_pts_set;             // M_t
    // partition_prob_measure;     // P_t
    // PoseStamped robot_pose;     // q_t
    // pts_in_odom;

    laser_geometry::LaserProjection projector_;

    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher cloud_pub;
    tf::TransformListener tf_listener;

    void clearPoints(){
        #pragma omp parallel for
        for (int i = 0; i < points.size(); i++) {
            delete points[i];
        }
    }

    void getPointCloud(const sensor_msgs::LaserScan::ConstPtr &scan_msg, sensor_msgs::PointCloud2 &cloud){
        projector_.transformLaserScanToPointCloud(base_frame_,*scan_msg, cloud,tf_listener);
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

        sensor_msgs::PointCloud2* cloud = new sensor_msgs::PointCloud2();

        if (points.empty()){
            ROS_WARN("Starting windows...");
            start_time = scan_msg->header.stamp.toSec();
            getPointCloud(scan_msg,*cloud);
            points.push_back(cloud);
        }else if ((scan_msg->header.stamp.toSec() - start_time) < window_duration_.toSec()){
            ROS_INFO("Inside windows...");
            ROS_INFO("List size: %lu", points.size());
            ROS_INFO("Time: %Lf", (scan_msg->header.stamp.toSec() - start_time));
            getPointCloud(scan_msg,*cloud);
            points.push_back(cloud);
        }else{
            ROS_WARN("Resetting...");
            ROS_INFO("Empty: %d", points.empty());
            clearPoints();
            ROS_INFO("Empty: %d", points.empty());
            return;
        }

        // publich point cloud
        cloud_pub.publish(*cloud);

    }

    void run() {}
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "motion_detection_algorithm");

    ros::NodeHandle nh;
    std::string scan_topic;
    nh.param("scan_topic", scan_topic, std::string("scan"));
    MotionDetector md(nh, scan_topic, 3.0);

    ros::spin();
    return 0;
}
