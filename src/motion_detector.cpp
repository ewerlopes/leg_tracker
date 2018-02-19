#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <eigen3/Eigen/Dense>

// ROS messages
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>


/**
* @basic The motion detector algorithm in Shen, Xiaotong, Seong-Woo Kim, and Marcelo H. Ang. 
* "Spatio-temporal motion features for laser-based moving objects detection and tracking. In 
* IEEE/RSJ Internation Conference on Intelligent Robots and Systems (IROS 2014). September 14-18,
* Chicago, IL, USA.
*/
class MotionDetector {
public:
    /**
    * @basic Constructor
    * @param nh A nodehandle
    * @param scan_topic The topic for the scan we would like to map
    */
    MotionDetector(ros::NodeHandle nh, std::string scan_topic): nh_(nh), scan_topic_(scan_topic){
        ros::NodeHandle nh_private("~");
        scan_sub_ = nh_.subscribe("/scan", 1, &MotionDetector::laserCallback, this);
        cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("laser_cloud", 10);
    }

private:
	std::string scan_topic_;
	std::string fixed_frame_;
	std::string base_frame_;

    //static_pts_set;             // S_t
    //movint_pts_set;             // M_t
    //partition_prob_measure;     // P_t
    //PoseStamped robot_pose;     // q_t
    //pts_in_odom;                

    laser_geometry::LaserProjection projector_;


	ros::NodeHandle nh_;
	ros::Subscriber scan_sub_;
    ros::Publisher cloud_pub;
	tf::TransformListener tf_listener;

	/**
	* @brief callback for laser scan message
	*/
	void laserCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg){
        sensor_msgs::PointCloud2 cloud;
        projector_.projectLaser(*scan_msg, cloud);
        cloud_pub.publish(cloud);
    }


    void run(){

    }

};

int main(int argc, char **argv) {
	ros::init(argc, argv, "motion_detection_algorithm");

	ros::NodeHandle nh;
	std::string scan_topic;
	nh.param("scan_topic", scan_topic, std::string("scan"));
	MotionDetector md(nh, scan_topic);

	ros::spin();
	return 0;
}
