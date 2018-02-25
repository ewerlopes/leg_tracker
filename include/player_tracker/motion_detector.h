#ifndef PLAYER_TRACKER_MOTION_DETECTOR_H_
#define PLAYER_TRACKER_MOTION_DETECTOR_H_

// Linear algebra
#include <eigen3/Eigen/Dense>

// Optmization
#include <omp.h>

// ROS core headers
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <laser_assembler/AssembleScans.h>
#include <sensor_msgs/point_cloud_conversion.h>

// PCL specific includes
#include <boost/lexical_cast.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

// Custom ROS messages
#include <player_tracker/LegArray.h>

// ROS messages
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <std_msgs/Header.h>

// basic file operations
#include <iostream>
#include <fstream>

// Buffer
#include <boost/circular_buffer.hpp>

// Markers
#include <visualization_msgs/Marker.h>

typedef boost::circular_buffer<geometry_msgs::Point32> PointCircularBuffer;
typedef std::vector<sensor_msgs::PointCloud2> Cloud2List;
typedef std::vector<sensor_msgs::PointCloud> PointCloudList;

#define OMP_THREADS 8

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
    * @param scan_topic the scan topic we would like to use
    * @param window_duration the slide windows duration in seconds
    * @param log_filename the name of the file to save log data
    */
    MotionDetector(ros::NodeHandle nh, std::string scan_topic, float window_duration, std::string log_filename);

    /**
    * @basic Constructor
    * @param nh A nodehandle
    * @param scan_topic the scan topic we would like to use
    * @param window_duration the slide windows duration in seconds
    */
    MotionDetector(ros::NodeHandle nh, std::string scan_topic, float window_duration);

    ~MotionDetector();

protected:

    tf::TransformListener tf_listener;
    
    void initRosComunication();

    ros::NodeHandle& nodeHandle();

    virtual Eigen::VectorXd computeSteepestDirection(Cloud2List &clusters);

    virtual Cloud2List extractClusterInWindow();

    /**
    * @brief gets the pointcloud from laserScan
    */
    virtual sensor_msgs::PointCloud getPointCloud(const sensor_msgs::LaserScan::ConstPtr &scan_msg);

    virtual void publishMergedCloud(ros::Time from, ros::Time to);

    virtual void publishMergedCloud(sensor_msgs::PointCloud cloud);

    virtual void publishEigenMarker(std_msgs::Header header, Eigen::VectorXd eigenvect, int marker_id, geometry_msgs::Point32 mean);

    virtual int mergeWindowClouds(ros::Time from, ros::Time to, sensor_msgs::PointCloud &cloud);

    virtual void setTemporal(sensor_msgs::PointCloud &cloud, double time);

    virtual void saveToLog(std::string text);

    virtual void saveCloudDataToLog(sensor_msgs::PointCloud &cloud);

    virtual geometry_msgs::Point32 getClusterMean(sensor_msgs::PointCloud2 &input);

    virtual Eigen::MatrixXd getProjection(Eigen::MatrixXd &dataset, Eigen::VectorXd &eigenvect);

    virtual Eigen::MatrixXd getCloudPointsAsMatrix(sensor_msgs::PointCloud2 &input);
    virtual Eigen::MatrixXd getCloudPointsAsMatrix(sensor_msgs::PointCloud &cloud);

    virtual Eigen::MatrixXd getClusterVariance(Eigen::MatrixXd &input);

    virtual Eigen::MatrixXd getClusterVariance(sensor_msgs::PointCloud2 &input);

    virtual Cloud2List getClusters(sensor_msgs::PointCloud2 &input);

    virtual Eigen::VectorXd getClusterSmallestEigenValue(Eigen::MatrixXd cov);
    virtual Eigen::VectorXd getClusterSmallestEigenValue(sensor_msgs::PointCloud2 &cluster);

    virtual Eigen::MatrixXd::Index getSteepestVector(Eigen::MatrixXd eigenvectors);

    virtual void saveProjectionToFile(Eigen::MatrixXd projected, int id);

    virtual void processWindows(const sensor_msgs::LaserScan::ConstPtr &scan_msg);

    virtual PointCloudList& getWindowCloudList();
    virtual void clearWindowCloudList();
    virtual void appendWindowCloudList(sensor_msgs::PointCloud &cloud);

    // What is the meaning of this function??
    virtual void run();

private:
    std::string scan_topic_;
    std::string fixed_frame_;
    std::string base_frame_;

    bool on_window;

    std::ofstream log_file;

    ros::Duration window_duration_;// delta_t
    ros::Time start_time;          // t_s

    ros::NodeHandle nh_;

    ros::Subscriber scan_sub_;
    ros::Subscriber leg_cluster_sub;

    ros::Publisher cloud_pub;                   // publishes laserScan as PointCloud
    ros::Publisher merged_cloud_pub;            // publish merged (across time) cloud
    ros::Publisher merged_cloud_cluster_pub;    // publish cluster clouds inside the merged one.
    ros::Publisher leg_cloud_pub;
    ros::Publisher markers_pub;

    ros::ServiceClient client;
    laser_assembler::AssembleScans srv;
    laser_geometry::LaserProjection projector_;

    PointCircularBuffer window; // 10 is the laser frequency

    PointCloudList cloudsInWindow;


    void legClusterCallback(const player_tracker::LegArray::ConstPtr &leg_clusters_msg);

    /*
    * @brief callback for laser scan message
    */
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);

};



#endif // PLAYER_TRACKER_MOTION_DETECTOR_H_
