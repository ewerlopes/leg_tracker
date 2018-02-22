
// Linear algebra
#include <eigen3/Eigen/Dense>

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

// basic file operations
#include <iostream>
#include <fstream>

// Buffer
#include <boost/circular_buffer.hpp>

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
    MotionDetector(ros::NodeHandle nh, std::string scan_topic, float window_duration, std::string log_filename)
        : nh_(nh), scan_topic_(scan_topic), window_duration_(window_duration), on_window(false),
        window(window_duration*10){
        ros::NodeHandle nh_private("~");
        
        base_frame_ = "odom";
        
        scan_sub_ = nh_.subscribe("/scan", 1, &MotionDetector::laserCallback, this);
        leg_cluster_sub = nh_.subscribe("detected_leg_clusters", 1, &MotionDetector::legClusterCallback, this);
        
        merged_cloud_pub = nh_.advertise<sensor_msgs::PointCloud>("merged_cloud", 10);
        cloud_pub = nh_.advertise<sensor_msgs::PointCloud>("laser_cloud", 10);
        merged_cloud_cluster_pub = nh_.advertise<sensor_msgs::PointCloud2>("merged_clusters", 10);
        leg_cloud_pub = nh_.advertise<sensor_msgs::PointCloud>("leg_cloud", 10);

        ros::service::waitForService("assemble_scans");
        client = nh_.serviceClient<laser_assembler::AssembleScans>("assemble_scans");

        log_file.open(log_filename);
    }

    /**
    * @basic Constructor
    * @param nh A nodehandle
    * @param scan_topic the scan topic we would like to use
    * @param window_duration the slide windows duration in seconds
    */
    MotionDetector(ros::NodeHandle nh, std::string scan_topic, float window_duration)
        : nh_(nh), scan_topic_(scan_topic), window_duration_(window_duration), on_window(false),
        window(window_duration*10){
        ros::NodeHandle nh_private("~");
        
        base_frame_ = "odom";
        
        scan_sub_ = nh_.subscribe("/scan", 1, &MotionDetector::laserCallback, this);
        leg_cluster_sub = nh_.subscribe("detected_leg_clusters", 1, &MotionDetector::legClusterCallback, this);

        merged_cloud_pub = nh_.advertise<sensor_msgs::PointCloud>("merged_cloud", 10);
        cloud_pub = nh_.advertise<sensor_msgs::PointCloud>("laser_cloud", 10);
        merged_cloud_cluster_pub = nh_.advertise<sensor_msgs::PointCloud2>("merged_clusters", 10);
        leg_cloud_pub = nh_.advertise<sensor_msgs::PointCloud>("leg_cloud", 10);

        ros::service::waitForService("assemble_scans");
        client = nh_.serviceClient<laser_assembler::AssembleScans>("assemble_scans");
    }

    ~MotionDetector(){
        if (log_file.is_open()){
            log_file.close();
        }
    }

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

    ros::ServiceClient client;
    laser_assembler::AssembleScans srv;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener tf_listener;

    boost::circular_buffer<geometry_msgs::Point32> window; //10 is the laser frequency

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

    void publishMergedCloud(sensor_msgs::PointCloud cloud){
        merged_cloud_pub.publish(cloud);
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

    void saveToLog(std::string text){
         if (log_file.is_open()){
            log_file << text.c_str() << std::endl;
        }
    }

    void saveCloudDataToLog(sensor_msgs::PointCloud &cloud){
        if (log_file.is_open()){
            #pragma omp parallel for    
            for (int i=0; i < cloud.points.size(); i++){
                log_file << cloud.points[i].x << "," << cloud.points[i].y << "," << cloud.points[i].z << std::endl;
            }
        }
    }

    void legClusterCallback(const player_tracker::LegArray::ConstPtr &leg_clusters_msg){
        for (int i=0; i < leg_clusters_msg->legs.size(); i++){
            sensor_msgs::PointCloud cloud;
            cloud.header = leg_clusters_msg->header;
            cloud.points = leg_clusters_msg->legs[i].points;

            try{
                tf_listener.waitForTransform("base_link", "odom", ros::Time(0.0), ros::Duration(0.1));
            }catch (std::exception ex){
                ROS_ERROR("%s",ex.what());
                return;
            }
            tf_listener.transformPointCloud("odom", cloud, cloud);
            setTemporal(cloud, (ros::Time::now().toSec() - start_time.toSec()));
            leg_cloud_pub.publish(cloud);

        }
    }

    geometry_msgs::Point32 getClusterMean(sensor_msgs::PointCloud2 &input){
        sensor_msgs::PointCloud cloud;
        if (sensor_msgs::convertPointCloud2ToPointCloud(input, cloud)){
            geometry_msgs::Point32 sum;
            float factor = (float) 1/cloud.points.size();
            for (int i=0; i < cloud.points.size(); i++){
                sum.x += cloud.points[i].x * factor;
                sum.y += cloud.points[i].y * factor;
                sum.z += cloud.points[i].z * factor;
            }
            return sum;
        }else{
            ROS_ERROR("ERROR WHEN GETTING PointCloud2 to PointCloud CONVERSION!");
        }
    }

    Eigen::MatrixXd getClusterVariance(sensor_msgs::PointCloud2 &input){
        sensor_msgs::PointCloud cloud;
        if (sensor_msgs::convertPointCloud2ToPointCloud(input, cloud)){
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> point_matrix(3,cloud.points.size());
            for (int i=0; i < cloud.points.size(); i++){
                point_matrix.col(i) << cloud.points[i].x, cloud.points[i].y, cloud.points[i].z;
            }

            Eigen::MatrixXd centered = point_matrix.colwise() - point_matrix.rowwise().mean();
            Eigen::MatrixXd cov = (centered * centered.transpose()) / double(point_matrix.cols() - 1);

            return cov;
        }else{
            ROS_ERROR("ERROR WHEN GETTING PointCloud2 to PointCloud CONVERSION!");
        }

    }

    void publishMergeClusters(sensor_msgs::PointCloud2 &input){
        pcl::PCLPointCloud2 to_convert;
        
        // Change from type sensor_msgs::PointCloud2 to pcl::PointXYZ
        pcl_conversions::toPCL(input, to_convert);

        pcl::PointCloud<pcl::PointXYZ>::Ptr converted(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(to_convert,*converted);

        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(converted);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.50); // 50cm
        ec.setMinClusterSize(20);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(converted);
        ec.extract(cluster_indices);

        int count(0);

        #pragma omp parallel for 
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (std::vector<int>::const_iterator pit = it->indices.begin();
            pit != it->indices.end(); pit++)
            cloud_cluster->points.push_back(converted->points[*pit]); //*
            
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            // Convert the pointcloud to be used in ROS
            sensor_msgs::PointCloud2::Ptr output(new sensor_msgs::PointCloud2);
            pcl::toROSMsg(*cloud_cluster, *output);
            output->header.frame_id = input.header.frame_id;

            // Publish the data
            merged_cloud_cluster_pub.publish(output);

            // get cluster mean
            //geometry_msgs::Point32 mean = getClusterMean(*output);
            //ROS_INFO("Mean of cluster %d: (%f,%f,%f)",count,mean.x,mean.y,mean.z);

            // get cluster variance
            Eigen::MatrixXd cov = getClusterVariance(*output);
            ROS_INFO("COV of cluster %d", count);
            Eigen::VectorXcd eigenvals = cov.eigenvalues();
            Eigen::EigenSolver<Eigen::MatrixXd> es(cov);
            Eigen::MatrixXd eigenvecs = es.eigenvectors().real();

            ROS_INFO_STREAM(eigenvecs);

            count++;
        }

        ROS_WARN("Number of clusters: %d", count);
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
            saveToLog("--");
            on_window = false;
            return;
        }

        // set time dimension
        setTemporal(cloud, (scan_msg->header.stamp.toSec() - start_time.toSec()));
        
        // save to file
        saveCloudDataToLog(cloud);

        // publish point cloud
        cloud_pub.publish(cloud);

        //publishMergedCloud(start_time, ros::Time::now());
        //publishMergedCloud(ros::Time(0.0), ros::Time::now());
        sensor_msgs::PointCloud to_merge_cloud;
        if (mergeCloud(start_time, ros::Time::now(), to_merge_cloud)){
            publishMergedCloud(to_merge_cloud);
            sensor_msgs::PointCloud2 as_pointcloud_2;
            if (sensor_msgs::convertPointCloudToPointCloud2(to_merge_cloud, as_pointcloud_2)){
                publishMergeClusters(as_pointcloud_2);
            }else{
                ROS_ERROR("ERROR WHEN GETTING PointCloud to PointCloud2 CONVERSION!");
            }
        }else{
            ROS_ERROR("ERROR WHEN MERGING CLOUD!");
        }

    }

    void run() {}
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "motion_detection_algorithm");

    ros::NodeHandle nh;
    std::string scan_topic;
    nh.param("scan_topic", scan_topic, std::string("scan"));
    //MotionDetector md(nh, scan_topic, 1, "/home/airlab/Scrivania/log_file.txt");

    MotionDetector md(nh, scan_topic, 1);

    ros::spin();
    return 0;
}
