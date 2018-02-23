
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
#include <std_msgs/Header.h>

// basic file operations
#include <iostream>
#include <fstream>

// Buffer
#include <boost/circular_buffer.hpp>

// Markers
#include <visualization_msgs/Marker.h>

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
        markers_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 20);

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
        markers_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 20);

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
    ros::Publisher markers_pub;

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

        if (mergeWindowClouds(from, to, cloud)){
            merged_cloud_pub.publish(cloud);
        }else{
            ROS_ERROR("Failed to publish merged cloud!");
        }
    }

    void publishMergedCloud(sensor_msgs::PointCloud cloud){
        merged_cloud_pub.publish(cloud);
    }

    void publishEigenMarker(std_msgs::Header header, Eigen::VectorXd eigenvect, int marker_id, geometry_msgs::Point32 mean){
        // Publish marker to rviz
        visualization_msgs::Marker m;
        m.header.stamp = header.stamp;
        m.header.frame_id = header.frame_id;
        m.ns = "Eigen";
        m.id = marker_id;
        m.type = m.ARROW;
        geometry_msgs::Point new_mean;
        new_mean.x = mean.x;
        new_mean.y = mean.y;
        new_mean.z = mean.z;
        m.points.push_back(new_mean);

        if (eigenvect(2) > 0)
            eigenvect = eigenvect * -1;

        geometry_msgs::Point eigen_point;
        eigen_point.x = eigenvect(0) + new_mean.x;
        eigen_point.y = eigenvect(1) + new_mean.y;
        eigen_point.z = eigenvect(2) + new_mean.z;
        m.points.push_back(eigen_point);
        //m.pose.position.x = eigenvect(0);
        //m.pose.position.y = eigenvect(1);
        //m.pose.position.z = eigenvect(2);
        m.scale.x = 0.05;
        m.scale.y = 0.05;
        m.scale.z = 0.05;
        m.color.a = 1;
        m.color.r = 1;
        m.color.g = 0;
        m.color.b = 0;
        m.lifetime = ros::Duration(0.1);

        #pragma omp critical
        markers_pub.publish(m);
    }

    int mergeWindowClouds(ros::Time from, ros::Time to, sensor_msgs::PointCloud &cloud){
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
            for (int i=0; i < cloud.points.size(); i++){
                log_file << cloud.points[i].x << "," << cloud.points[i].y << "," << cloud.points[i].z << std::endl;
            }
        }
    }

    void legClusterCallback(const player_tracker::LegArray::ConstPtr &leg_clusters_msg){
        #pragma omp parallel for
        for (int i=0; i < leg_clusters_msg->legs.size(); i++){
            sensor_msgs::PointCloud cloud;
            cloud.header = leg_clusters_msg->header;
            cloud.points = leg_clusters_msg->legs[i].points;

            try{
                tf_listener.waitForTransform("base_link", "odom", ros::Time(0.0), ros::Duration(0.1));
                tf_listener.transformPointCloud("odom", cloud, cloud);
                setTemporal(cloud, (ros::Time::now().toSec() - start_time.toSec()));
                #pragma omp critical
                leg_cloud_pub.publish(cloud);
            }catch (std::exception ex){
                ROS_ERROR("%s",ex.what());
            }
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

    Eigen::MatrixXd getProjection(Eigen::MatrixXd &dataset, Eigen::VectorXd &eigenvect){
        return eigenvect.transpose() * dataset;
    }

    void getCloudPointsAsMatrix(sensor_msgs::PointCloud2 &input, Eigen::MatrixXd &output){
        sensor_msgs::PointCloud cloud;
        if (sensor_msgs::convertPointCloud2ToPointCloud(input, cloud)){
            #pragma omp parallel for
            for (int i=0; i < cloud.points.size(); i++){
                output.col(i) << cloud.points[i].x, cloud.points[i].y, cloud.points[i].z;
            }
        }else{
            ROS_ERROR("ERROR WHEN GETTING PointCloud2 to PointCloud CONVERSION!");
        }
    }


    Eigen::MatrixXd getClusterVariance(Eigen::MatrixXd &input){
        Eigen::MatrixXd centered = input.colwise() - input.rowwise().mean();
        Eigen::MatrixXd cov = (centered * centered.transpose()) / double(input.cols() - 1);
        return cov;
    }


    Eigen::MatrixXd getClusterVariance(sensor_msgs::PointCloud2 &input){
        sensor_msgs::PointCloud cloud;
        if (sensor_msgs::convertPointCloud2ToPointCloud(input, cloud)){
            Eigen::MatrixXd point_matrix(3,cloud.points.size());
            #pragma omp parallel for
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

    std::vector<sensor_msgs::PointCloud2> getClusters(sensor_msgs::PointCloud2 &input){

        std::vector<sensor_msgs::PointCloud2> clusters;

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
            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {
                cloud_cluster->points.push_back(converted->points[*pit]); //*
            }
                
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            // Convert the pointcloud to be used in ROS
            sensor_msgs::PointCloud2::Ptr output(new sensor_msgs::PointCloud2);
            pcl::toROSMsg(*cloud_cluster, *output);
            output->header.frame_id = input.header.frame_id;

            // store cluster
            #pragma omp critical
            clusters.push_back(*output);
            count++;
        }

        ROS_WARN("Number of clusters: %d", count);
        return clusters;
    }

    Eigen::VectorXd getCLusterSmallestEigenValue(Eigen::MatrixXd cov){
        Eigen::VectorXd eigenvals = cov.eigenvalues().real();
        Eigen::EigenSolver<Eigen::MatrixXd> es(cov);
        Eigen::MatrixXd eigenvecs = es.eigenvectors().real();

        Eigen::MatrixXd::Index minIndex;
        double minVal = eigenvals.col(0).minCoeff(&minIndex);

        ROS_INFO_STREAM("EIGENVALUES:\n " << eigenvals);
        ROS_INFO_STREAM("minVal: " << minVal << "\tminIndex: " << minIndex);
        return eigenvecs.col(minIndex);
    }

    Eigen::MatrixXd::Index getSteepestVector(Eigen::MatrixXd eigenvectors){
        Eigen::MatrixXd A = eigenvectors.topRows(2);
        Eigen::VectorXd B = A.colwise().squaredNorm();
        Eigen::MatrixXd::Index minIndex;
        B.minCoeff(&minIndex);
        return minIndex;
    }

    Eigen::VectorXd getCLusterSmallestEigenValue(sensor_msgs::PointCloud2 &cluster){
        // get cluster variance
        Eigen::MatrixXd cov = getClusterVariance(cluster);
        Eigen::VectorXd eigenvals = cov.eigenvalues().real();
        Eigen::EigenSolver<Eigen::MatrixXd> es(cov);
        Eigen::MatrixXd eigenvecs = es.eigenvectors().real();


        Eigen::MatrixXf::Index minIndex;
        double minVal = eigenvals.col(0).minCoeff(&minIndex);


        ROS_INFO_STREAM("EIGENVALUES:\n " << eigenvals);
        ROS_INFO_STREAM("minVal: " << minVal << "\tminIndex: " << minIndex);
        return eigenvecs.col(minIndex);
    }


    void saveProjectionToFile(Eigen::MatrixXd projected){
        if (log_file.is_open()){
            #pragma omp critical
            for (int i=0; i < projected.cols(); i++){
                log_file << projected(i) << ",";
            }
        }

    }


    void processWindows(const sensor_msgs::LaserScan::ConstPtr &scan_msg){

        sensor_msgs::PointCloud to_merge_cloud;

        if (mergeWindowClouds(start_time, ros::Time::now(), to_merge_cloud)){

            if (!to_merge_cloud.points.size()){
                ROS_WARN("Merged point cloud is empty. Skipping...");
                return;
            }

            publishMergedCloud(to_merge_cloud);

            sensor_msgs::PointCloud2 as_pointcloud_2;
            
            if (sensor_msgs::convertPointCloudToPointCloud2(to_merge_cloud, as_pointcloud_2)){

                std::vector<sensor_msgs::PointCloud2> clusters = getClusters(as_pointcloud_2);
                
                if (clusters.empty()) {
                    return;
                }

                Eigen::MatrixXd eigenvects(3, clusters.size());      // Holds all eigenvectors;

                #pragma omp parallel for
                for(int i=0; i < clusters.size(); i++){
                    ROS_INFO("Processing cluster %d", i);
                    Eigen::VectorXd eigenvect = getCLusterSmallestEigenValue(clusters[i]);
                    eigenvects.col(i) << eigenvect(0), eigenvect(1), eigenvect(2);

                    #pragma omp critical
                    merged_cloud_cluster_pub.publish(clusters[i]);
                    geometry_msgs::Point32 mean = getClusterMean(clusters[i]);
                    publishEigenMarker(to_merge_cloud.header, eigenvect, i, mean);
                }


                Eigen::MatrixXd::Index steepest_eigenvect = getSteepestVector(eigenvects);
                Eigen::VectorXd a = eigenvects.col(steepest_eigenvect);

                // Save to file
                for(int i=0; i < clusters.size(); i++){
                    Eigen::MatrixXd asMatrix(3, clusters[i].width * clusters[i].height);       //Contains to_merge_cloud as matrix.
                    getCloudPointsAsMatrix(clusters[i], asMatrix);
                    Eigen::MatrixXd projected = getProjection(asMatrix, a);
                    ROS_INFO_STREAM(asMatrix.cols() << " <?> " << projected.cols());
                    if (projected.cols() != clusters[i].width * clusters[i].height){
                        ROS_ERROR("NOT EQUAL");
                    }
                    saveProjectionToFile(projected);
                }

            }else{
                ROS_ERROR("ERROR WHEN GETTING PointCloud to PointCloud2 CONVERSION!");
            }
        
        }else{
            ROS_ERROR("ERROR WHEN MERGING CLOUD!");
        }
    }

    /**
    * @brief callback for laser scan message
    */
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg){

        // check whether we can transform data
        try{
            if (!tf_listener.waitForTransform(scan_msg->header.frame_id,
                base_frame_, scan_msg->header.stamp + ros::Duration().fromSec(scan_msg->ranges.size() * 
                scan_msg->time_increment), ros::Duration(1.0))) {
                return;
            }
        }catch (std::exception ex){
            ROS_ERROR("%s",ex.what());
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
            ROS_WARN("End of windows... Processing");
            processWindows(scan_msg);
            saveToLog("\n--");
            on_window = false;
            return;
        }

        // set time dimension
        setTemporal(cloud, (scan_msg->header.stamp.toSec() - start_time.toSec()));

        // publish point cloud
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

    //MotionDetector md(nh, scan_topic, 1);

    ros::spin();
    return 0;
}
