#ifndef PLAYER_TRACKER_CLUSTER_LOGGER_H_
#define PLAYER_TRACKER_CLUSTER_LOGGER_H_

#include <fstream>
#include <deque>
#include <vector>
#include <cmath>

#include <Eigen/Dense>

#include <ros/ros.h>

#include <tf/transform_listener.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <player_tracker/LegArray.h>
#include <laser_assembler/AssembleScans.h>
#include <laser_geometry/laser_geometry.h>

// pcl
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


namespace logging {

    typedef struct WindowReference {
        unsigned int startSequence;
        unsigned int currentNum = 0;
        ros::Time startTime;
        ros::Time endTime;
        bool valid = false;
    } WindowReference;
    typedef std::deque<WindowReference> WindowsQueue;
    typedef std::vector<sensor_msgs::PointCloud2> PointCloud2List;
    typedef std::vector<sensor_msgs::PointCloud> PointCloudList;
    typedef std::deque<PointCloudList> CloudQueue;
    typedef std::pair<int, float> IntFloatPair;
    typedef std::vector<IntFloatPair> IntFloatList;
    typedef std::vector<float> FloatList;

    /**
    *  @brief a comparison fuction used for the set std template class.
    */
    struct{
        bool operator()(const geometry_msgs::Point32 &a, const geometry_msgs::Point32 &b){
            return ( a.x < b.x && a.y < b.y && a.z < b.z );
        }
    }compPoints;

    struct{
    bool operator() (std::pair <int,float> i , std::pair <int,float> j) {
        return i.second < j.second; }
    } compPair;



    class ClusterLogger {
    public:
        ClusterLogger(ros::NodeHandle &nh, std::string scanTopic, std::string logFilename);
        ~ClusterLogger();

    protected:
        virtual void initROSCommunication();
        virtual void openLogFile();
        virtual void saveToLogFile(std::string text);
        virtual void closeWindow();

        void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);

        virtual void fillWindowReference(WindowReference &window, const sensor_msgs::LaserScan::ConstPtr &msg);

        virtual void publishWindow(sensor_msgs::LaserScan::ConstPtr &msg);

        virtual sensor_msgs::PointCloud transformLaserToCloud(const sensor_msgs::LaserScan::ConstPtr &scanMsg);
        virtual void setTimeToCloud(sensor_msgs::PointCloud &cloud, unsigned int laserSeq);
        virtual void computeSpatialTemporalFeatures(unsigned int time);
        virtual float extractClusterAngle(sensor_msgs::PointCloud2 &cluster, Eigen::MatrixXd::ColXpr eigenCol);
        virtual float computeAngle(Eigen::VectorXd &eigenvect);
        virtual PointCloud2List getClustersInWindow();
        virtual IntFloatList computeSimilarity(sensor_msgs::PointCloud2 &cluster, PointCloud2List &clusters);
        virtual float computeJaccardSimilarity(sensor_msgs::PointCloud &cloudIn1, sensor_msgs::PointCloud &cloudIn2);

        virtual Eigen::VectorXd computeSmallestEigenvector(Eigen::MatrixXd &cov);
        virtual Eigen::VectorXd computeSmallestEigenvector(sensor_msgs::PointCloud2 &cluster);

        virtual int assembledCloud(ros::Time from, ros::Time to, sensor_msgs::PointCloud &cloud);
        virtual PointCloud2List getClusters(sensor_msgs::PointCloud2 &input);
        virtual void computeClusterIndices(std::vector<pcl::PointIndices> &cluster_indices, pcl::search::KdTree<pcl::PointXYZ>::Ptr &tree, pcl::PointCloud<pcl::PointXYZ>::Ptr &converted);
        virtual Eigen::MatrixXd computeClusterVariance(Eigen::MatrixXd &input);
        virtual Eigen::MatrixXd computeClusterVariance(sensor_msgs::PointCloud2 &input);
        virtual Eigen::MatrixXd cloudPointAsMatrix(sensor_msgs::PointCloud2 &input);
        virtual Eigen::MatrixXd cloudPointAsMatrix(sensor_msgs::PointCloud &cloud);

    private:
        ros::NodeHandle nh;
        std::string scanTopic;
        std::string logFilename;

        ros::Subscriber laserSubscriber;
        ros::Publisher laserCloudPublisher;
        ros::Publisher cloudClusterPublisher;
        ros::ServiceClient assembleScanService;              // assemble scan service client.

        std::ofstream logFile;

        WindowsQueue windows;
        unsigned int windowSize;

        laser_geometry::LaserProjection projector;                 // object to transform LaserScan data.
        tf::TransformListener tfListener;

        sensor_msgs::LaserScan::ConstPtr prevMsg;

    };


} // namespace logging

#endif // PLAYER_TRACKER_CLUSTER_LOGGER_H_