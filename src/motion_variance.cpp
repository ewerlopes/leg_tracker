#include <player_tracker/motion_variance.h>

MotionVariance::MotionVariance(ros::NodeHandle nh, std::string scan_topic, float window_duration, std::string log_filename)
: MotionDetector(nh, scan_topic, window_duration, log_filename)
{

}

MotionVariance::MotionVariance(ros::NodeHandle nh, std::string scan_topic, float window_duration):
MotionDetector(nh, scan_topic,  window_duration)
{

}

void MotionVariance::initRosComunication()
{
    blob_sub_ = nodeHandle().subscribe("/blob_detection", 1, &MotionVariance::blobDetectionCallback, this);
    variance_pub_ = nodeHandle().advertise<player_tracker::TrackVariance>("/track_variance", 10);
}

void MotionVariance::blobDetectionCallback(const player_tracker::Blob &msg)
{
    if (!msg.observed) {
        return;
    }

    Cloud2List clusters = extractClusterInWindow();
    PointCloudList clouds(clusters.size());
    Eigen::VectorXd direction;

    #pragma omp parallel sections
    {
        #pragma omp section
        direction = computeSteepestDirection(clusters);
        #pragma omp section
        convertClouds(clusters, clouds);
    }

    MinimalPointCloudList minimalClouds(clouds.size());
    Eigen::VectorXd distances(clouds.size());

    #pragma omp parallel for
    for (int i = 0; i < clouds.size(); i++) {
        MinimalPointCloud pc = minimizeCloud(clouds[i]);
        minimalClouds[i] = pc;
        distances(i) = computeDistance(pc, msg.pose);
    }

    Eigen::VectorXd::Index closestCloud;
    distances.minCoeff(&closestCloud);

    publishVariance(minimalClouds[closestCloud]);

    /*
        -> compute eigenvector for projection
        iterate over all clouds
        get closest one to position of blob
        compute its variance
        publish it
     */
}


void MotionVariance::convertClouds(Cloud2List &clusters, PointCloudList &clouds)
{
    #pragma omp parallel for
    for (int i = 0; i < clusters.size(); i++) {
        sensor_msgs::convertPointCloud2ToPointCloud(clusters[i], clouds[i]);
    }
}

MinimalPointCloud MotionVariance::minimizeCloud(sensor_msgs::PointCloud &cloud)
{
    MinimalPointCloud minimizedCloud;
    Eigen::MatrixXd cloudMatrix = getCloudPointsAsMatrix(cloud);
    minimizedCloud.centroid = cloudMatrix.rowwise().mean();
    Eigen::MatrixXd covariance = getClusterVariance(cloudMatrix);
    minimizedCloud.variance = covariance(0);    // covariance of elements of 1 point is a single element!
    return minimizedCloud;
}

float MotionVariance::computeDistance(MinimalPointCloud &cloud, const geometry_msgs::Pose &pose)
{
    Eigen::Vector2d vectorPose;
    vectorPose << pose.position.x, pose.position.y;

    return (cloud.centroid - vectorPose).squaredNorm();
}


void MotionVariance::publishVariance(MinimalPointCloud &minimalCloud)
{
    player_tracker::TrackVariance msg;
    msg.variance = minimalCloud.variance;
    msg.centroid.x = minimalCloud.centroid.x();
    msg.centroid.y = minimalCloud.centroid.y();
    msg.centroid.z = 0.1;

    variance_pub_.publish(msg);
}

#include <omp.h>

int main(int argc, char **argv) {
    omp_set_num_threads(OMP_THREADS);
    ros::init(argc, argv, "motion_detection_algorithm");

    ros::NodeHandle nh;
    std::string scan_topic;
    nh.param("scan_topic", scan_topic, std::string("scan"));
    MotionVariance md(nh, scan_topic, 1, "/home/airlab/Scrivania/log_file.txt");

    //MotionDetector md(nh, scan_topic, 1);

    ros::spin();
    return 0;
}