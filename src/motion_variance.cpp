#include <player_tracker/motion_variance.h>

MotionVariance::MotionVariance(ros::NodeHandle nh, std::string scan_topic, float window_duration, std::string log_filename)
: MotionDetector(nh, scan_topic, window_duration, log_filename)
{
    initRosComunication();
}

MotionVariance::MotionVariance(ros::NodeHandle nh, std::string scan_topic, float window_duration):
MotionDetector(nh, scan_topic,  window_duration)
{
    initRosComunication();
 }

MotionVariance::~MotionVariance()
{ /*    */ }

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

    if (clusters.empty()) {
        return;
    }

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
        Eigen::MatrixXd cloudMatrix = getCloudPointsAsMatrix(clouds[i]);       //Contains to_merge_cloud as matrix.
        Eigen::MatrixXd projected = getProjection(cloudMatrix, direction);
        MinimalPointCloud pc = minimizeCloud(clouds[i], cloudMatrix, projected);
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

MinimalPointCloud MotionVariance::minimizeCloud(sensor_msgs::PointCloud &pc, Eigen::MatrixXd cloudMatrix, Eigen::MatrixXd &projection)
{
    MinimalPointCloud minimizedCloud;
    minimizedCloud.centroid = cloudMatrix.rowwise().mean(); // this is just a col
    minimizedCloud.points = pc;
    Eigen::MatrixXd covariance = getClusterVariance(projection);
    minimizedCloud.variance = covariance(0);    // covariance of elements of 1 point is a single element!
    return minimizedCloud;
}

float MotionVariance::computeDistance(MinimalPointCloud &cloud, const geometry_msgs::Pose &pose)
{
    Eigen::Vector2d vectorPose;
    vectorPose << pose.position.x, pose.position.y;

    return (cloud.centroid.topRows(2) - vectorPose).squaredNorm();
}


void MotionVariance::publishVariance(MinimalPointCloud &minimalCloud)
{
    player_tracker::TrackVariance msg;
    msg.header.stamp = ros::Time::now();
    msg.variance = minimalCloud.variance;
    msg.centroid.x = minimalCloud.centroid.x();
    msg.centroid.y = minimalCloud.centroid.y();
    msg.centroid.z = minimalCloud.centroid.z();
    msg.points = minimalCloud.points.points;

    variance_pub_.publish(msg);
}
