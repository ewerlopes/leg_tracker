#ifndef PLAYER_TRACKER_MOTION_VARIANCE_H_
#define PLAYER_TRACKER_MOTION_VARIANCE_H_

#include <player_tracker/motion_detector.h>
#include <player_tracker/Blob.h>
#include <player_tracker/TrackVariance.h>

typedef struct MinimalPointCloud {
    float variance;
    Eigen::Vector2d centroid;
} MinimalPointCloud;

typedef std::vector<MinimalPointCloud> MinimalPointCloudList;

#define OMP_THREADS 8

class MotionVariance : public MotionDetector {
public:
    MotionVariance(ros::NodeHandle nh, std::string scan_topic, float window_duration, std::string log_filename);
    MotionVariance(ros::NodeHandle nh, std::string scan_topic, float window_duration);
    ~MotionVariance();

protected:
    void initRosComunication();

    void convertClouds(Cloud2List &clusters, PointCloudList &clouds);

    virtual MinimalPointCloud minimizeCloud(sensor_msgs::PointCloud &cloud);
    virtual float computeDistance(MinimalPointCloud &cloud, const geometry_msgs::Pose &pose);

    void publishVariance(float variance, sensor_msgs::PointCloud cloud);
    void publishVariance(MinimalPointCloud &minimalCloud);

private:
    void blobDetectionCallback(const player_tracker::Blob &msg);

    ros::Subscriber blob_sub_;
    ros::Publisher variance_pub_;
};

#endif // PLAYER_TRACKER_MOTION_VARIANCE_H_
