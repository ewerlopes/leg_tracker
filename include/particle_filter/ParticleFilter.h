#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H

#include <random>

#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <player_tracker/PersonArray.h>

#include "particle_filter/Particle.h"

#define WHITE_NOISE_MEAN 0.0f
#define WHITE_NOISE_STD 0.1f

typedef struct{
    float mu;
    float variance;
}user_position;

class ParticleFilter {
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber personDetectedSub;
    ros::Subscriber blobSub;
    int num_particles;                                      // number of particles
    ParticleList particles;
    float tao;                                              // temperature parameter used to compute the weights of the particles with kinect observations

    void resample();
    void propagateParticles();
    void updateParticles(Eigen::Matrix<float, 1, 3> &Z);                                  // uses Kalman filter update equation
    void deleteOldParticles();
    void publishParticles();
    void initParticles();
    void initParticles(const geometry_msgs::Pose &pose);
    void computeWeights(Eigen::Matrix<float, 1, 3> &obs);
    void computeKinectWeights(Eigen::Matrix<float, 1, 3> &obs);

public:
    
    ParticleFilter(int num_particles=1000);
    ~ParticleFilter();

    void peopleDetectedCallback(const player_tracker::PersonArray &people);
    void blobDetectedCallback(const geometry_msgs::Pose &pose);
    void track(const geometry_msgs::Pose &pose);
};


#endif //PARTICLEFILTER_H
