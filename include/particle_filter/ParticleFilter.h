#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <eigen3/Eigen/Dense>
#include <player_tracker/PersonArray.h>
#include "particle_filter/Particle.h"

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
    std::vector<Particle*> particles;
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
    
    ParticleFilter(int num_particles=2000){
        this->num_particles = num_particles;
        particles.resize(num_particles);
        pub = nh.advertise<geometry_msgs::PoseArray>("/pf_cloud", 10);
        personDetectedSub = nh.subscribe("/people_tracked", 10, &ParticleFilter::peopleDetectedCallback, this);
        blobSub = nh.subscribe("/kinect/player_position", 10, &ParticleFilter::blobDetectedCallback, this);
        initParticles();
    }

    ~ParticleFilter(){
        deleteOldParticles();
    };

    void peopleDetectedCallback(const player_tracker::PersonArray &people);
    void blobDetectedCallback(const geometry_msgs::Pose &pose);
    void track(const geometry_msgs::Pose &pose);
};


#endif //PARTICLEFILTER_H
