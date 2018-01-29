#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H

#include <ros.h>
#include <eigen3/Eigen/Dense>
#include "Particle.h"

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
    int num_particles;                          // number of particles
    std::vector<Particle*> particles;
    float tao;                  // temperature parameter used to compute the weights of the particles with kinect observations

    void resample();
    void propagateParticles();
    void updateParticles()
    void publishParticles();
    void initParticles();
    void initParticles(const geometry_msgs::Pose &pose);
    
    void computeKinectWeights(Eigen::Vec2f &obs);
    void computeWeights(Eigen::Vec2f &obs);

public:
    
    ParticleFilter(num_particles=2000): num_particles(num_particles), particles(num_particles){
        pub = nh.advertise<geometry_msgs::PoseArray>("/pf_cloud", 10);
        personDetectedSub = nh.subscribe("/people_tracker/pose_array", 10, &ParticleFilter::peopleDetectedCallback, this);
        blobSub = nh.subscribe("/people_tracker/pose_array", 10, &ParticleFilter::blobDetectedCallback, this);
        initParticles();
    }

    ~ParticleFilter(){
        deleteOldParticles();
    };

    void peopleDetectedCallback(const geometry_msgs::PoseArray &people);
    void blobDetectedCallback(const geometry_msgs::Pose &pose);
    void track(const geometry_msgs::Pose &pose);
};


#endif //PARTICLEFILTER_H
