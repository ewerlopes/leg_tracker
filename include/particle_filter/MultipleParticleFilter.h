#ifndef MULTIPLE_PARTICLE_FILTER_H_
#define MULTIPLE_PARTICLE_FILTER_H_

#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <player_tracker/Blob.h>
#include <player_tracker/PersonArray.h>

#include "particle_filter/Particle.h"
#include "particle_filter/ParticleFilter.h"

typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> AssociationMatrix;
typedef Eigen::Matrix<float, Eigen::Dynamic, 1> RowVector;
typedef Eigen::Matrix<float, 1, Eigen::Dynamic> ColVector;

typedef std::set<int> IntSet;


class MultipleParticleFilter
{
private:
    int maxClouds;
    ParticleFilterList clouds;

    ros::NodeHandle nh;
    ros::Publisher pubClouds;
    ros::Subscriber subPeopleDetector;
    ros::Subscriber subBlobDetector;

protected:
    virtual void initRosCommunication();

    virtual void track(int cloudIndex, const geometry_msgs::Pose &person);
    virtual void killObservedClouds();

    virtual void setAssociation(const geometry_msgs::Pose &person, RowVector &association);
    virtual void setAssociation(const geometry_msgs::Point &person, RowVector &association);
    virtual int getMostAssociatedPerson(ColVector &association) const;
    virtual int getMostAssociatedCloud(RowVector &association) const;
    
    int cloneCloud(int cloudIndex);
    ParticleFilterPtr extractCloud(int cloudIndex);

    void deleteCloud(int index);
    void deleteClouds();

    void publishClouds();

public:
    MultipleParticleFilter(int numParticles=1000, int maxClouds=5);
    ~MultipleParticleFilter();

    void peopleDetectionCallback(const player_tracker::PersonArray &people);
    void blobDetectionCallback(const player_tracker::Blob &person);

};



#endif // MULTIPLE_PARTICLE_FILTER_H_
