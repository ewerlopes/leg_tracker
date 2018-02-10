#include "particle_filter/MultipleParticleFilter.h"

MultipleParticleFilter::MultipleParticleFilter(int numParticles, int maxClouds) : maxClouds(maxClouds)
{
    ParticleFilterPtr cloud = new ParticleFilter(numParticles);
    clouds.push_back(cloud);

    initRosCommunication();
}

void MultipleParticleFilter::initRosCommunication()
{
    pubClouds = nh.advertise<geometry_msgs::PoseArray>("/tracking_clouds", 10);
    subPeopleDetector = nh.subscribe("/people_tracked", 10, &MultipleParticleFilter::peopleDetectionCallback, this);
    subBlobDetector = nh.subscribe("/blob_detection", 10, &MultipleParticleFilter::blobDetectionCallback, this);
}

MultipleParticleFilter::~MultipleParticleFilter()
{
    deleteClouds();
}


void MultipleParticleFilter::deleteClouds()
{
    #pragma omp parallel for
    for (int i = 0; i < clouds.size(); i++) {
        deleteCloud(i);
    }
    clouds.clear();
}

void MultipleParticleFilter::deleteCloud(int index)
{
    delete clouds[index];
    clouds[index] = NULL;
}

void MultipleParticleFilter::track(int cloudIndex, const geometry_msgs::Pose &person)
{
    clouds[cloudIndex]->track(person);
}

void MultipleParticleFilter::peopleDetectionCallback(const player_tracker::PersonArray &people)
{
    if (people.people.size() == 0) {
        return;
    }

    AssociationMatrix association = AssociationMatrix::Zero(people.people.size(), clouds.size()); // #people X #clouds
    IntSet peopleToAssociate;

    #pragma omp parallel for
    for (int i = 0; i < people.people.size(); i++) {
        RowVector vec = RowVector::Zero(clouds.size());
        setAssociation(people.people[i].pose, vec);
        association.row(i) = vec;
        #pragma omp critical
        peopleToAssociate.insert(i);
    }

    #pragma omp parallel for
    for (int i = 0; i < clouds.size(); i++) {
        ROS_WARN("%d\n", i);
        ColVector vec = association.col(i);
        int j = getMostAssociatedPerson(vec);   // this must be disjoint-> each person must be unique, two people can not be associated to the same cloud
        ROS_WARN("index: %d\n", j);
        if (j == -1) {
            continue;
        }
        track(i, people.people[j].pose);
        #pragma omp critical
        peopleToAssociate.erase(j);
    }

    #pragma omp parallel for
    for (auto it = peopleToAssociate.begin(); it != peopleToAssociate.end(); ++it) {
        RowVector vec = association.row(*it);
        int j = getMostAssociatedCloud(vec);
        if (j == -1) {
            continue;
        }
        #pragma omp critical
        int new_j = cloneCloud(j);
        track(new_j, people.people[*it].pose);
    }

    publishClouds();
}

void MultipleParticleFilter::blobDetectionCallback(const player_tracker::Blob &person)
{
    if (!person.observed) {
        killObservedClouds();
    } else {

        RowVector association = RowVector::Zero(1, clouds.size());

        setAssociation(person.pose, association);
        int j = getMostAssociatedCloud(association);
        track(j, person.pose);

        ParticleFilterPtr cloud = extractCloud(j);
        
        // // do tracking but using only cloud associated!
        // // then clear all clouds and put new cloud
        // deleteClouds();
        // clouds.push_back(cloud);
    }

    publishClouds();
}

void MultipleParticleFilter::killObservedClouds()
{
    // for each cloud
    //      if cloud in fov
    //          delete cloud
}

void MultipleParticleFilter::setAssociation(const geometry_msgs::Pose &person, RowVector &association) 
{
    #pragma omp parallel for
    for (int i = 0; i < clouds.size(); i++) {
        association(i) = clouds[i]->computeAssociation(person);
    }
}

void MultipleParticleFilter::setAssociation(const geometry_msgs::Point &person, RowVector &association) 
{
    #pragma omp parallel for
    for (int i = 0; i < clouds.size(); i++) {
        association(i) = clouds[i]->computeAssociation(person);
    }
}

// most associated person, wrt the cloud, is the person with the lowest value
int MultipleParticleFilter::getMostAssociatedPerson(ColVector &association) const 
{
    std::cout << association.transpose() << std::endl;
    if (association.sum() == 0) {
        return -1;
    }
    Eigen::MatrixXf::Index index;
    association.minCoeff(&index);
    return index;
}

// most associated cloud, wrt the cloud, is the cloud with the lowest value
int MultipleParticleFilter::getMostAssociatedCloud(RowVector &association) const 
{
    if (association.sum() == 0) {
        return -1;
    }
    Eigen::MatrixXf::Index index;
    association.minCoeff(&index);
    return index;
}

int MultipleParticleFilter::cloneCloud(int cloudIndex)
{
    clouds.push_back(new ParticleFilter(*clouds[cloudIndex]));
    return clouds.size() - 1;
}

ParticleFilterPtr MultipleParticleFilter::extractCloud(int cloudIndex)
{
    ParticleFilterPtr cloud = clouds[cloudIndex];
    clouds.erase(clouds.begin() + cloudIndex);
    return cloud;
}

void MultipleParticleFilter::publishClouds()
{
    geometry_msgs::PoseArray cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "/map";

    #pragma omp parallel for
    for (int j = 0; j < clouds.size(); j++) {
        clouds[j]->fillPoseArray(cloud);
    }

    pubClouds.publish(cloud);
}