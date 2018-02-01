#include "particle_filter/ParticleFilter.h"
#include <math.h>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */


float rand_FloatRange(float a, float b){
    return ((b - a) * ((float)rand() / RAND_MAX)) + a;
}

ParticleFilter::ParticleFilter(int num_particles) : num_particles(num_particles) {
    particles.resize(num_particles);
    pub = nh.advertise<geometry_msgs::PoseArray>("/pf_cloud", 10);
    personDetectedSub = nh.subscribe("/people_tracked", 10, &ParticleFilter::peopleDetectedCallback, this);
    blobSub = nh.subscribe("/kinect/player_position", 10, &ParticleFilter::blobDetectedCallback, this);
    initParticles();
}

ParticleFilter::~ParticleFilter() {
    deleteOldParticles();
};

void ParticleFilter::initParticles(const geometry_msgs::Pose &pose){
    #pragma omp parallel for    
    for (int p=0; p < particles.size();p++){
        Eigen::Matrix<float, 5, 1> state;
        state << pose.position.x, pose.position.y, rand_FloatRange(0, 2*M_PI), 0, 0;
        particles[p] = new Particle(state);
    }
}


void ParticleFilter::initParticles() {  // particles init at random
    #pragma omp parallel for    
    for (int p=0; p < particles.size();p++) {
        Eigen::Matrix<float, 5, 1> state;
        // assumed -10 lowest x, 10 highest x possible (field of robogame)
        state << rand_FloatRange(-10, 10), rand_FloatRange(-10, 10), rand_FloatRange(0, 2*M_PI), 0, 0;  // TODO fix ranges of field
        particles[p] = new Particle(state);
    }
}

void ParticleFilter::propagateParticles() {
    #pragma omp parallel for
    for (int i = 0; i < particles.size(); i++) {
        particles[i]->propagate();
    }
}

void ParticleFilter::updateParticles(Eigen::Matrix<float, 1, 3> &Z) {
    #pragma omp parallel for
    for (int i = 0; i < particles.size(); i++) {
        particles[i]->update(Z);
    }
}

void ParticleFilter::computeWeights(Eigen::Matrix<float, 1, 3> &obs) {
    float summation = 0.0f;

    #pragma omp parallel for reduction(+:summation)
    for (int i = 0; i < particles.size(); i++) {
        particles[i]->setWeight(1 / particles[i]->distance(obs));
        summation += particles[i]->getWeight();
    }

    #pragma omp parallel for
    for (int i = 0; i < particles.size(); i++) {
        particles[i]->normalizeWeight(summation);
    }
}

void ParticleFilter::computeKinectWeights(Eigen::Matrix<float, 1, 3> &obs) {
    float summation = 0.0f;

    #pragma omp parallel for reduction(+:summation)
    for (int i = 0; i < particles.size(); i++) {
        particles[i]->setWeight(std::exp(- particles[i]->distance(obs) / this->tao)); // numerator of Boltzmann
        summation += particles[i]->getWeight();  // denom of Boltzmann weights
    }

    #pragma omp parallel for
    for (int i = 0; i < particles.size(); i++) {
        particles[i]->normalizeWeight(summation);    // normalization of weights computed with Boltzmann
    }
}


void ParticleFilter::resample()
{
    ParticleList newParticles;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<float> gauss(WHITE_NOISE_MEAN, WHITE_NOISE_STD);
    
    for (int i = 0; i < particles.size(); i++) {
        int qt = round(((float)particles.size()) * particles[i]->getWeight());
        for (int j = 0; j < qt; j++) {
            ParticlePtr perturbated = particles[i]->perturbate(gauss(gen), gauss(gen));
            newParticles.push_back(perturbated);
        }
        if (newParticles.size() >= particles.size()) {
            break;
        }
    }

    std::cout << "num particles: " << newParticles.size() << std::endl;
    deleteOldParticles();

    particles = newParticles;
}


void ParticleFilter::track(const geometry_msgs::Pose &pose) {
    
    Eigen::Matrix<float, 1, 3> obs;
    obs << pose.position.x, pose.position.y, pose.orientation.z;

    updateParticles(obs);
    propagateParticles();
    computeWeights(obs);
    resample();
    publishParticles();
    // missing update of velocity and theta!
}

void ParticleFilter::deleteOldParticles() {
    #pragma omp parallel for
    for (int i = 0; i < particles.size(); i++) {
        delete particles[i];
    }
}

void ParticleFilter::publishParticles() {
    geometry_msgs::PoseArray cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "/map";

    #pragma omp parallel for
    for (int i = 0; i < particles.size(); i++) {
        geometry_msgs::Pose pose;
        particles[i]->fillPose(pose);
        #pragma omp critical
        cloud.poses.push_back(pose);
    }

    pub.publish(cloud);

}

void ParticleFilter::peopleDetectedCallback(const player_tracker::PersonArray &msg){
    for (int i = 0; i < msg.people.size(); i++) {
        // TODO add gating here to avoid update everybody!
        track(msg.people[i].pose);
    }
}


void ParticleFilter::blobDetectedCallback(const geometry_msgs::Pose &pose) {
    track(pose);
}
