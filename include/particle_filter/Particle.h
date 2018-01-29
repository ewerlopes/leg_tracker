#ifndef PARTICLE_H
#define PARTICLE_H

#include <ros.h>
#include <eigen3/Eigen/Dense>


class Particle{
private:
    float weight;
    Eigen::Matrix<float, 5, 1> state;
    Eigen::Matrix<float, 5, 1> u;  // external motion (NO CONTROL INPUT ASSUMED) 
    Eigen::Matrix<float, 3, 5> H;  // measurement function 5 states - 3 observed (x,y, theta)
    Eigen::Matrix<float, 5, 5> I;  // identity matrix
    Eigen::Matrix<float, 3, 3> R;  // measurement uncertainty (3 uncorrelated measures with uncertainty)
    Eigen::Matrix<float, 5, 5> P;  // initial uncertainty
    Eigen::Matrix<float, 5, 5> F;  // Transition Matrix
    Eigen::Matrix<float, 5, 5> Q;  // process noise matrix
public:

    Particle(const Particle& that) {
        state = that.state;
        u = that.u;
        H = that.H;
        I = that.I;
        R = that.R;
        P = that.P;
        F = that.F;
        Q = that.Q;
        weight = that.weight;
    }

    Particle(const Particle* that) {
        state = that->state;
        u = that->u;
        H = that->H;
        I = that->I;
        R = that->R;
        P = that->P;
        F = that->F;
        Q = that->Q;
        weight = that->weight;
    }

    Particle(Eigen::Matrix<float, 5, 1> state): weight(0), state(state){

        u << 0,0,0,0,0;

        H << 1,0,0,0,0,           // measurement function 5 states - 3 observed (x, y, theta)
             0,1,0,0,0,
             0,0,1,0,0;  

        I << 1,0,0,0,0,           // identity matrix
             0,1,0,0,0,
             0,0,1,0,0,
             0,0,0,1,0,
             0,0,0,0,1;
    

        R << 0.5,0,0,               // measurement uncertainty (3 uncorrelated measures with uncertainty)
             0,0.5,0,
             0,0,0.5;

        P << 1,0,0,0,0,             // initial uncertainty
             0,1,0,0,0,
             0,0,1,0,0,
             0,0,0,1,0,
             0,0,0,0,1;            
                                             // Transition Matrix
        F << 1,0,0,0,0,             // x
             0,1,0,0,0,             // y
             0,0,1,0,0,             // theta
             0,0,0,1,0,             // x_dot
             0,0,0,0,1;             // y_dot

        Q << 0.1,0,0,0,0,           // process noise matrix
             0,0.1,0,0,0,
             0,0,0.01,0,0,
             0,0,0,0.01,0,
             0,0,0,0,0.01;
        }

    ~Particle(){

    };

    /*
     * Propagate using Kalman Predict equation
     */
    void propagate(){
        state = F * state + u;
        P = F*P*F.transpose() + Q;
    }

    void drift();

    float distance(Eigen::Vec2f obs) {
        Eigen::Vec2f pose = state.topRows(2);
        return (pose - obs).norm();
    }

    void fillPose(geometry_msgs::Pose &pose) {
        pose.position.x = state(0);
        pose.position.y = state(1);
        pose.orientation.z = state(2);
    }

    void setWeight(float weight) { this->weight = weight; }
    float getWeight() { return weight; }
    void normalizeWeight(float summation) { this->weight /= summation; }

    /*
     * Implements standard Kalman filter update equation
     */
    void update(float x, float y, float theta){
        Eigen::Matrix<float, 1, 3> Z;
        Z << x, y, theta;
        auto Y =  Z.transpose() - (H*state);
        auto S = ((H*P) * H.transpose()) + R;
        auto K = (P * H.transpose()) * S.inverse();
        state = state + (K * Y);
        P = (I - (K*H)) * P;
    }
};

#endif //PARTICLE_H
