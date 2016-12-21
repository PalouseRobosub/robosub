#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <random>
#include <vector>

#include "ros/console.h"
#include "ros/ros.h"
#include "tf/transform_datatypes.h"

#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "robosub/depth_stamped.h"
#include "robosub/PositionsStamped.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float32.h"
#include "std_srvs/Empty.h"

using namespace Eigen;

#define PRINT_THROTTLE(x) if(num_iterations % 25 == 0) { x }

template <typename T>
std::ostream& operator<< (std::ostream& out, const std::vector<T>& v) {
  if ( !v.empty() ) {
    out << '[';
    std::copy (v.begin(), v.end(), std::ostream_iterator<T>(out, ", "));
    out << "\b\b]";
  }
  return out;
}

std::vector<double> CumSum(std::vector<double> v)
{
    std::vector<double> cumsum;
    cumsum.push_back(v[0]);
    for(unsigned int i=1; i<v.size(); i++)
    {
        cumsum.push_back(cumsum[i-1] + v[i]);
    }

    return cumsum;
}


class LocalizationSystem
{
public:
    LocalizationSystem(double _dt, int _num_particles);
    ~LocalizationSystem();
    void InitializeParticleFilter();

    bool resetFilterCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &rep);
    void depthCallback(const robosub::depth_stamped::ConstPtr &msg);
    void hydrophoneCallback(const robosub::PositionsStamped::ConstPtr &msg);
    void linAccelCallback(const geometry_msgs::Vector3::ConstPtr &msg);

    void Update();

    Matrix<double,4,1> state_to_observation(Matrix<double,6,1> state);
    Matrix<double,4,1> add_observation_noise(Matrix<double,4,1> particle_obs);

    geometry_msgs::Vector3 GetLocalizationMessage();

private:

    int num_iterations;

    double dt;
    double num_particles;

    bool new_hydrophone;
    bool new_depth;
    bool new_lin_velocity;

    geometry_msgs::Vector3 lin_velocity;

    // Multiply system_update_model x state(k-1) to get state(k)
    Matrix<double,6,6> system_update_model;
    // I'm not 100% sure on this. Covariance matrices are magic
    Matrix<double,6,6> system_update_covar;

    Matrix<double,6,1> initial_distribution;
    // system_update_variance = [x_variance, y_variance, z_variance]
    //Matrix<double,3,1> system_update_variance;
    // measurement_covar is diagnal (currently at least) so easier
    // to understand
    Matrix<double,4,4> measurement_covar;
    // measurement_variance is noise of each sensor
    //Matrix<double,4,1> measurement_variance;

    // Observation is [hydrophones/position, depth] = [hx, hy, hz, depth]
    Matrix<double,4,1> last_observation;
    Matrix<double,4,1> observation;

    // state = [x, y, z, x_vel, y_vel, z_vel]
    // This should be global position and velocity
    Matrix<double,6,1> last_est_state;
    Matrix<double,6,1> est_state;
    Matrix<double,6,1> initial_state;

    // Could convert to 3-d matrices
    std::vector<Matrix<double, 6,1> > last_particle_states;
    std::vector<Matrix<double, 6,1> > particle_states;
    std::vector<double> last_particle_weights;
    std::vector<double> particle_weights;
    // Particle observations
    std::vector<Matrix<double, 4,1> > last_particle_obs;
    std::vector<Matrix<double, 4,1> > particle_obs;

    std::default_random_engine rand_generator;
    std::normal_distribution<double> *norm_distribution;
    std::uniform_real_distribution<double> *uniform_distribution;

public:
    // This is meant to emulate the matlab function randn which 
    // returns a random value from a normal distribution with
    // mean = 0 std dev = 1
    double randn() { return (*norm_distribution)(rand_generator); }
    // Uniform 0-1 random real
    double randu() { return (*uniform_distribution)(rand_generator); }
    MatrixXd randn_mat(int rows, int cols) 
    { 
        MatrixXd r(rows, cols);
        for(int i=0; i<rows; i++)
        {
            for(int j=0; j<cols; j++)
            {
                r(i,j) = randn();
            }
        }
        return r;
    }

    MatrixXd sqrt_elementwise(MatrixXd in)
    {
        MatrixXd out(in.rows(), in.cols());
        for(int i=0; i<in.rows(); i++)
        {
            for(int j=0; j<in.cols(); j++)
            {
                out(i,j) = std::sqrt(in(i,j));
            }
        }
        return out;
    }
};
