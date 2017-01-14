#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <random>
#include <vector>

#include "ros/console.h"
#include "ros/ros.h"
#include "tf/transform_datatypes.h"

using namespace Eigen;

#define PT_RATE 10
#define PRINT_THROTTLE(x) if(num_iterations % PT_RATE == 0) { x }
//#define PRINT_THROTTLE(x) if(0 && num_iterations % PT_RATE == 0) { x }

template <typename T>
std::ostream& operator<< (std::ostream& out, const std::vector<T>& v) 
{
  if ( !v.empty() ) 
  {
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

double vector_max(std::vector<double> v)
{
    if(v.size() == 0)
    {
        return 0.0;
    }

    double max = v[0];
    for(unsigned int i=1; i<v.size(); i++)
    {
        if(v[i] > max)
        {
            max = v[i];
        }
    }

    return max;
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

double gaussian_prob(double mean, double sigma, double x)
{
    double p = std::exp(- std::pow((mean - x), 2) / std::pow(sigma, 2) / 2.0) / std::sqrt(2.0 * 3.1415 * std::pow(sigma, 2));
    //PRINT_THROTTLE(ROS_INFO_STREAM("gaussian_prob(" << mean << "), " << sigma << "), "
    //PRINT_THROTTLE(ROS_INFO("gaussian_prob(%f, %f, %f) = %f", mean, sigma, x, p););
    return p;
}

class ParticleFilter
{
public:
    ParticleFilter(int _num_particles);
    ~ParticleFilter();
    void Update();
    tf::Vector3 GetPosition();
    void Reset();

    void InputDepth(const double depth, const ros::Time msg_time);
    void InputHydrophone(const tf::Vector3 position, const ros::Time msg_time);
    void InputLinAccel(const tf::Vector3 linaccel, const double dt, const ros::Time msg_time);

private:
    void initialize();
    void reload_params();

    void update_particle_states();
    void update_particle_weights();
    void resample_particles();
    void estimate_state();
    void zero_system_update_dt();

    Matrix<double,7,1> state_to_observation(Matrix<double,6,1> state, Matrix<double,6,1> last_state, double dt);
    Matrix<double,7,1> add_observation_noise(Matrix<double,7,1> particle_obs);

    int num_particles;
    int num_iterations;
    double pinger_depth;
    ros::Time last_update_time;
    ros::Duration update_dt;
    ros::Time last_hydrophone_time;
    bool new_hydrophone;

    tf::Vector3 estimated_position;

    // Multiply system_update_model x state(k-1) to get state(k)
    Matrix<double,6,6> system_update_model;
    // I'm not 100% sure on this. Covariance matrices are magic
    Matrix<double,6,6> system_update_covar;

    Matrix<double,6,1> initial_distribution;
    Matrix<double,7,7> measurement_covar;

    Matrix<double,7,1> observation;

    Matrix<double,6,1> est_state;
    Matrix<double,6,1> initial_state;

    // Could convert to 3-d matrices
    std::vector<Matrix<double, 6,1> > last_particle_states;
    std::vector<Matrix<double, 6,1> > particle_states;
    std::vector<double> last_particle_weights;
    std::vector<double> particle_weights;

    // Particle observations
    std::vector<Matrix<double, 7,1> > last_particle_obs;
    std::vector<Matrix<double, 7,1> > particle_obs;

    std::default_random_engine rand_generator;
    std::normal_distribution<double> *norm_distribution;
    std::uniform_real_distribution<double> *uniform_distribution;

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
};