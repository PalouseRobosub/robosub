#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <random>
#include <vector>

#include "ros/console.h"
#include "ros/ros.h"
#include "tf/transform_datatypes.h"

#include "geometry_msgs/Point32.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/ChannelFloat32.h"

using namespace Eigen;

#define PF_PT_RATE 10
#define PF_PRINT_THROTTLE(x) if(num_iterations % PF_PT_RATE == 0) { x }
//#define PF_PRINT_THROTTLE(x) if(0 && num_iterations % PF_PT_RATE == 0) { x }

constexpr double RAD_TO_DEG = (180.0 / 3.1415);
constexpr double DEG_TO_RAD = (3.1415 / 180.0);
constexpr double PI = (3.1415);

double vector_max(std::vector<double> v)
{
    if(v.size() == 0)
    {
        return 0.0;
    }

    double max = v[0];
    for(unsigned int i = 1; i < v.size(); i++)
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
    for(int i = 0; i < in.rows(); i++)
    {
        for(int j = 0; j < in.cols(); j++)
        {
            out(i, j) = std::sqrt(in(i, j));
        }
    }
    return out;
}

double gaussian_prob(double mean, double sigma, double x)
{
    double p = std::exp(- std::pow((mean - x), 2) / std::pow(sigma, 2) * 2.0) /
               std::sqrt(2.0 * 3.1415 * std::pow(sigma, 2));

    return p;
}

class ParticleFilter
{
public:
    ParticleFilter(ros::NodeHandle *_nh, int _num_particles);
    ~ParticleFilter();

    bool NewPosition();
    tf::Vector3 GetPosition();
    double GetPositionDT();
    void InputDepth(const double depth, const double dt);
    void InputHydrophones(const tf::Vector3 position, const double dt);
    void InputAbsLinVel(const tf::Vector3 lin_vel, const double dt);
    void Reset();

private:
    void initialize();
    void reload_params();
    void publish_point_cloud();
    Matrix<double, 4, 1> state_to_observation(Matrix<double, 3, 1> state);
    Matrix<double, 4, 1> add_observation_noise(
        Matrix<double, 4, 1> particle_obs);
    void update_particle_states();
    void update_particle_weights();
    void resample_particles();
    void estimate_state();
    void predict();
    void update();

    ros::Publisher particle_cloud_pub;
    ros::NodeHandle *nh;

    bool new_position;

    int num_particles;
    int num_iterations;

    double estimated_position_dt;
    ros::Time last_estimated_position_time;
    tf::Vector3 estimated_position;
    double pinger_depth;

    // Multiply system_update_model x state(k-1) to get state(k)
    Matrix<double, 3, 3> system_update_model;
    // I'm not 100% sure on this. Covariance matrices are magic
    Matrix<double, 3, 3> system_update_covar;
    Matrix<double, 3, 3> control_update_model;

    Matrix<double, 3, 1> initial_distribution;
    Matrix<double, 4, 4> measurement_covar;

    Matrix<double, 4, 1> observation;
    Matrix<double, 3, 1> control_input;

    Matrix<double, 3, 1> est_state;
    Matrix<double, 3, 1> initial_state;

    // Could convert to 3-d matrices
    std::vector<Matrix<double, 3, 1> > last_particle_states;
    std::vector<Matrix<double, 3, 1> > particle_states;
    std::vector<double> last_particle_weights;
    std::vector<double> particle_weights;

    // Particle observations
    std::vector<Matrix<double, 4, 1> > last_particle_obs;
    std::vector<Matrix<double, 4, 1> > particle_obs;

    std::default_random_engine rand_generator;
    std::normal_distribution<double> *norm_distribution;
    std::uniform_real_distribution<double> *uniform_distribution;

    // This is meant to emulate the matlab function randn which
    // returns a random value from a normal distribution with
    // mean = 0 std dev = 1
    double randn()
    {
        return (*norm_distribution)(rand_generator);
    }

    double pos_randn()
    {
        return std::abs((*norm_distribution)(rand_generator));
    }

    // Uniform 0-1 random real
    double randu()
    {
        return (*uniform_distribution)(rand_generator);
    }

    MatrixXd randn_mat(int rows, int cols)
    {
        MatrixXd r(rows, cols);
        for(int i = 0; i < rows; i++)
        {
            for(int j = 0; j < cols; j++)
            {
                r(i, j) = randn();
            }
        }
        return r;
    }
};
#endif //PARTICLE_FILTER_H
