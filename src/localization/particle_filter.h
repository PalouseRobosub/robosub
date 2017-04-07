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

#include "localization/filter_utilities.h"

using namespace Eigen;
using namespace filter_utilities;

class ParticleFilter
{
public:
    ParticleFilter(ros::NodeHandle &_nh);

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
    Vector4d state_to_observation(Vector3d state);
    Vector4d add_observation_noise(
        Vector4d particle_obs);
    void update_particle_states();
    void update_particle_weights();
    void resample_particles();
    void estimate_state();
    void predict();
    void update();

    // Nodehandle received from localization_system
    ros::NodeHandle &nh;

    // Publisher for point cloud of particles.
    ros::Publisher particle_cloud_pub;

    // Set to true when estimate_state() called.
    bool new_position;

    // Number of particles. Comes from localization param.
    int num_particles;

    // Number of times update and predict steps have been run.
    int num_iterations;

    // Stores estimated position and metadata.
    tf::Vector3 estimated_position;
    ros::Time last_estimated_position_time;
    double estimated_position_dt;

    // Stores position of pinger in pool. Loaded from param.
    tf::Vector3 pinger_position;

    // Vectors of all particle states.
    // Each particle state is a column vector as follows:
    // | x_pos |
    // | y_pos |
    // | z_pos |
    std::vector<Vector3d> particle_states;
    std::vector<Vector3d> last_particle_states;

    // Vector of particle weights
    std::vector<double> particle_weights;

    // This stores the standard deviations of the observation inputs. Loaded
    // from params.
    Matrix<double, 4, 4> observation_stddev;

    // This stores the current hydrophone and depth observations as a column
    // vector as follows:
    // | azimuth     |
    // | inclination |
    // | range       |
    // | depth       |
    Vector4d observation;

    // Stores the initial state of the sub. Loaded from params.
    Vector3d initial_state;

    // This stores the standard deviations of the initial state in order to
    // generate the initial particles. Loaded from params.
    Vector3d initial_distribution;

    // This pushes the state forward based on the previous state. Since the
    // state for each particle is only the subs position this is simply an
    // identity matrix.
    Matrix<double, 3, 3> system_update_model;

    // This stores the standard deviations of the system update step. This adds
    // noise to the particles during the system update step.
    Matrix<double, 3, 3> system_update_stddev;

    // This relates the control input to the state. It is currently an
    // identity matrix * dt.
    Matrix<double, 3, 3> control_update_model;

    // This stores the inputted linear velocity.
    Vector3d control_input;
};
#endif //PARTICLE_FILTER_H
