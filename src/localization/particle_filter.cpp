#include "localization/particle_filter.h"
#include <vector>
#include <tf/tf.h>

ParticleFilter::ParticleFilter(ros::NodeHandle &_nh):
    nh(_nh)
{
    particle_cloud_pub =
        nh.advertise<sensor_msgs::PointCloud>("localization/particles", 1);

    if(!ros::param::getCached("localization/particle_filter/num_particles",
                num_particles))
    {
            ROS_FATAL_STREAM("Failed to load number of particles.");
            ros::shutdown();
    }

    initialize();
}

bool ParticleFilter::NewPosition()
{
    return new_position;
}

tf::Vector3 ParticleFilter::GetPosition()
{
    new_position = false;
    return estimated_position;
}

double ParticleFilter::GetPositionDT()
{
    return estimated_position_dt;
}

void ParticleFilter::InputDepth(const double depth)
{
    observation(3, 0) = depth;
}

void ParticleFilter::InputHydrophones(const tf::Vector3 bearing, const double
        dt)
{
    // bearing towards the pinger is represented in i,j,k vector notation
    observation(0, 0) = bearing[0];
    observation(1, 0) = bearing[1];
    observation(2, 0) = bearing[2];

    update();
}

void ParticleFilter::InputAbsLinVel(const tf::Vector3 lin_vel, const double dt)
{
    // Set control_input to linear velocity
    control_input(0, 0) = lin_vel[0];
    control_input(1, 0) = lin_vel[1];
    control_input(2, 0) = lin_vel[2];

    // Set control_update_model to I * dt
    control_update_model(0, 0) = dt;
    control_update_model(1, 1) = dt;
    control_update_model(2, 2) = dt;

    predict();
}

void ParticleFilter::InputOrientation(const tf::Quaternion orientation)
{
    current_orientation = orientation;
}

void ParticleFilter::Reset()
{
    initialize();
}

void ParticleFilter::initialize()
{
    num_iterations = 0;

    // Clear vectors
    last_particle_states.clear();
    particle_states.clear();
    particle_weights.clear();

    // You get a zero and you get a zero! Everybody gets a zero.
    system_update_model.setZero();
    control_input.setZero();
    control_update_model.setZero();
    system_update_stddev.setZero();
    observation_stddev.setZero();
    initial_state.setZero();
    observation.setZero();
    initial_distribution.setZero();

    // Load all necessary parmeters.
    if(!ros::param::has("localization/"))
    {
        ROS_FATAL("localization parameters not found");
        ros::shutdown();
    }

    if(!ros::param::has("localization/pinger/depth"))
    {
        ROS_FATAL("pinger depth failed to load");
        ros::shutdown();
    }

    reload_params();

    ROS_DEBUG_STREAM("initial_state:\n" << initial_state);
    ROS_DEBUG_STREAM("initial_distribution:\n" << initial_distribution);
    ROS_DEBUG_STREAM("system_update_stddev:\n" << system_update_stddev);
    ROS_DEBUG_STREAM("observation_stddev:\n" << observation_stddev);

    // Set up system update model.
    system_update_model.setIdentity();

    // Initialize all particles
    for(int n = 0; n < num_particles; n++)
    {
        // Randomly generate particle around initial state based on initial
        // gaussian distribution.
        Vector3d s;
        s(0, 0) = initial_state(0, 0) + (rand_normal() *
                initial_distribution(0, 0));
        s(1, 0) = initial_state(1, 0) + (rand_normal() *
                initial_distribution(1, 0));
        s(2, 0) = initial_state(2, 0) + (rand_normal() *
                initial_distribution(2, 0));

        particle_states.push_back(s);

        // Since we have no information at this point, set all particles
        // equally likely.
        particle_weights.push_back(1.0/num_particles);
    }
    last_particle_states = particle_states;

    new_position = false;
    estimated_position_dt = 0.0;
    last_estimated_position_time = ros::Time::now();

    ROS_INFO_STREAM("Finished PF init");
}

void ParticleFilter::reload_params()
{
    // Load pinger_depth and set pinger position. Currently the pinger is the
    // origin so the x and y of the pinger position is just 0.
    double pinger_depth = 0.0;
    ROS_FATAL_COND(!ros::param::getCached("localization/pinger/depth",
                pinger_depth), "localization/pinger/depth failed to load");
    pinger_position[0] = 0.0;
    pinger_position[1] = 0.0;
    pinger_position[2] = pinger_depth;

    ROS_FATAL_COND(
            !ros::param::getCached("localization/particle_filter/initial/x",
                initial_state(0, 0)), "localization/particle_filter/initial/x"
            " failed to load");
    ROS_FATAL_COND(
            !ros::param::getCached("localization/particle_filter/initial/y",
                initial_state(1, 0)), "localization/particle_filter/initial/y"
            " failed to load");
    ROS_FATAL_COND(
            !ros::param::getCached("localization/particle_filter/initial/z",
                initial_state(2, 0)), "localization/particle_filter/initial/z"
            " failed to load");

    ROS_FATAL_COND(!ros::param::getCached(
            "localization/particle_filter/initial/x_stddev",
            initial_distribution(0, 0)),
            "localization/particle_filter/initial/x_stddev failed to load");
    ROS_FATAL_COND(!ros::param::getCached(
            "localization/particle_filter/initial/y_stddev",
            initial_distribution(1, 0)),
            "localization/particle_filter/initial/y_stddev failed to load");
    ROS_FATAL_COND(!ros::param::getCached(
            "localization/particle_filter/initial/z_stddev",
            initial_distribution(2, 0)),
            "localization/particle_filter/initial/z_stddev failed to load");

    ROS_FATAL_COND(!ros::param::getCached(
            "localization/particle_filter/stddev/x_state_update",
            system_update_stddev(0, 0)),
            "localization/particle_filter/stddev/x_state_update failed to"
            " load");
    ROS_FATAL_COND(!ros::param::getCached(
            "localization/particle_filter/stddev/y_state_update",
            system_update_stddev(1, 1)),
            "localization/particle_filter/stddev/y_state_update failed to"
            " load");
    ROS_FATAL_COND(!ros::param::getCached(
            "localization/particle_filter/stddev/z_state_update",
            system_update_stddev(2, 2)),
            "localization/particle_filter/stddev/z_state_update failed to"
            " load");

    ROS_FATAL_COND(!ros::param::getCached(
            "localization/particle_filter/stddev/i",
            observation_stddev(0, 0)),
            "localization/particle_filter/stddev/i failed to load");
    ROS_FATAL_COND(!ros::param::getCached(
            "localization/particle_filter/stddev/j",
            observation_stddev(1, 1)),
            "localization/particle_filter/stddev/j failed to load");
    ROS_FATAL_COND(!ros::param::getCached(
            "localization/particle_filter/stddev/k",
            observation_stddev(2, 2)),
            "localization/particle_filter/stddev/k failed to load");
    ROS_FATAL_COND(!ros::param::getCached(
            "localization/particle_filter/stddev/depth",
            observation_stddev(3, 3)),
            "localization/particle_filter/stddev/depth failed to load");
}

void ParticleFilter::publish_point_cloud()
{
    // Set up pointcloud message with each point cooresponding to a particle
    // plus an additional weights channel.
    sensor_msgs::PointCloud point_cloud;
    sensor_msgs::ChannelFloat32 chan;
    chan.name = "weight";

    for(int i = 0; i < num_particles; i++)
    {
        geometry_msgs::Point32 p;
        p.x = particle_states[i](0, 0);
        p.y = particle_states[i](1, 0);
        p.z = particle_states[i](2, 0);

        point_cloud.points.push_back(p);

        chan.values.push_back(particle_weights[i]);
    }

    point_cloud.header.frame_id = "world";
    point_cloud.channels.push_back(chan);
    point_cloud.header.stamp = ros::Time::now();

    particle_cloud_pub.publish(point_cloud);
}

Vector4d ParticleFilter::state_to_observation(Vector3d state)
{
    Vector4d obs;
    tf::Vector3 bearing;

    // create a vector pointing from the pinger to the particle
    bearing.setX(state(0, 0) - pinger_position[0]);
    bearing.setY(state(1, 0) - pinger_position[1]);
    bearing.setZ(state(2, 0) - pinger_position[2]);

    // normalize the vector to be a unit vector
    bearing.normalize();

    // invert the vector so it points from the particle to the pinger
    bearing *= -1;

    // rotate the bearing so that it is relative to the sub's current
    // orientation
    bearing = tf::Transform(current_orientation).inverse()(bearing);

    // fill out bearing observation
    obs(0, 0) = bearing.getX();
    obs(1, 0) = bearing.getY();
    obs(2, 0) = bearing.getZ();

    // fill out depth observation
    obs(3, 0) = state(2, 0);

    return obs;
}

void ParticleFilter::update_particle_states()
{
    // Update each particle state from the previous state based on the system
    // update model, control model, and system update noise. This pushes the
    // particles based on inputted linear velocity and adds some noise.

    MatrixXd noise;

    for(int n = 0; n < num_particles; n++)
    {
        noise = randn_mat(3, 1);

        ROS_DEBUG_STREAM("PF State noise update for particle " << n << ": " <<
                        noise);

        particle_states[n] = system_update_model * last_particle_states[n] +
            control_update_model * control_input +
            system_update_stddev * noise;
    }
}

void ParticleFilter::update_particle_weights()
{
    // TODO: Change to use unit-vector scalar angle as primary measurement
    // Sets weights for all particles based on the hydrophone and depth
    // observation. Also normalizes the weights afterward.
    double particle_weights_sum = 0.0;
    for(int n = 0; n < num_particles; n++)
    {
        Vector4d particle_obs;

        // Obtain a predicted observation based on a particles state.
        particle_obs = state_to_observation(particle_states[n]);

        /*
        // Add noise to predicted observation.
        particle_obs += (observation_stddev *
               randn_mat(particle_obs.rows(), 1));

        */
        // Calculate particle weights using the gaussian probability
        // distribution function for each observation.
        // TODO: Convert range to use rayleigh distribution per Brians
        // suggestion.
        particle_weights[n] = 1.0;
        for(unsigned int i = 0; i < observation.rows(); i++)
        {
            particle_weights[n] *= gaussian_prob(observation(i, 0),
                    observation_stddev(i, i), particle_obs(i, 0));
        }

        // Accumulate sum.
        particle_weights_sum += particle_weights[n];
    }

    // Normalize weights so that they sum to 1.
    for(int n = 0; n < num_particles; n++)
    {
        // This should never happen.
        if(particle_weights_sum == 0.0)
        {
            particle_weights[n] = 1.0 / static_cast<float>(num_particles);
        }
        else
        {
            particle_weights[n] /= particle_weights_sum;
        }
    }
}

void ParticleFilter::resample_particles()
{
    // Generate N new particles by drawing, with replacement, from the current
    // set of particles. Particles are selected conditionally with probability
    // based on their (previously normalized) weights. In the end the new
    // particles will satisfy a binomial distribution, that is, a particle with
    // weight W will appear in the new set approximately W*N times. Intuitively
    // the goal of this step is to concentrate particles in the most likely
    // locations for the sub to be in the iteration, increasing the accuracy of
    // the filter.

    std::vector<Vector3d> selected_particles;

    int index = static_cast<int>(rand_uniform() * num_particles);
    double beta = 0.0;
    double max_weight = *(std::max_element(std::begin(particle_weights),
                std::end(particle_weights)));

    for(int i = 0; i < num_particles; i++)
    {
        beta += rand_uniform() * 2.0 * max_weight;

        while(beta > particle_weights[index])
        {
            beta -= particle_weights[index];
            index = (index + 1) % num_particles;
        }
        selected_particles.push_back(particle_states[index]);
    }

    particle_states = selected_particles;
    last_particle_states = particle_states;
}

void ParticleFilter::estimate_state()
{
    Vector3d est_state;

    // Estimate state from all particles by averaging.
    // Note: Do not use a weighted average here since the resampling step will
    // not guarantee that the weights are normalized anymore.
    est_state.setZero();
    for(int i = 0; i < num_particles; i++)
    {
        est_state += particle_states[i];
    }

    est_state(0, 0) /= num_particles;
    est_state(1, 0) /= num_particles;
    est_state(2, 0) /= num_particles;

    PRINT_THROTTLE(ROS_DEBUG_STREAM("est_state:\n" << est_state););

    // Set estimated position return value.
    estimated_position[0] = est_state(0, 0);
    estimated_position[1] = est_state(1, 0);
    estimated_position[2] = est_state(2, 0);

    // Set estimated position metadata.
    new_position = true;
    estimated_position_dt = (ros::Time::now() -
            last_estimated_position_time).toSec();
    last_estimated_position_time = ros::Time::now();
}

// Update all particle states and restimate state. No weighting or resampling
// is done here.
void ParticleFilter::predict()
{
    update_particle_states();

    update_particle_weights();
    
    last_particle_states = particle_states;

    estimate_state();

    publish_point_cloud();

    num_iterations++;
}

// Update all particle states, generate weights, resample, and estimate state.
void ParticleFilter::update()
{
    reload_params();

    // Since we don't want to double count control input, zero it out.
    control_input.setZero();

    update_particle_states();

    update_particle_weights();

    publish_point_cloud();

    resample_particles();

    estimate_state();

    // publish_point_cloud();

    num_iterations++;
}
