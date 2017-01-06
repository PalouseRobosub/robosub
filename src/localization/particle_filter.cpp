#include "particle_filter.h"

ParticleFilter::ParticleFilter(int _num_particles)
{
    num_particles = _num_particles;

    norm_distribution = new std::normal_distribution<double>(0.0, 1.0);
    uniform_distribution = new std::uniform_real_distribution<double>(0.0, 1.0);

    initialize();
    reload_params();
}

ParticleFilter::~ParticleFilter()
{
    delete norm_distribution;
    delete uniform_distribution;
}

void ParticleFilter::initialize()
{
    num_iterations = 0;

    last_particle_states.clear();
    particle_states.clear();
    last_particle_weights.clear();
    particle_weights.clear();
    last_particle_obs.clear();
    particle_obs.clear();

    system_update_model.setZero();
    system_update_covar.setZero();
    measurement_covar.setZero();
    initial_state.setZero();
    observation.setZero();

    est_state.setZero();

    // system_update_model =
    // | 1  0  0  dt 0  0  |
    // | 0  1  0  0  dt 0  |
    // | 0  0  1  0  0  dt |
    // | 0  0  0  1  0  0  |
    // | 0  0  0  0  1  0  |
    // | 0  0  0  0  0  1  |
    system_update_model(0,0) = 1;
    system_update_model(1,1) = 1;
    system_update_model(2,2) = 1;
    system_update_model(3,3) = 1;
    system_update_model(4,4) = 1;
    system_update_model(5,5) = 1;

    // Intial std_devs of position around pinger
    initial_distribution.setZero();

    for(int n=0; n<num_particles; n++)
    {
        Matrix<double,6,1> s;
        for(int j=0; j<initial_state.rows(); j++)
        {
            s(j,0) = initial_state(j,0) + randn() * initial_distribution(j,0);
            particle_states.push_back(s);
        }
        last_particle_states = particle_states;

        // Zeroed at this point
        particle_obs.push_back(observation);
        last_particle_obs.push_back(observation);
        particle_weights.push_back(0);
    }

    ROS_INFO_STREAM("Finished PF init");
}

void ParticleFilter::Reset()
{
    initialize();
    reload_params();
}

void ParticleFilter::reload_params()
{
    if(!ros::param::getCached("hydrophones/pinger/depth", pinger_depth))
    {
        ROS_ERROR("pinger depth failed to load");
        ros::shutdown();
    }

    if(!ros::param::has("localization/"))
    {
        ROS_FATAL("localization parameters not found");
        ros::shutdown();
    }

    ros::param::getCached("localization/initial_state/x_pos", initial_state(0,0));
    ros::param::getCached("localization/initial_state/y_pos", initial_state(1,0));
    ros::param::getCached("localization/initial_state/z_pos", initial_state(2,0));
    ros::param::getCached("localization/initial_state/x_lin_vel", initial_state(3,0));
    ros::param::getCached("localization/initial_state/y_lin_vel", initial_state(4,0));
    ros::param::getCached("localization/initial_state/z_lin_vel", initial_state(5,0));

    ros::param::getCached("localization/variance/state_update_x_pos_variance", system_update_covar(0,0));
    ros::param::getCached("localization/variance/state_update_y_pos_variance", system_update_covar(1,1));
    ros::param::getCached("localization/variance/state_update_z_pos_variance", system_update_covar(2,2));
    ros::param::getCached("localization/variance/state_update_x_vel_pos_variance", system_update_covar(3,3));
    ros::param::getCached("localization/variance/state_update_y_vel_pos_variance", system_update_covar(4,4));
    ros::param::getCached("localization/variance/state_update_z_vel_pos_variance", system_update_covar(5,5));

    ros::param::getCached("localization/variance/x_pos_initial_stddev", initial_distribution(0,0));
    ros::param::getCached("localization/variance/y_pos_initial_stddev", initial_distribution(1,0));
    ros::param::getCached("localization/variance/z_pos_initial_stddev", initial_distribution(2,0));
    ros::param::getCached("localization/variance/x_lin_vel_initial_stddev", initial_distribution(3,0));
    ros::param::getCached("localization/variance/y_lin_vel_initial_stddev", initial_distribution(4,0));
    ros::param::getCached("localization/variance/z_lin_vel_initial_stddev", initial_distribution(5,0));

    ros::param::getCached("localization/variance/hydrophone_x_variance", measurement_covar(0,0));
    ros::param::getCached("localization/variance/hydrophone_y_variance", measurement_covar(1,1));
    ros::param::getCached("localization/variance/hydrophone_z_variance", measurement_covar(2,2));
    ros::param::getCached("localization/variance/lin_velocity_x_variance", measurement_covar(3,3));
    ros::param::getCached("localization/variance/lin_velocity_y_variance", measurement_covar(4,4));
    ros::param::getCached("localization/variance/lin_velocity_z_variance", measurement_covar(5,5));
    ros::param::getCached("localization/variance/depth_variance", measurement_covar(6,6));
}

tf::Vector3 ParticleFilter::GetPosition()
{
    return estimated_position;
}

// TODO: Should input functions have mutexes since they modify observation
// directly?

void ParticleFilter::InputDepth(const double depth)
{
    observation(6,0) = depth;
}

void ParticleFilter::InputHydrophone(const tf::Vector3 position)
{
    observation(0,0) = position.getX();
    observation(1,0) = position.getY();
    observation(2,0) = position.getZ();
}

void ParticleFilter::InputLinAccel(const tf::Vector3 linaccel, const double dt)
{
    observation(3,0) = linaccel.getX();
    observation(4,0) = linaccel.getY();
    observation(5,0) = linaccel.getZ();
    system_update_model(0,3) = dt;
    system_update_model(1,4) = dt;
    system_update_model(2,5) = dt;
}

Matrix<double,7,1> ParticleFilter::state_to_observation(Matrix<double,6,1> state)
{
    Matrix<double,7,1> obs;

    // Position relative to pinger
    obs(0,0) = state(0,0);
    obs(1,0) = state(1,0);
    obs(2,0) = state(2,0);

    // Lin Accel
    obs(3,0) = state(3,0);
    obs(4,0) = state(4,0);
    obs(5,0) = state(5,0);

    obs(6,0) = pinger_depth + state(2,0);

    return obs;
}

Matrix<double,7,1> ParticleFilter::add_observation_noise(Matrix<double,7,1> obs)
{
    return obs + (sqrt_elementwise(measurement_covar) * randn_mat(obs.rows(), 1));
}

void ParticleFilter::update_particle_states()
{
    for(int n=0; n<num_particles; n++)
    {
        // Update each particle_states state based on the update model plus noise
        particle_states[n] = system_update_model * last_particle_states[n] + sqrt_elementwise(system_update_covar) * randn_mat(6,1);

        // Update each particle_states observation based on its state
        particle_obs[n] = state_to_observation(particle_states[n]);
        // add noise to observations
        particle_obs[n] = add_observation_noise(particle_obs[n]);
    }
}

void ParticleFilter::update_particle_weights()
{
    double particle_weights_sum = 0.0;
    for(int n=0; n<num_particles; n++)
    {
        // Calculate particle weights
        particle_weights[n] = 1.0;
        for(unsigned int i=0; i<observation.rows(); i++)
        {
            // TODO: Update selectively based on whether sensor readings are
            // current or not?

            particle_weights[n] *= gaussian_prob(observation(i, 0),
                    std::sqrt(measurement_covar(i, i)),
                    particle_obs[n](i, 0));
        }

        // Need to get sum for normalization
        particle_weights_sum += particle_weights[n];
    }
    last_particle_obs = particle_obs;

    // Normalize weights so that they sum to 1
    for(int n=0; n<num_particles; n++)
    {
        if(particle_weights_sum == 0.0)
        {
            // TODO: Reinit here
            particle_weights[n] = 1.0/(float)num_particles;
        }
        else
        {
            particle_weights[n] /= particle_weights_sum;
        }
    }
}

void ParticleFilter::resample_particles()
{
    // Resample particles
    std::vector<Matrix<double, 6,1> > p;
    int index = int(randu() * num_particles);
    double beta = 0.0;
    double max_w = vector_max(particle_weights);
    for(int i=0; i<num_particles; i++)
    {
        beta += randu() * 2.0 * max_w;

        while(beta > particle_weights[index])
        {
            beta -= particle_weights[index];
            index = (index + 1) % num_particles;
        }
        p.push_back(particle_states[index]);
    }
    particle_states = p;
    last_particle_states = particle_states;
}

void ParticleFilter::estimate_state()
{
    // Estimate state from all particle_states
    // Just weighted averaging for now
    est_state.setZero();
    Matrix<double,6,1> state_sum;
    for(int i=0; i<num_particles; i++)
    {
        est_state += particle_states[i] * particle_weights[i];
    }

    for(int i=0; i<est_state.rows(); i++)
    {
        if(std::isnan(est_state(i, 0)))
        {
            est_state(i, 0) = 0.0;
        }
    }

    estimated_position.setX(est_state(0,0));
    estimated_position.setY(est_state(1,0));
    estimated_position.setZ(est_state(2,0));
}

void ParticleFilter::Update()
{
    PRINT_THROTTLE(ROS_INFO_STREAM("=================================="););

    reload_params();

    update_particle_states();

    update_particle_weights();

    resample_particles();

    estimate_state();

    // TODO: Add sanity check to estimated state and reinitialize filter if
    // needed (Possibly just check if est_state is roughly within bounds of
    // pool)

    PRINT_THROTTLE(ROS_INFO_STREAM("est_state: " << std::endl << est_state););

    num_iterations++;
}
