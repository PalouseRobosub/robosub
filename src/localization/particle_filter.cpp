#include "localization/particle_filter.h"
#include <vector>

ParticleFilter::ParticleFilter(int _num_particles)
{
    num_particles = _num_particles;

    norm_distribution = new std::normal_distribution<double>(0.0, 1.0);
    uniform_distribution =
                          new std::uniform_real_distribution<double>(0.0, 1.0);

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
    control_input.setZero();
    control_update_model.setZero();
    system_update_covar.setZero();
    measurement_covar.setZero();
    initial_state.setZero();
    observation.setZero();

    est_state.setZero();

    // system_update_model =
    // | 1  0  0  |
    // | 0  1  0  |
    // | 0  0  1  |
    system_update_model(0, 0) = 1;
    system_update_model(1, 1) = 1;
    system_update_model(2, 2) = 1;

    // Intial std_devs of position around pinger
    initial_distribution.setZero();

    reload_params();

    ROS_INFO_STREAM("initial_state:\n" << initial_state);
    ROS_INFO_STREAM("initial_distribution:\n" << initial_distribution);
    ROS_INFO_STREAM("system_update_covar:\n" << system_update_covar);
    ROS_INFO_STREAM("measurement_covar:\n" << measurement_covar);

    for(int n = 0; n < num_particles; n++)
    {
        Matrix<double, 3, 1> s;
        s(0, 0) = initial_state(0, 0) + (randn() * initial_distribution(0, 0));
        s(1, 0) = initial_state(1, 0) + (randn() * initial_distribution(1, 0));
        s(2, 0) = initial_state(2, 0) + (randn() * initial_distribution(2, 0));

        particle_states.push_back(s);

        // Zeroed at this point
        particle_obs.push_back(observation);
        last_particle_obs.push_back(observation);

        // Initially uniform probabilities
        particle_weights.push_back(1.0/num_particles);
    }
    last_particle_states = particle_states;

    ROS_INFO_STREAM("particle_states.size(): " << particle_states.size());
    ROS_DEBUG_STREAM("Finished PF init");
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

    ros::param::getCached("localization/initial/x",
                          initial_state(0, 0));
    ros::param::getCached("localization/initial/y",
                          initial_state(1, 0));
    ros::param::getCached("localization/initial/z",
                          initial_state(2, 0));

    ros::param::getCached("localization/initial/x_initial_stddev",
                          initial_distribution(0, 0));
    ros::param::getCached("localization/initial/y_initial_stddev",
                          initial_distribution(1, 0));
    ros::param::getCached("localization/initial/z_initial_stddev",
                          initial_distribution(2, 0));

    ros::param::getCached("localization/variances/x_state_update",
                          system_update_covar(0, 0));
    ros::param::getCached("localization/variances/y_state_update",
                          system_update_covar(1, 1));
    ros::param::getCached("localization/variances/z_state_update",
                          system_update_covar(2, 2));

    ros::param::getCached("localization/variances/azimuth_measurement",
                          measurement_covar(0, 0));
    ros::param::getCached("localization/variances/inclination_measurement",
                          measurement_covar(1, 1));
    ros::param::getCached("localization/variances/range_measurement",
                          measurement_covar(2, 2));
    ros::param::getCached("localization/variances/depth_measurement",
                          measurement_covar(3, 3));

    measurement_covar(0, 0) *= DEG_TO_RAD;
    measurement_covar(1, 1) *= DEG_TO_RAD;
}

tf::Vector3 ParticleFilter::GetPosition()
{
    return estimated_position;
}

void ParticleFilter::InputDepth(const double depth, const double dt)
{
    observation(3, 0) = depth;
}

void ParticleFilter::InputHydrophones(const tf::Vector3 position, const double dt)
{
    double azimuth = std::atan2(position[1], position[0]);
    double inclination = std::atan2(position[2],
            std::sqrt(std::pow(position[0], 2) + std::pow(position[1], 2)));
    double range = std::sqrt(std::pow(position[0], 2) + std::pow(position[1],
                2) + std::pow(position[2], 2));

    double x = std::cos(azimuth) * range;
    double y = std::sin(azimuth) * range;
    double z = std::sin(inclination) * range;

    ROS_INFO_STREAM("azimuth: " << azimuth);
    ROS_INFO_STREAM("inclination: " << inclination);
    ROS_INFO_STREAM("range: " << range);
    ROS_INFO_STREAM("azimuth(deg): " << azimuth * RAD_TO_DEG);
    ROS_INFO_STREAM("inclination(deg): " << inclination * RAD_TO_DEG);
    ROS_INFO_STREAM("range: " << range);
    ROS_INFO_STREAM("x: " << x);
    ROS_INFO_STREAM("y: " << y);
    ROS_INFO_STREAM("z: " << z);
    ROS_INFO_STREAM("");

    observation(0, 0) = azimuth;
    observation(1, 0) = inclination;
    observation(2, 0) = range;

    Update();
}

void ParticleFilter::InputAbsLinVel(const tf::Vector3 lin_vel, const double dt)
{
    control_input(0, 0) = lin_vel[0];
    control_input(1, 0) = lin_vel[1];
    control_input(2, 0) = lin_vel[2];

    control_update_model(0, 0) = dt;
    control_update_model(1, 1) = dt;
    control_update_model(2, 2) = dt;

    //PF_PRINT_THROTTLE(ROS_DEBUG_STREAM("control_input:\n" << control_input););
    //PF_PRINT_THROTTLE(ROS_DEBUG_STREAM("control_update_model:\n" << control_update_model););

    Predict();
}

Matrix<double, 4, 1> ParticleFilter::state_to_observation(Matrix<double, 3, 1> state)
{
    Matrix<double, 4, 1> obs;

    double hz = -pinger_depth + state(2, 0);

    double azimuth = std::atan2(state(1, 0), state(0, 0));
    double inclination = std::atan2(hz,
            std::sqrt(std::pow(state(0, 0), 2) + std::pow(state(1, 0), 2)));
    double range = std::sqrt(std::pow(state(0, 0), 2) + std::pow(state(1, 0),
                2) + std::pow(hz, 2));

    // Observation in (azimuth, inclination, range)
    obs(0, 0) = azimuth;
    obs(1, 0) = inclination;
    obs(2, 0) = range;

    // Depth observation
    obs(3, 0) = state(2, 0);

    return obs;
}

Matrix<double, 4, 1> ParticleFilter::add_observation_noise(
    Matrix<double, 4, 1> obs)
{
    return obs +
           (sqrt_elementwise(measurement_covar) * randn_mat(obs.rows(), 1));
}

// Update each particle_states state based on the update model plus noise
void ParticleFilter::update_particle_states()
{
    for(int n = 0; n < num_particles; n++)
    {
        particle_states[n] = system_update_model * last_particle_states[n] +
            control_update_model * control_input +
            sqrt_elementwise(system_update_covar) * randn_mat(3, 1);

        //particle_states[n](0, 0) = fmod(particle_states[n](0, 0), PI);
        //particle_states[n](1, 0) = fmod(particle_states[n](1, 0), PI);
    }
}

void ParticleFilter::update_particle_weights()
{
    double particle_weights_sum = 0.0;
    for(int n = 0; n < num_particles; n++)
    {
        // Update each particle_states observation based on its state
        particle_obs[n] = state_to_observation(particle_states[n]);
        // add noise to observations
        particle_obs[n] = add_observation_noise(particle_obs[n]);

        // Calculate particle weights
        particle_weights[n] = 1.0;
        for(unsigned int i = 0; i < observation.rows(); i++)
        {
            // TODO: Update selectively based on whether sensor readings are
            // current or not?

            double p = gaussian_prob(observation(i, 0),
                                     std::sqrt(measurement_covar(i, i)),
                                     particle_obs[n](i, 0));

            particle_weights[n] *= p;
        }

        // Need to get sum for normalization
        particle_weights_sum += particle_weights[n];
    }
    last_particle_obs = particle_obs;

    // Normalize weights so that they sum to 1
    for(int n = 0; n < num_particles; n++)
    {
        if(particle_weights_sum == 0.0)
        {
            // TODO: Reinit here
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
    // Resample particles
    std::vector<Matrix<double, 3, 1> > p;
    int index = static_cast<int>(randu() * num_particles);
    double beta = 0.0;
    double max_w = vector_max(particle_weights);
    for(int i = 0; i < num_particles; i++)
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
    for(int i = 0; i < num_particles; i++)
    {
        est_state += particle_states[i];
    }

    for(int i = 0; i < est_state.rows(); i++)
    {
        if(std::isnan(est_state(i, 0)))
        {
            est_state(i, 0) = 0.0;
        }
    }

    est_state(0, 0) /= num_particles;
    est_state(1, 0) /= num_particles;
    est_state(2, 0) /= num_particles;

    // Get Position from (azimuth, inclination, range)
    //double x = std::cos(est_state(0, 0)) * est_state(2, 0);
    //double y = std::sin(est_state(0, 0)) * est_state(2, 0);
    //double z = pinger_depth + (std::sin(est_state(1, 0)) * est_state(2, 0));
    estimated_position[0] = est_state(0, 0);
    estimated_position[1] = est_state(1, 0);
    estimated_position[2] = est_state(2, 0);
}

void ParticleFilter::Predict()
{
    PF_PRINT_THROTTLE(ROS_DEBUG_STREAM("PREDICT"););

    update_particle_states();

    last_particle_states = particle_states;

    estimate_state();

    PF_PRINT_THROTTLE(ROS_DEBUG_STREAM("est_state: " << std::endl << est_state););
    num_iterations++;
}

// TODO: Add sanity check to estimated state and reinitialize filter if
// needed (Possibly just check if est_state is roughly within bounds of
// pool)
void ParticleFilter::Update()
{
    PF_PRINT_THROTTLE(ROS_DEBUG_STREAM("UPDATE"););

    reload_params();

    update_particle_states();

    update_particle_weights();

    resample_particles();

    estimate_state();

    PF_PRINT_THROTTLE(ROS_DEBUG_STREAM("est_state: " << std::endl << est_state););
    num_iterations++;

}
