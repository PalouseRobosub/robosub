#include "localization_system.hpp"

LocalizationSystem::LocalizationSystem(double _dt, int _num_particles)
{
    dt = _dt;
    num_particles = _num_particles;

    new_hydrophone = new_depth = new_lin_velocity = false;

    norm_distribution = new std::normal_distribution<double>(0.0, 1.0);
    uniform_distribution = new std::uniform_real_distribution<double>(0.0, 1.0);

    //std::chrono::high_resolution_clock::time_point t = std::chrono::high_resolution_clock::now();
    //rand_generator.seed(static_cast<int>(t.time_since_epoch()));

    InitializeParticleFilter();
}

LocalizationSystem::~LocalizationSystem()
{
    delete norm_distribution;
    delete uniform_distribution;
}

geometry_msgs::Vector3 LocalizationSystem::GetLocalizationMessage()
{
    geometry_msgs::Vector3 s;
    s.x = est_state[0];
    s.y = est_state[1];
    s.z = est_state[2];

    return s;
}

void LocalizationSystem::InitializeParticleFilter()
{
    num_iterations = 0;

    lin_velocity.x = 0.0;
    lin_velocity.y = 0.0;
    lin_velocity.z = 0.0;

    system_update_model.setZero();
    system_update_covar.setZero();
    initial_state.setZero();
    observation.setZero();

    // state initalized to zero
    est_state.setZero();
    last_est_state.setZero();

    // TODO: Tune
    // measurement covariance matrix along diagnol
    // variance of each [x_acl, y_acl, z_acl, depth] on diagnal
    measurement_covar = measurement_covar.Identity() * 3.0;

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
    // TEMPORARY until we integrate linear velocity
    //system_update_model(0,3) = dt;
    //system_update_model(1,4) = dt;
    //system_update_model(2,5) = dt;

    // Intial std_devs of position around pinger
    initial_distribution.setZero();
    initial_distribution(0,0) = 5.0;
    initial_distribution(1,0) = 5.0;
    initial_distribution(2,0) = -2.0;

    // TODO: Tune
    // system_update_covariance =
    system_update_covar = system_update_covar.Identity() * 0.2;

    for(int i=0; i<num_particles; i++)
    {
        // TODO: Randomly distribute initial particle_states
        Matrix<double,6,1> s;
        for(int j=0; j<initial_state.rows(); j++)
        {
            s(j,0) = initial_state(j,0) + randn() * initial_distribution(j,0);
            particle_states.push_back(s);
        }
        last_particle_states = particle_states;

        ROS_INFO_STREAM("initial_particle_states: " << particle_states);

        // Zeroed at this point
        particle_obs.push_back(observation);
        last_particle_obs.push_back(observation);
        last_particle_weights.push_back(0);
        particle_weights.push_back(0);
    }

    ROS_INFO_STREAM("Finished PF init");
}

//*********************//
// Observation updates //
//*********************//

// observation = [hydrophone_x, hydrophone_y, hydrophone_z, depth]

void LocalizationSystem::depthCallback(const robosub::depth_stamped::ConstPtr &msg)
{
    observation(3,0) = msg->depth;
    new_depth = true;
}

void LocalizationSystem::hydrophoneCallback(const robosub::PositionsStamped::ConstPtr &msg)
{
    observation(0,0) = msg->positions[0].x;
    observation(1,0) = msg->positions[0].y;
    observation(2,0) = msg->positions[0].z;
    new_hydrophone = true;
}

void LocalizationSystem::linAccelCallback(const geometry_msgs::Vector3::ConstPtr &msg)
{
    // This is the glorified dead reckoning part...
    lin_velocity.x += msg->x;
    lin_velocity.y += msg->y;
    lin_velocity.z += msg->z;

    //observation(3,0) = lin_velocity.x;
    //observation(4,0) = lin_velocity.y;
    //observation(5,0) = lin_velocity.z;

    new_lin_velocity = true;
}

bool LocalizationSystem::resetFilterCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &rep)
{
    return true;
}

Matrix<double,4,1> LocalizationSystem::state_to_observation(Matrix<double,6,1> state)
{
    Matrix<double,4,1> obs;

    // state = [x, y, z, x_vel, y_vel, z_vel]
    // observation = [hx, hy, hz, depth]

    obs(0,0) = state(0,0);
    obs(1,0) = state(1,0);
    obs(2,0) = state(2,0);
    obs(3,0) = state(2,0);

    return obs;
}

Matrix<double,4,1> LocalizationSystem::add_observation_noise(Matrix<double,4,1> particle_obs)
{
    return particle_obs + (sqrt_elementwise(measurement_covar) * randn_mat(particle_obs.rows(), 1));
}

void LocalizationSystem::Update()
{
    // Getto ros message filter
    // TODO: This probably doesn't work anymore. Possibly run iteration on any
    // new sensor reading, keeping old readings same.
    if(new_hydrophone && new_depth && new_lin_velocity)
    {
        PRINT_THROTTLE(ROS_INFO_STREAM("==================================");)
        PRINT_THROTTLE(ROS_INFO_STREAM("num_iterations: " << num_iterations);)
        double particle_weights_sum = 0.0;
        for(int i=0; i<num_particles; i++)
        {
            // Update each particle_states state based on the update model plus noise
            particle_states[i] = system_update_model * last_particle_states[i] + system_update_covar * randn_mat(6,1);
            //PRINT_THROTTLE(ROS_INFO_STREAM("particle_states[" << i << "]: " << particle_states[i]);)

            // Update each particle_states observation based on its state
            particle_obs[i] = state_to_observation(particle_states[i]);
            //PRINT_THROTTLE(ROS_INFO_STREAM("particle_obs[" << i << "]: " << particle_obs[i]);)
            // add noise to observations
            particle_obs[i] = add_observation_noise(particle_obs[i]);
            //PRINT_THROTTLE(ROS_INFO_STREAM("particle_obs + obs_noise[" << i << "]: " << particle_obs[i]);)

            // This is dense but it is the multivariate normal distribution function found here:
            // https://en.wikipedia.org/wiki/Multivariate_normal_distribution
            // https://wikimedia.org/api/rest_v1/media/math/render/svg/077fb81ca53daa87ecdce68c70df278f46e77298
            Matrix<double,4,1> diff;
            diff = particle_obs[i] - observation;
            particle_weights[i] = std::exp( -0.5 * diff.transpose() *
                    measurement_covar.inverse() * diff ) / std::sqrt(
                    (2.0*3.14*measurement_covar).determinant() );

            //PRINT_THROTTLE(ROS_INFO_STREAM("particle_obs[i]: " << particle_obs[i]);)
            //PRINT_THROTTLE(ROS_INFO_STREAM("observation: " << observation);)
            //PRINT_THROTTLE(ROS_INFO_STREAM("diff: " << diff);)
            //PRINT_THROTTLE(ROS_INFO_STREAM("particle_weights[" << i << "]: " << particle_weights[i]);)

            // Need to get sum for normalization
            particle_weights_sum += particle_weights[i];
        }
        last_particle_obs = particle_obs;

        // Normalize weights so that they sum to 1
        for(int i=0; i<num_particles; i++)
        {
            if(particle_weights_sum == 0.0)
            {
                particle_weights[i] = 1.0/(float)num_particles;
            }
            else
            {
                particle_weights[i] /= particle_weights_sum;
            }
        }

        last_particle_states = particle_states;
        last_particle_weights = particle_weights;
        // Resample particles
        //
        // TODO: Explain algorithm
        PRINT_THROTTLE(ROS_INFO_STREAM("particle_weights: " << particle_weights);)
        std::vector<double> weights_cumsum = CumSum(particle_weights);
        PRINT_THROTTLE(ROS_INFO_STREAM("weights_cumsum: " <<  weights_cumsum);)
        int i, j;
        double threshold;
        for(i=0; i<num_particles; i++)
        {
            threshold = randu();
            for(j=0; j<num_particles; j++)
            {
                if(threshold <= weights_cumsum[j])
                {
                    break;
                }
            }
            particle_states[i] = last_particle_states[j];
        }

        // Estimate state from all particle_states
        // Just weighted averaging for now
        est_state.setZero();
        Matrix<double,6,1> state_sum;
        for(int i=0; i<num_particles; i++)
        {
            est_state += particle_states[i] * particle_weights[i];
        }
        est_state[3] = est_state[4] = est_state[5] = 0.0;

        PRINT_THROTTLE(ROS_INFO_STREAM("est_state: " << std::endl << est_state);)

        new_hydrophone = new_depth = new_lin_velocity = false;
        num_iterations++;
    }
}
