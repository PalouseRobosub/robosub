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

    ReloadParams();
    InitializeParticleFilter();
}

LocalizationSystem::~LocalizationSystem()
{
    delete norm_distribution;
    delete uniform_distribution;
}

void LocalizationSystem::ReloadParams()
{
    if(!ros::param::getCached("hydrophones/pinger/depth", pinger_depth))
    {
        ROS_ERROR("pinger depth failed to load");
        ros::shutdown();
    }
}

geometry_msgs::Vector3Stamped LocalizationSystem::GetLocalizationMessage()
{
    geometry_msgs::Vector3Stamped s;
    s.vector.x = est_state[0];
    s.vector.y = est_state[1];
    s.vector.z = est_state[2];
    s.header.stamp = ros::Time::now();

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

    est_state.setZero();
    last_est_state.setZero();

    initial_state(0,0) = -30.0;
    initial_state(1,0) = -20.0;
    //initial_state(2,0) = 0.0;
    //initial_state(3,0) = 0.0;
    //initial_state(4,0) = 0.0;
    //initial_state(5,0) = 0.0;

    // TODO: Tune
    // measurement covariance matrix along diagnol
    // measurement_covar is covar of [hydrophones_position, lin_accel, depth] =
    // [hx, hy, hz, lx, ly, lz, d]
    measurement_covar(0,0) = 4.0;
    measurement_covar(1,1) = 4.0;
    measurement_covar(2,2) = 4.0;
    measurement_covar(3,3) = 2.0;
    measurement_covar(4,4) = 2.0;
    measurement_covar(5,5) = 2.0;
    measurement_covar(6,6) = 0.5;

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
    //system_update_model(0,3) = dt;
    //system_update_model(1,4) = dt;
    //system_update_model(2,5) = dt;

    // Intial std_devs of position around pinger
    initial_distribution.setZero();
    initial_distribution(0,0) = 5.0;
    initial_distribution(1,0) = 5.0;
    initial_distribution(2,0) = -2.0;
    initial_distribution(3,0) = 0.1;
    initial_distribution(4,0) = 0.1;
    initial_distribution(5,0) = 0.1;

    // TODO: Tune
    // system_update_covariance =
    system_update_covar = system_update_covar.Identity() * 0.4;

    for(int n=0; n<num_particles; n++)
    {
        // TODO: Randomly distribute initial particle_states
        Matrix<double,6,1> s;
        for(int j=0; j<initial_state.rows(); j++)
        {
            s(j,0) = initial_state(j,0) + randn() * initial_distribution(j,0);
            particle_states.push_back(s);
        }
        last_particle_states = particle_states;

        //ROS_INFO_STREAM("initial_particle_states: " << particle_states);

        // Zeroed at this point
        particle_obs.push_back(observation);
        last_particle_obs.push_back(observation);
        last_particle_weights.push_back(0);
        particle_weights.push_back(0);
    }

    last_lin_accel_receive_time = ros::Time::now();

    ROS_INFO_STREAM("Finished PF init");
}

//*********************//
// Observation updates //
//*********************//

// observation = [hydrophone_x, hydrophone_y, hydrophone_z, lin_accel_x,
// lin_accel_y, lin_accel_z, depth]

void LocalizationSystem::depthCallback(const robosub::depth_stamped::ConstPtr &msg)
{
    observation(6,0) = msg->depth;
    new_depth = true;
}

void LocalizationSystem::hydrophoneCallback(const robosub::PositionArrayStamped::ConstPtr &msg)
{
    // pinger_depth = -4.9
    // depth is pinger_depth + hz
    // state_z is depth?
    observation(0,0) = msg->positions[0].position.x;
    observation(1,0) = msg->positions[0].position.y;
    observation(2,0) = msg->positions[0].position.z;
    new_hydrophone = true;
}

void LocalizationSystem::linAccelCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
    // This is the glorified dead reckoning part...
    ros::Duration dt = ros::Time::now() - last_lin_accel_receive_time;
    lin_velocity.x += msg->vector.x * dt.toSec();
    lin_velocity.y += msg->vector.y * dt.toSec();
    lin_velocity.z += msg->vector.z * dt.toSec();
    // TEMP
    //system_update_model(0,3) = dt.toSec();
    //system_update_model(1,4) = dt.toSec();
    //system_update_model(2,5) = dt.toSec();

    //observation(3,0) = lin_velocity.x;
    //observation(4,0) = lin_velocity.y;
    //observation(5,0) = lin_velocity.z;

    observation(3,0) = 0.0;
    observation(4,0) = 0.0;
    observation(5,0) = 0.0;

    //ROS_INFO_STREAM("lin_velocity: " << lin_velocity);

    new_lin_velocity = true;
    last_lin_accel_receive_time = ros::Time::now();
}

bool LocalizationSystem::resetFilterCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &rep)
{
    return true;
}

Matrix<double,7,1> LocalizationSystem::state_to_observation(Matrix<double,6,1> state)
{
    Matrix<double,7,1> obs;

    // state = [x, y, z, x_vel, y_vel, z_vel]
    // observation = [hx, hy, hz, lx, ly, lz, depth]

    // Position
    // TODO: Need to define coordinates since x,y,z is global frame, hx,hy,hz
    // is relative to pinger
    obs(0,0) = state(0,0);
    obs(1,0) = state(1,0);
    obs(2,0) = state(2,0) - pinger_depth;

    // Lin Accel
    //obs(3,0) = state(3,0);
    //obs(4,0) = state(4,0);
    //obs(5,0) = state(5,0);
    obs(3,0) = 0.0;
    obs(4,0) = 0.0;
    obs(5,0) = 0.0;

    obs(6,0) = state(2,0);

    return obs;
}

Matrix<double,7,1> LocalizationSystem::add_observation_noise(Matrix<double,7,1> obs)
{
    return obs + (sqrt_elementwise(measurement_covar) * randn_mat(obs.rows(), 1));
}

void LocalizationSystem::Update()
{
    // Getto ros message filter
    // TODO: This probably doesn't work anymore. Possibly run iteration on any
    // new sensor reading, keeping old readings same.
    ReloadParams();
    new_lin_velocity = true;
    if(new_hydrophone && new_depth && new_lin_velocity)
    {
        PRINT_THROTTLE(ROS_INFO_STREAM("==================================");)
        PRINT_THROTTLE(ROS_INFO_STREAM("num_iterations: " << num_iterations);)
        double particle_weights_sum = 0.0;
        for(int n=0; n<num_particles; n++)
        {
            // Update each particle_states state based on the update model plus noise
            particle_states[n] = system_update_model * last_particle_states[n] + sqrt_elementwise(system_update_covar) * randn_mat(6,1);
            //PRINT_THROTTLE(ROS_INFO_STREAM("particle_states[" << n << "]: " << particle_states[n]);)

            // Update each particle_states observation based on its state
            particle_obs[n] = state_to_observation(particle_states[n]);
            //PRINT_THROTTLE(ROS_INFO_STREAM("particle_obs[" << n << "]: " << particle_obs[n]);)
            // add noise to observations
            particle_obs[n] = add_observation_noise(particle_obs[n]);
            //PRINT_THROTTLE(ROS_INFO_STREAM("particle_obs + obs_noise[" << n << "]: " << particle_obs[n]);)

            // This is dense but it is the multivariate normal distribution function found here:
            // https://en.wikipedia.org/wiki/Multivariate_normal_distribution
            // https://wikimedia.org/api/rest_v1/media/math/render/svg/077fb81ca53daa87ecdce68c70df278f46e77298
            //Matrix<double,7,1> diff;
            //diff = particle_obs[n] - observation;
            //particle_weights[n] = std::exp( -0.5 * diff.transpose() *
                    //measurement_covar.inverse() * diff ) / std::sqrt(
                    //(2.0*3.14*measurement_covar).determinant() );

            // Method 2
            particle_weights[n] = 1.0;
            for(unsigned int i=0; i<observation.rows(); i++)
            {
                // For now dont consider lin_velocity
                if(i == 3 || i == 4 || i == 5)
                {
                    continue;
                }
                // double gaussian_prob(double mean, double sigma, double x)
                particle_weights[n] *= gaussian_prob(observation(i, 0), std::sqrt(measurement_covar(i, i)), particle_obs[n](i, 0));
            }

            //PRINT_THROTTLE(ROS_INFO_STREAM("particle_obs[n]: " << particle_obs[n]);)
            //PRINT_THROTTLE(ROS_INFO_STREAM("observation: " << observation);)
            //PRINT_THROTTLE(ROS_INFO_STREAM("diff: " << diff);)
            PRINT_THROTTLE(ROS_INFO_STREAM("particle_weights[" << n << "]: " << particle_weights[n]);)

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

        /*
        last_particle_states = particle_states;
        last_particle_weights = particle_weights;
        // Resample particles
        //
        // TODO: Explain algorithm
        PRINT_THROTTLE(ROS_INFO_STREAM("particle_weights: " << particle_weights);)
        std::vector<double> weights_cumsum = CumSum(particle_weights);
        PRINT_THROTTLE(ROS_INFO_STREAM("weights_cumsum: " <<  weights_cumsum);)
        double threshold;
        for(int i=0; i<num_particles; i++)
        {
            threshold = randu();
            for(int j=0; j<num_particles; j++)
            {
                if(threshold <= weights_cumsum[j])
                {
                    particle_states[i] = last_particle_states[j];
                    break;
                }
            }
        }
        */

        std::vector<Matrix<double, 6,1> > p;
        int index = int(randu() * num_particles);
        double beta = 0.0;
        double max_w = vector_max(particle_weights);
        // Resample particles
        // method 2
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

        // Estimate state from all particle_states
        // Just weighted averaging for now
        est_state.setZero();
        Matrix<double,6,1> state_sum;
        for(int i=0; i<num_particles; i++)
        {
            est_state += particle_states[i] * particle_weights[i];
        }

        PRINT_THROTTLE(ROS_INFO_STREAM("est_state: " << std::endl << est_state);)

        new_hydrophone = new_depth = new_lin_velocity = false;
        num_iterations++;
    }
}
