#include "localization_system.hpp"

LocalizationSystem::LocalizationSystem(double _dt, int _num_particles)
{
    dt = _dt;
    num_particles = _num_particles;

    new_hydrophone = new_depth = false;

    norm_distribution = new std::normal_distribution<double>(0.0, 1.0);
    uniform_distribution = new std::uniform_real_distribution<double>(0.0, 1.0);

    InitializeParticleFilter();
}

LocalizationSystem::~LocalizationSystem()
{
    delete norm_distribution;
    delete uniform_distribution;
}

void LocalizationSystem::InitializeParticleFilter()
{
    system_update_model.setZero();
    system_update_covar.setZero();
    initial_state.setZero();
    observation.setZero();

    // state initalized to zero
    est_state.setZero();
    last_est_state.setZero();

    initial_state(0,0) = 10.0;
    initial_state(1,0) = 10.0;
    initial_state(2,0) = -2.0;
    initial_state(3,0) = 0.0;
    initial_state(4,0) = 0.0;
    initial_state(5,0) = 0.0;

    // TODO: Tune
    // measurement covariance matrix along diagnol
    // variance of each [x_acl, y_acl, z_acl, depth] on diagnal
    measurement_covar = measurement_covar.Identity() * 0.2;

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

    // TODO: Tune
    // system_update_covariance =
    system_update_covar = system_update_covar.Identity() * 0.2;

    for(int i=0; i<num_particles; i++)
    {
        // TODO: Randomly distribute initial particle_states
        Matrix<double,6,1> s;
        for(int j=0; j<est_state.rows(); j++)
        {
            s(j,0) = initial_state(j,0) + randn() * std::sqrt(system_update_covar(j,j));
            particle_states.push_back(s);
        }
        last_particle_states = particle_states;

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

bool LocalizationSystem::resetFilterCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &rep)
{
    return true;
}

Matrix<double,4,1> LocalizationSystem::StateToObservation(Matrix<double,6,1> state)
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

Matrix<double,4,1> LocalizationSystem::AddObservationNoise(Matrix<double,4,1> particle_obs)
{
    return particle_obs + (sqrt_elementwise(measurement_covar) * randn_mat(particle_obs.rows(), 1));
}

void LocalizationSystem::Update()
{
    // Getto ros message filter
    // Should work for now since bno sends its data up all at one time
    if(new_hydrophone && new_depth)
    {
        ROS_INFO_STREAM_THROTTLE(2.0, "=========================================");
        double particle_weights_sum = 0.0;
        for(int i=0; i<num_particles; i++)
        {
            // Update each particle_states state based on the update model plus noise
            particle_states[i] = system_update_model * last_particle_states[i] + system_update_covar * randn_mat(6,1);
            ROS_INFO_STREAM_THROTTLE(2.0, "particle_states[" << i << "]: " << particle_states[i]);

            // Update each particle_states observation based on its state
            particle_obs[i] = StateToObservation(particle_states[i]);
            ROS_INFO_STREAM_THROTTLE(2.0, "particle_obs[" << i << "]: " << particle_obs[i]);
            // add noise to observations
            particle_obs[i] = AddObservationNoise(particle_obs[i]);
            ROS_INFO_STREAM_THROTTLE(2.0, "particle_obs + obs_noise[" << i << "]: " << particle_obs[i]);

            // This is dense but it is the multivariate normal distribution function found here:
            // https://en.wikipedia.org/wiki/Multivariate_normal_distribution
            // https://wikimedia.org/api/rest_v1/media/math/render/svg/077fb81ca53daa87ecdce68c70df278f46e77298
            Matrix<double,4,1> diff;
            diff = particle_obs[i] - observation;
            particle_weights[i] = std::exp( -0.5 * diff.transpose() *
                    measurement_covar.inverse() * diff ) / std::sqrt(
                    (2.0*3.14*measurement_covar).determinant() );

            //ROS_INFO_STREAM_THROTTLE(1.0, "particle_obs[i]: " << particle_obs[i]);
            //ROS_INFO_STREAM_THROTTLE(1.0, "observation: " << observation);
            //ROS_INFO_STREAM_THROTTLE(1.0, "diff: " << diff);
            ROS_INFO_STREAM_THROTTLE(2.0, "particle_weights[" << i << "]: " << particle_weights[i]);

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

        //ROS_INFO_STREAM_THROTTLE(1.0, "particle_weights: " << particle_weights);
        last_particle_states = particle_states;
        last_particle_weights = particle_weights;
        // Resample particles
        //
        // TODO: Explain algorithm
        //ROS_INFO_STREAM_THROTTLE(1.0, "particle_weights: " << particle_weights);
        std::vector<double> weights_cumsum = CumSum(particle_weights);
        //ROS_INFO_STREAM_THROTTLE(1.0, "weights_cumsum: " <<  weights_cumsum);
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

        // Print everything
        for(int i=0; i<num_particles; i++)
        {
            //ROS_INFO_STREAM_THROTTLE(1.0, "particle [" << i << "]: ");
            //ROS_INFO_STREAM_THROTTLE(1.0, "STATE" << std::endl << particle_states[i]);
            //ROS_INFO_STREAM_THROTTLE(1.0, "WEIGHT" << std::endl << particle_weights[i]);
            //ROS_INFO_STREAM_THROTTLE(1.0, "OBSERVATION: " << std::endl << particle_obs[i] << std::endl);
        }
        ROS_INFO_STREAM_THROTTLE(2.0, "particle_weights" << std::endl << particle_weights);

        ROS_INFO_STREAM("est_state: " << std::endl << est_state);

        new_hydrophone = new_depth = false;
    }
}
