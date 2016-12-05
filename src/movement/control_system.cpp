#include "control_system.hpp"

/**
 * Constructor.
 *
 * @param _nh A node handle reference to use for accessing node parameters.
 * @param _pub A pointer to a ROS publisher to publish thruster messages to.
 */
ControlSystem::ControlSystem(ros::Publisher *_pub) :
    nh(ros::NodeHandle("control")),
    pub(_pub)
{
    ReloadPIDParams();

    /*
     * Load parameters from the settings file.
     */
    nh.getParamCached("mass", sub_mass[0]);
    nh.getParamCached("mass", sub_mass[1]);
    nh.getParamCached("mass", sub_mass[2]);
    nh.getParamCached("inertia/psi", sub_mass[3]);
    nh.getParamCached("inertia/phi", sub_mass[4]);
    nh.getParamCached("inertia/theta", sub_mass[5]);
    nh.getParamCached("back_thrust_ratio", back_thrust_ratio);
    nh.getParamCached("limits/translation", t_lim);
    nh.getParamCached("limits/rotation", r_lim);
    nh.getParamCached("max_thrust", max_thrust);
    int rate = 1;
    nh.getParamCached("rate", rate);

    /*
     * Calculate the change in time between each call.
     * TODO: Replace dt with actual message timestamps.
     */
    dt = 1.0/rate;

    /*
     * Initialize the state of the goals to error.
     */
    for(int i=0; i < 6; ++i)
    {
        goal_types[i] = robosub::control::STATE_ERROR;
    }

    /*
     * Ensure that the commands and states are all set to zero initially.
     */
    state_vector = Vector12d::Zero();
    integral_state = Vector6d::Zero();

    /*
     * Scale thruster limits to be in terms of the backwards thruster ratio.
     */
    t_lim *= back_thrust_ratio;
    r_lim *= back_thrust_ratio;

    /*
     * Load thruster node settings. A nodehandle reference is not available, so
     * make use of rosparam get.
     */
    XmlRpc::XmlRpcValue thruster_settings;
    if(!ros::param::get("thrusters", thruster_settings))
    {
        ROS_FATAL("thruster params failed to load");
        exit(1);
    }
    ROS_INFO_STREAM("Loaded " << thruster_settings.size() << " thrusters");

    position = MatrixXd(1,3);
    orientation = MatrixXd(1,3);
    for(int i = 0; i < thruster_settings.size(); ++i, ++num_thrusters)
    {
        /*
         * Increment the number of rows in the position and orientation
         * matrices to accomodate the new thruster.
         */
        position.conservativeResize(i+1, NoChange_t());
        orientation.conservativeResize(i+1, NoChange_t());

        /*
         * Load in the thruster positions and orientations. The column of the
         * position and orientation matrices denote the x, y, and z components
         * sequentially.
         */
        position(i,0) = thruster_settings[i]["position"]["x"];
        position(i,1) = thruster_settings[i]["position"]["y"];
        position(i,2) = thruster_settings[i]["position"]["z"];
        orientation(i,0) = thruster_settings[i]["orientation"]["x"];
        orientation(i,1) = thruster_settings[i]["orientation"]["y"];
        orientation(i,2) = thruster_settings[i]["orientation"]["z"];
    }

    motors = MatrixXd(6,num_thrusters);
    motor_commands = VectorXd::Zero(num_thrusters);
    for (int i = 0; i < num_thrusters; ++i)
    {
        /*
         * Calculate the force and moment for each thruster.
         * TODO: Validate moment calculations are correct through utilization
         * of the cross product.
         */
        motors.block<3,1>(0,i) = max_thrust *
            orientation.block<1,3>(i,0).transpose();
        motors.block<3,1>(3,i) = max_thrust * position.block<1,3>(i,0).cross(
                orientation.block<1,3>(i,0)).transpose();
    }

    ROS_INFO_STREAM("motor_matrix:\n" << motors);
    ROS_INFO_STREAM("orientation:\n" << orientation);
    ROS_INFO_STREAM("position:\n" << position);

    /*
     * TODO: Discover why the motor matrix is inverted.
     */
    if(motors.rows() == motors.cols())
    {
        motors = motors.inverse();
    }
    else
    {
        motors = pinv(motors);
    }
}

/**
 * Reload PID parameters from ROS parameter lists.
 *
 * @return None.
 */
void ControlSystem::ReloadPIDParams()
{
    nh.getParamCached("proportional/x", P[0]);
    nh.getParamCached("proportional/y", P[1]);
    nh.getParamCached("proportional/z", P[2]);
    nh.getParamCached("proportional/psi", P[3]);
    nh.getParamCached("proportional/phi", P[4]);
    nh.getParamCached("proportional/theta", P[5]);
    nh.getParamCached("integral/x", I[0]);
    nh.getParamCached("integral/y", I[1]);
    nh.getParamCached("integral/z", I[2]);
    nh.getParamCached("integral/psi", I[3]);
    nh.getParamCached("integral/phi", I[4]);
    nh.getParamCached("integral/theta", I[5]);
    nh.getParamCached("derivative/x", D[0]);
    nh.getParamCached("derivative/y", D[1]);
    nh.getParamCached("derivative/z", D[2]);
    nh.getParamCached("derivative/psi", D[3]);
    nh.getParamCached("derivative/phi", D[4]);
    nh.getParamCached("derivative/theta", D[5]);
    nh.getParamCached("windup/x", windup[0]);
    nh.getParamCached("windup/y", windup[1]);
    nh.getParamCached("windup/z", windup[2]);
    nh.getParamCached("windup/psi", windup[3]);
    nh.getParamCached("windup/phi", windup[4]);
    nh.getParamCached("windup/theta", windup[5]);
    nh.getParamCached("hysteresis/x", hysteresis[0]);
    nh.getParamCached("hysteresis/y", hysteresis[1]);
    nh.getParamCached("hysteresis/z", hysteresis[2]);
    nh.getParamCached("hysteresis/psi", hysteresis[3]);
    nh.getParamCached("hysteresis/phi", hysteresis[4]);
    nh.getParamCached("hysteresis/theta", hysteresis[5]);
}

/**
 * Update the current goals of the submarine.
 *
 * @note This function updates he desired state Sd.
 *
 * @param msg The control message used to update the desired state.
 *
 * @return None.
 */
void ControlSystem::InputControlMessage(robosub::control msg)
{
    Vector6d control_values;
    Matrix<uint8_t, 6, 1> control_states;

    control_states[0] = msg.forward_state;
    control_states[1] = msg.strafe_state;
    control_states[2] = msg.dive_state;
    control_states[3] = msg.roll_state;
    control_states[4] = msg.pitch_state;
    control_states[5] = msg.yaw_state;

    control_values[0] = msg.forward;
    control_values[1] = msg.strafe_left;
    control_values[2] = msg.dive;
    control_values[3] = msg.roll_right * _PI_OVER_180;
    control_values[4] = msg.pitch_down * _PI_OVER_180;
    control_values[5] = msg.yaw_left * _PI_OVER_180;

    for(int i=0; i < 6; ++i)
    {
        switch(control_states[i])
        {
            case robosub::control::STATE_ABSOLUTE:
                this->goal_types[i] = robosub::control::STATE_ABSOLUTE;
                this->goals[i] = control_values[i];
                break;

            /*
             * Note that all rotational goals must be wrapped according to the
             * following bounding limits:
             *   Roll: (-180, 180)
             *   Pitch: (-90, 90)
             *   Yaw: (-180, 180)
             */
            case robosub::control::STATE_RELATIVE:
                this->goal_types[i] = robosub::control::STATE_ABSOLUTE;
                if(i < 3)
                    goals[i] = state_vector[i] + control_values[i];
                else if (i == 3)
                    goals[i] = wraparound(state_vector[i+3] + control_values[i], -180.0 * _PI_OVER_180, 180.0 * _PI_OVER_180);
                else if (i == 4)
                    goals[i] = wraparound(state_vector[i+3] + control_values[i], -90.0 * _PI_OVER_180, 90.0 * _PI_OVER_180);
                else if (i == 5)
                    goals[i] = wraparound(state_vector[i+3] + control_values[i], -180.0 * _PI_OVER_180, 180.0 * _PI_OVER_180);
                break;

            case robosub::control::STATE_ERROR:
                this->goal_types[i] = robosub::control::STATE_ERROR;
                this->goals[i] = control_values[i];
                break;

            case robosub::control::STATE_NONE:
                break;

            default:
                ROS_ERROR("received invalid control state");
                break;
        }
    }
}

/**
 * Update the current state of the submarine.
 *
 * @brief Updates the current state vector S of the submarine given an input
 *        orientation message.
 *
 * @param quat_msg The input ROS Quaternion message that defines orientation.
 *
 * @return None.
 */
void ControlSystem::InputOrientationMessage(geometry_msgs::QuaternionStamped quat_msg)
{
    /*
     * Convert the Quaternion to roll, pitch, and yaw and store the result into
     * the state vector.
     */
    tf::Matrix3x3 m(tf::Quaternion(quat_msg.quaternion.x,
                quat_msg.quaternion.y, quat_msg.quaternion.z,
                quat_msg.quaternion.w));
    m.getRPY(state_vector[6], state_vector[7], state_vector[8]);
    /*
    state_vector[9] = _PI_OVER_180 * sp.droll; //roll'
    state_vector[10] = _PI_OVER_180 * sp.dpitch; //pitch'
    state_vector[11] = _PI_OVER_180 * sp.dyaw; //yaw'
    */
}

/**
 * Update the depth of the submarine.
 *
 * @brief Updates the current state vector S with an updated depth reading.
 *
 * @return None.
 */
void ControlSystem::InputDepthMessage(robosub::depth_stamped depth_msg)
{
    /*
     * Update the X, Y, and Z positions of the state vector. Note that X and Y
     * position are currently unknown and always set to zero.
     */
    state_vector[0] = 0; //x
    state_vector[1] = 0; //y
    state_vector[2] = depth_msg.depth; //z

    /*
     * Update the derivatives of position within the state vector.
     */
    state_vector[3] = 0;
    state_vector[4] = 0;
    state_vector[5] = prev_depth_msg.depth - depth_msg.depth;

    prev_depth_msg.depth = depth_msg.depth;

}

/**
 * Calculates motor control signal C given a desired state.
 *
 * @return None.
 */
void ControlSystem::CalculateThrusterMessage()
{
    ReloadPIDParams();

    motor_commands = motor_control();

    tp.data.clear();

    std::vector<double> motor_commands_vector(motor_commands.data(),
            motor_commands.data() + motor_commands.size());
    for (int i = 0; i < motor_commands_vector.size(); ++i)
        tp.data.push_back(motor_commands_vector[i]);
}

/**
 * Create a control message.
 *
 * @return A control message to be published.
 */
robosub::control ControlSystem::PublishControlState()
{
    robosub::control current_state;
    current_state.forward_state =  goal_types[0];
    current_state.strafe_state =  goal_types[1];
    current_state.dive_state =  goal_types[2];
    current_state.roll_state =  goal_types[3];
    current_state.pitch_state =  goal_types[4];
    current_state.yaw_state =  goal_types[5];

    current_state.forward = goals[0];
    current_state.strafe_left = goals[1];
    current_state.dive = goals[2];
    current_state.roll_right = goals[3] / _PI_OVER_180;
    current_state.pitch_down = goals[4] / _PI_OVER_180;
    current_state.yaw_left = goals[5] / _PI_OVER_180;

    return current_state;
}

/**
 * Publishes a thruster command.
 *
 * @return None.
 */
void ControlSystem::PublishThrusterMessage()
{
    pub->publish(tp);
}

/**
 * Calculates the motor control signal.
 *
 * @return The control signal C to send to the thrusters.
 */
VectorXd ControlSystem::motor_control()
{
    /*
     * Calculate the error between the current state and the goal. If the state
     * is set to error, override the error calculation to assume that our
     * current state is correct and no modification is required.
     */
    ROS_INFO_STREAM("Current State: \n" << state_vector);
    Vector3d rotation_goals;
    Vector3d translation_error = goals.segment<3>(0) - state_vector.segment<3>(0);
    for(int i=0; i < 3; ++i)
    {
        if(goal_types[i] == robosub::control::STATE_ERROR)
        {
            translation_error[i] = goals[i];
            integral_state[i] = 0.0;
        }
        if(goal_types[i+3] == robosub::control::STATE_ERROR)
        {
            rotation_goals[i] = state_vector[i+6] + goals[i+3];
            integral_state[i+3] = 0.0;
        }
        else
        {
            rotation_goals[i] = goals[i+3];
        }
    }
    Vector3d rotation_error = ir3D(r3D(state_vector.segment<3>(6)).transpose() * r3D(rotation_goals));

    Vector6d error;
    error << translation_error, rotation_error;
    ROS_INFO_STREAM("Translation Error: \n" << translation_error);
    ROS_INFO_STREAM("Rotation Error: \n" << rotation_error);

    /*
     * Update and bound-check the integral terms.
     */
    integral_state += error * dt;
    for(int i=0; i < 6; ++i)
    {
        if(fabs(integral_state(i)) > fabs(windup(i)))
            integral_state(i) = windup(i) * ((integral_state(i) < 0)? -1 : 1);
    }
    ROS_INFO_STREAM("Integral States: \n" << integral_state);

    /*
     * Nullify any controlling movements for proportional control if the error
     * is below the hysteresis threshold. Add in integral terms and incorporate
     * derivative terms.
     */
    Vector6d hist = (error.array().abs() >= hysteresis.array()).cast<double>();
    Vector6d derivative_vector = Vector6d::Zero();
    Vector6d m_accel = Vector6d::Zero();
    derivative_vector.segment<3>(0) = state_vector.segment<3>(3);
    derivative_vector.segment<3>(3) = state_vector.segment<3>(9);

    m_accel += P.cwiseProduct(error).cwiseProduct(hist);
    m_accel += I.cwiseProduct(integral_state);
    m_accel += D.cwiseProduct(derivative_vector);

    /*
     * Convert accelerations to force by multipling by masses.
     */
    Vector6d m_force = m_accel.cwiseProduct(sub_mass);
    ROS_INFO_STREAM("Desired Force: \n" << m_force);

    /*
     * Grab the current orientation of the submarine for rotating the current
     * translational goals. The order of this vector is roll, pitch, and yaw.
     * TODO: Verify that the yaw should be set to zero.
     */
    Vector3d current_orientation;
    current_orientation[0] = state_vector[6];
    current_orientation[1] = state_vector[7];
    current_orientation[2] = 0;

    /*
     * Normalize the translational forces based on the current orientation of
     * the submarine and calculate the translation control for each thruster.
     */
    Vector6d translation_command = Vector6d::Zero();
    translation_command.segment<3>(0) = r3D(current_orientation).transpose() *
        m_force.segment<3>(0);
    VectorXd translation_control = motors * translation_command;

    /*
     * Calculate the rotation control for each thruster.
     */
    Vector6d rotation_command = Vector6d::Zero();
    rotation_command.segment<3>(3) = m_force.segment<3>(3);
    VectorXd rotation_control = motors * rotation_command;
    ROS_INFO_STREAM("Rotation control (no truncation): \n" << translation_control);
    ROS_INFO_STREAM("Translation control (no truncation): \n" << rotation_control);

    /*
     * Truncate any goals that are over thresholds.
     * TODO: Intelligently scale each portion of the thruster goal so that all
     *       goals are equally represented.
     */
    for (int i=0; i < num_thrusters; ++i)
    {
        if(fabs(translation_control(i)) > t_lim)
            translation_control(i) = t_lim*((translation_control(i) < 0)? -1 : 1);
        if(fabs(rotation_control(i)) > r_lim)
            rotation_control(i) = r_lim*((rotation_control(i) < 0)? -1 : 1);
    }

    /*
     * Sum together the translation and rotation goals to attain a final
     * control for each thruster. Scale any reverse directions by the backward
     * thruster ratio to achieve our desired backward thrust goal.
     */
    VectorXd total_control = translation_control + rotation_control;
    for (int i = 0; i < num_thrusters; ++i)
    {
        if(total_control[i] < 0)
            total_control[i] /= back_thrust_ratio;
    }
    ROS_INFO_STREAM("Total control (truncated): \n" << total_control);

    return total_control;
}

/**
 * Wraps a value given to reside between a minimum and a maximum.
 *
 * @param x The value to wrap.
 * @param min The minimum boundary value.
 * @param max The maximum boundary value.
 *
 * @return The wrapped value within (min, max).
 */
double ControlSystem::wraparound(double x, double min, double max)
{
    double var;
    if (x > max)
        var = min + (x - max);
    else if (x < min)
        var = max + (x + max);
    else
        var = x;
    return var;
}
