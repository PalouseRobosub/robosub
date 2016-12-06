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
    nh.getParamCached("rate", dt);

    /*
     * Calculate the change in time between each call.
     * TODO: Replace dt with actual message timestamps.
     */
    dt = 1.0/dt;

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
    current_integral = Vector6d::Zero();

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
        ROS_FATAL("Failed to load thruster parameters.");
        exit(1);
    }
    ROS_INFO_STREAM("Loaded " << thruster_settings.size() << " thrusters.");
    num_thrusters = thruster_settings.size();

    MatrixXd position = MatrixXd(num_thrusters,3);
    MatrixXd orientation = MatrixXd(num_thrusters,3);
    motors = MatrixXd(6,num_thrusters);
    total_control = VectorXd::Zero(num_thrusters);

    for(int i = 0; i < num_thrusters; ++i)
    {
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

    ROS_INFO_STREAM("Motor Matrix:\n" << motors);
    ROS_INFO_STREAM("Thruster Orientations:\n" << orientation);
    ROS_INFO_STREAM("Thruster Positions:\n" << position);

    /*
     * Invert the motor matrix to quickly find the solution for the control
     * commands in future iterations. The algorithm is defined at:
     *      http://robosub.eecs.wsu.edu/wiki/cs/control/start
     */
    if(motors.rows() == motors.cols())
    {
        motors = motors.inverse();
    }
    else
    {
        motors = pinv(motors);
    }

    ROS_INFO_STREAM("Motor Matrix Inverted:\n" << motors);
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
    std::vector<double> control_values(6);
    std::vector<uint8_t> control_states(6);

    control_states[0] = msg.forward_state;
    control_states[1] = msg.strafe_state;
    control_states[2] = msg.dive_state;
    control_states[3] = msg.roll_state;
    control_states[4] = msg.pitch_state;
    control_states[5] = msg.yaw_state;

    control_values[0] = msg.forward;
    control_values[1] = msg.strafe_left;
    control_values[2] = msg.dive;
    control_values[3] = msg.roll_right;
    control_values[4] = msg.pitch_down;
    control_values[5] = msg.yaw_left;

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
                    goals[i] = wraparound(state_vector[i+3] + control_values[i], -180.0, 180.0);
                else if (i == 4)
                    goals[i] = wraparound(state_vector[i+3] + control_values[i], -90.0, 90.0);
                else if (i == 5)
                    goals[i] = wraparound(state_vector[i+3] + control_values[i], -180.0, 180.0);
                break;

            case robosub::control::STATE_ERROR:
                this->goal_types[i] = robosub::control::STATE_ERROR;
                this->goals[i] = control_values[i];
                break;

            case robosub::control::STATE_NONE:
                break;

            default:
                ROS_ERROR("Received invalid control state.");
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
    state_vector[6] *= _180_OVER_PI;
    state_vector[7] *= _180_OVER_PI;
    state_vector[8] *= _180_OVER_PI;
    /*
    state_vector[9] = sp.droll; //roll'
    state_vector[10] = sp.dpitch; //pitch'
    state_vector[11] = sp.dyaw; //yaw'
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
    /*
     * Force a reload of control system parameters incase they have been
     * updated since the last control cycle.
     */
    ReloadPIDParams();

    /*
     * Calculate the new motor controls. The result will be stored in the
     * internal total_control vector.
     */
    calculate_motor_control();

    /*
     * Create a new thruster control message based upon the newly calculated
     * total_control vector.
     */
    tp.data.clear();
    std::vector<double> control_vector(total_control.data(),
            total_control.data() + total_control.size());

    for (unsigned int i = 0; i < control_vector.size(); ++i)
    {
        tp.data.push_back(control_vector[i]);
    }
}

/**
 * Create a control status message.
 *
 * @return A control message to be published.
 */
robosub::control_status ControlSystem::GetControlStatus()
{
    robosub::control_status current_state;

    current_state.forward_state =  state_to_string(goal_types[0]);
    current_state.strafe_left_state =  state_to_string(goal_types[1]);
    current_state.dive_state =  state_to_string(goal_types[2]);
    current_state.roll_right_state =  state_to_string(goal_types[3]);
    current_state.pitch_down_state =  state_to_string(goal_types[4]);
    current_state.yaw_left_state =  state_to_string(goal_types[5]);

    current_state.forward_goal = goals[0];
    current_state.strafe_left_goal = goals[1];
    current_state.dive_goal = goals[2];
    current_state.roll_right_goal = goals[3];
    current_state.pitch_down_goal = goals[4];
    current_state.yaw_left_goal = goals[5];

    current_state.forward_error = current_error[0];
    current_state.strafe_left_error = current_error[1];
    current_state.dive_error = current_error[2];
    current_state.roll_right_error = current_error[3];
    current_state.pitch_down_error = current_error[4];
    current_state.yaw_left_error = current_error[5];

    current_state.forward_integral = current_integral[0];
    current_state.strafe_left_integral = current_integral[1];
    current_state.dive_integral = current_integral[2];
    current_state.roll_right_integral = current_integral[3];
    current_state.pitch_down_integral = current_integral[4];
    current_state.yaw_left_integral = current_integral[5];

    current_state.forward_derivative = current_derivative[0];
    current_state.strafe_left_derivative = current_derivative[1];
    current_state.dive_derivative = current_derivative[2];
    current_state.roll_right_derivative = current_derivative[3];
    current_state.pitch_down_derivative = current_derivative[4];
    current_state.yaw_left_derivative = current_derivative[5];

    std::vector<float> rotation_control_vector(rotation_control.data(),
            rotation_control.data() + rotation_control.size());
    std::vector<float> translation_control_vector(translation_control.data(),
            translation_control.data() + translation_control.size());

    for (unsigned int i = 0; i < rotation_control_vector.size(); ++i)
    {
        current_state.translation_control.push_back(translation_control_vector[i]);
        current_state.rotation_control.push_back(rotation_control_vector[i]);
    }

    return current_state;
}

std::string ControlSystem::state_to_string(uint8_t state)
{
    switch (state)
    {
        case robosub::control::STATE_NONE:
            return "No State";
            break;
        case robosub::control::STATE_ERROR:
            return "Error State";
            break;
        case robosub::control::STATE_ABSOLUTE:
            return "Absolute State";
            break;
        case robosub::control::STATE_RELATIVE:
            return "Relative State";
            break;
        default:
            return "Unknown State";
            break;
    }
    return "Unknown State";
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
void ControlSystem::calculate_motor_control()
{
    /*
     * Calculate the error between the current state and the goal. If the state
     * is set to error, override the error calculation to assume that our
     * current state is correct and no modification is required.
     */
    Vector3d rotation_goals;
    Vector3d translation_error = goals.segment<3>(0) - state_vector.segment<3>(0);
    for(int i=0; i < 3; ++i)
    {
        if(goal_types[i] == robosub::control::STATE_ERROR)
        {
            translation_error[i] = goals[i];
            current_integral[i] = 0.0;
        }
        if(goal_types[i+3] == robosub::control::STATE_ERROR)
        {
            rotation_goals[i] = state_vector[i+6] + goals[i+3];
            current_integral[i+3] = 0.0;
        }
        else
        {
            rotation_goals[i] = goals[i+3];
        }
    }
    Vector3d rotation_error = ir3D(
            r3D(state_vector.segment<3>(6)).transpose() * r3D(rotation_goals));

    /*
     * Update the current error vector with the calculated errors.
     */
    current_error = Vector6d::Zero();
    current_error << translation_error, rotation_error;

    /*
     * Update and bound-check the integral terms.
     */
    current_integral += current_error * dt;
    for(int i=0; i < 6; ++i)
    {
        ROS_INFO_STREAM("Integral: " << current_integral[i] << " Windup: " << windup[i]);
        if(fabs(current_integral[i]) > fabs(windup[i]))
        {
            current_integral[i] = windup[i] * ((current_integral[i] < 0)? -1 : 1);
            ROS_INFO_STREAM("Truncating integral " << i << " to " << current_integral[i]);
        }
    }

    /*
     * Nullify any controlling movements for proportional control if the error
     * is below the hysteresis threshold. Add in integral terms and incorporate
     * derivative terms.
     */
    Vector6d hist = (current_error.array().abs() >= hysteresis.array()).cast<double>();
    current_derivative = Vector6d::Zero();
    Vector6d m_accel = Vector6d::Zero();
    current_derivative.segment<3>(0) = state_vector.segment<3>(3);
    current_derivative.segment<3>(3) = state_vector.segment<3>(9);

    m_accel += P.cwiseProduct(current_error).cwiseProduct(hist);
    m_accel += I.cwiseProduct(current_integral);
    m_accel += D.cwiseProduct(current_derivative);

    /*
     * Convert accelerations to force by multipling by masses.
     */
    Vector6d m_force = m_accel.cwiseProduct(sub_mass);

    /*
     * Grab the current orientation of the submarine for rotating the current
     * translational goals. The order of this vector is roll, pitch, and yaw.
     * Note that by setting the yaw to zero, all control signals are relative.
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
    translation_control = motors * translation_command;

    /*
     * Calculate the rotation control for each thruster.
     */
    Vector6d rotation_command = Vector6d::Zero();
    rotation_command.segment<3>(3) = m_force.segment<3>(3);
    rotation_control = motors * rotation_command;

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
    total_control = translation_control + rotation_control;
    for (int i = 0; i < num_thrusters; ++i)
    {
        if(total_control[i] < 0)
            total_control[i] /= back_thrust_ratio;
    }
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
