#include "movement/control_system.h"

#include <vector>
#include <string>
#include <deque>

namespace robosub
{
    /**
     * Constructor.
     */
    ControlSystem::ControlSystem()
    {
        enabled = false;
        ReloadPIDParams();

        /*
         * Load parameters from the settings file.
         */
        ROS_ERROR_COND(!ros::param::getCached("control/inertia/psi",
                sub_mass[3]), "Failed to load inertial mass psi.");
        ROS_ERROR_COND(!ros::param::getCached("control/inertia/phi",
                sub_mass[4]), "Failed to load inertial mass phi");
        ROS_ERROR_COND(!ros::param::getCached("control/inertia/theta",
                sub_mass[5]), "Failed to load inertial mass theta.");
        ROS_ERROR_COND(!ros::param::getCached("control/limits/translation",
                t_lim), "Failed to load the translation control limit.");
        ROS_ERROR_COND(!ros::param::getCached("control/limits/rotation",
                r_lim), "Failed to load the rotiation control limit.");
        ROS_ERROR_COND(!ros::param::getCached("thrusters/max_thrust",
                max_thrust), "Failed to load the max thrust output.");

        /*
         * All translational masses are the submarines total mass.
         */
        ROS_ERROR_COND(!ros::param::getCached("control/mass", sub_mass[0]),
                "Failed to load mass of the submarine.");
        sub_mass[1] = sub_mass[2] = sub_mass[0];

        /*
         * Initialize the state of the goals to error.
         */
        for(int i = 0; i < 6; ++i)
        {
            goal_types[i] = robosub_msgs::control::STATE_ERROR;

            /*
             * Set the previous message queues to empty.
             */
            previous_error[i].clear();

            new_measurement_available[i] = false;
        }

        /*
         * Ensure that the commands and states are all set to zero initially.
         */
        state_vector = Vector6d::Zero();
        current_error = Vector6d::Zero();
        current_integral = Vector6d::Zero();
        current_derivative = Vector6d::Zero();
        goals = Vector6d::Zero();
        acceleration_estimate = Vector6d::Zero();

        /*
         * Load thruster node settings. A nodehandle reference is not
         * available, so make use of rosparam get.
         */
        XmlRpc::XmlRpcValue thruster_settings;
        if(!ros::param::getCached("thrusters/mapping", thruster_settings))
        {
            ROS_FATAL("Failed to load thruster parameters.");
            exit(1);
        }
        ROS_INFO_STREAM("Loaded " << thruster_settings.size() <<
                " thrusters.");
        num_thrusters = thruster_settings.size();

        MatrixXd position = MatrixXd(num_thrusters, 3);
        MatrixXd orientation = MatrixXd(num_thrusters, 3);
        MatrixXd motors = MatrixXd(6, num_thrusters);
        total_control = VectorXd::Zero(num_thrusters);
        translation_control = VectorXd::Zero(num_thrusters);
        rotation_control = VectorXd::Zero(num_thrusters);

        for(int i = 0; i < num_thrusters; ++i)
        {
            /*
             * Load in the thruster positions and orientations. The column of
             * the position and orientation matrices denote the x, y, and z
             * components sequentially.
             */
            position(i, 0) = thruster_settings[i]["position"]["x"];
            position(i, 1) = thruster_settings[i]["position"]["y"];
            position(i, 2) = thruster_settings[i]["position"]["z"];
            orientation(i, 0) = thruster_settings[i]["orientation"]["x"];
            orientation(i, 1) = thruster_settings[i]["orientation"]["y"];
            orientation(i, 2) = thruster_settings[i]["orientation"]["z"];
        }

        for (int i = 0; i < num_thrusters; ++i)
        {
            /*
             * Calculate the force and moment for each thruster.
             * TODO: Validate moment calculations are correct through
             * utilization of the cross product.
             */
            motors.block<3, 1>(0, i) = max_thrust *
                    orientation.block<1, 3>(i, 0).transpose();
            motors.block<3, 1>(3, i) = max_thrust *
                    position.block<1, 3>(i, 0).cross(
                    orientation.block<1, 3>(i, 0)).transpose();
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
            motors_inverted = motors.inverse();
        }
        else
        {
            motors_inverted = pinv(motors);
        }

        ROS_INFO_STREAM("Motor Matrix Inverted:\n" << motors_inverted);
    }

    /*
      getter function that returns variable telling us if we should
      allow thruster message to publish
    */
    bool ControlSystem::isEnabled(void)
    {
        return enabled;
    }

    /*
       Enables/disables the control system.
    */
    void ControlSystem::setEnabled(bool enable)
    {
        enabled = enable;
    }

    /*
       Checks if the control system has received a control system recently, if
       it hasen't then the control system should be disabled.
    */
    void ControlSystem::CheckTimeout(const ros::TimerEvent& timer_event)
    {
        float param_duration = 2.0;
        ros::param::getCached("control/timeout", param_duration);
        ros::Duration timeout_duration(param_duration);

        if (ros::Time::now() > last_msg_time + timeout_duration)
        {
            ROS_WARN_COND(this->enabled == true,
                    "control system has timed out.");
            this->enabled = false;
        }
    }

    /**
     * Reload PID parameters from the ROS parameter server.
     *
     * @return None.
     */
    void ControlSystem::ReloadPIDParams()
    {
        ros::param::getCached("control/proportional/x", P[0]);
        ros::param::getCached("control/proportional/y", P[1]);
        ros::param::getCached("control/proportional/z", P[2]);
        ros::param::getCached("control/proportional/psi", P[3]);
        ros::param::getCached("control/proportional/phi", P[4]);
        ros::param::getCached("control/proportional/theta", P[5]);

        ros::param::getCached("control/integral/x", I[0]);
        ros::param::getCached("control/integral/y", I[1]);
        ros::param::getCached("control/integral/z", I[2]);
        ros::param::getCached("control/integral/psi", I[3]);
        ros::param::getCached("control/integral/phi", I[4]);
        ros::param::getCached("control/integral/theta", I[5]);

        ros::param::getCached("control/derivative/x", D[0]);
        ros::param::getCached("control/derivative/y", D[1]);
        ros::param::getCached("control/derivative/z", D[2]);
        ros::param::getCached("control/derivative/psi", D[3]);
        ros::param::getCached("control/derivative/phi", D[4]);
        ros::param::getCached("control/derivative/theta", D[5]);

        ros::param::getCached("control/windup/x", windup[0]);
        ros::param::getCached("control/windup/y", windup[1]);
        ros::param::getCached("control/windup/z", windup[2]);
        ros::param::getCached("control/windup/psi", windup[3]);
        ros::param::getCached("control/windup/phi", windup[4]);
        ros::param::getCached("control/windup/theta", windup[5]);

        ros::param::getCached("control/hysteresis/x", hysteresis[0]);
        ros::param::getCached("control/hysteresis/y", hysteresis[1]);
        ros::param::getCached("control/hysteresis/z", hysteresis[2]);
        ros::param::getCached("control/hysteresis/psi", hysteresis[3]);
        ros::param::getCached("control/hysteresis/phi", hysteresis[4]);
        ros::param::getCached("control/hysteresis/theta", hysteresis[5]);

        ros::param::getCached("control/buoyancy_offset", buoyancy_offset);
    }

    /**
     * Update the current goals of the submarine.
     *
     * @note This function updates the desired state of the submarine.
     *
     * @param msg The control message used to update the desired state.
     *
     * @return None.
     */
    void ControlSystem::InputControlMessage(const
            robosub_msgs::control::ConstPtr& msg)
    {
        enabled = true;
        last_msg_time = ros::Time::now();

        std::vector<double> control_values(6);
        std::vector<uint8_t> control_states(6);

        control_states[0] = msg->forward_state;
        control_states[1] = msg->strafe_state;
        control_states[2] = msg->dive_state;
        control_states[3] = msg->roll_state;
        control_states[4] = msg->pitch_state;
        control_states[5] = msg->yaw_state;

        control_values[0] = msg->forward;
        control_values[1] = msg->strafe_left;
        control_values[2] = msg->dive;
        control_values[3] = msg->roll_right;
        control_values[4] = msg->pitch_down;
        control_values[5] = msg->yaw_left;

        for(int i = 0; i < 6; ++i)
        {
            switch(control_states[i])
            {
                case robosub_msgs::control::STATE_ABSOLUTE:
                    this->goal_types[i] = robosub_msgs::control::STATE_ABSOLUTE;
                    this->goals[i] = control_values[i];
                    break;

                /*
                 * Note that all rotational goals must be wrapped according to
                 * the following bounding limits:
                 *   Roll (PSI): (-180, 180)
                 *   Pitch (PHI): (-90, 90)
                 *   Yaw (THETA): (-180, 180)
                 */
                case robosub_msgs::control::STATE_RELATIVE:
                    /*
                     * Set the current state to absolute, but update the goal
                     * by adding the new goal relative to the current state.
                     */

                    //if we just want to maintain our current state
                    if(control_values[i] == 0)
                    {
                        //if the previous goal was not absolute we should use
                        //our current state as the new absolute goal
                        if (this->goal_types[i] !=
                            robosub_msgs::control::STATE_ABSOLUTE)
                        {
                            this->goals[i] = state_vector[i];
                        }
                        //else, the previous goal was absolute, we can just
                        //reuse the last absolute goal (don't need to change)
                    }
                    //else, the update is non-zero, use the normal update logic
                    else
                    {
                        if(i < 3)
                        {
                            this->goals[i] = state_vector[i] +
                                             control_values[i];
                        }
                        else if (i == 3)
                        {
                            this->goals[i] = wraparound(state_vector[i] +
                                    control_values[i], -180.0, 180.0);
                        }
                        else if (i == 4)
                        {
                            this->goals[i] = wraparound(state_vector[i] +
                                    control_values[i], -90.0, 90.0);
                        }
                        else if (i == 5)
                        {
                            this->goals[i] = wraparound(state_vector[i] +
                                    control_values[i], -180.0, 180.0);
                        }
                    }

                    this->goal_types[i] = robosub_msgs::control::STATE_ABSOLUTE;

                    break;

                case robosub_msgs::control::STATE_ERROR:
                    this->goal_types[i] = robosub_msgs::control::STATE_ERROR;
                    this->goals[i] = control_values[i];
                    break;

                case robosub_msgs::control::STATE_NONE:
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
     * @brief Updates the current state vector S of the submarine given an
     *        input orientation message.
     *
     * @param quat_msg The input ROS Quaternion message that defines
     *        orientation.
     *
     * @return None.
     */
    void ControlSystem::InputOrientationMessage(
            const geometry_msgs::QuaternionStamped::ConstPtr &quat_msg)
    {
        /*
         * Convert the Quaternion to roll, pitch, and yaw and store the result
         * into the state vector.
         */
        tf::Matrix3x3 m(tf::Quaternion(quat_msg->quaternion.x,
                    quat_msg->quaternion.y, quat_msg->quaternion.z,
                    quat_msg->quaternion.w));
        m.getRPY(state_vector[3], state_vector[4], state_vector[5]);

        /*
         * Note that input values are calculated to be radians, but all
         * internal values of the control system are represented in degrees.
         * Convert the radians to degrees.
         */
        state_vector[3] *= _180_OVER_PI;
        state_vector[4] *= _180_OVER_PI;
        state_vector[5] *= _180_OVER_PI;

        /*
         * Indicate that new rotation state measurements are available.
         */
        new_measurement_available[3] = true;
        new_measurement_available[4] = true;
        new_measurement_available[5] = true;
    }

    /**
     * Update the current state of the submarine with a localization message
     *
     * @brief Updates the current state vector of the submarine given an
     *        input localization message
     *
     * @param point_msg The input ROS PointStamped geometry message that defines
     *        position
     *
     * @return None.
     */
    void ControlSystem::InputLocalizationMessage(
            const geometry_msgs::PointStamped::ConstPtr &point_msg)
    {
        state_vector[0] = point_msg->point.x;
        state_vector[1] = point_msg->point.y;

        /*
         * Indicate that new state measurements are available for X and Y.
         */
        new_measurement_available[0] = true;
        new_measurement_available[1] = true;
    }

    /**
     * Update the depth of the submarine.
     *
     * @brief Updates the current state vector with an updated depth reading.
     *
     * @return None.
     */
    void ControlSystem::InputDepthMessage(const
            robosub_msgs::Float32Stamped::ConstPtr& depth_msg)
    {
        state_vector[2] = depth_msg->data;

        /*
         * Indicate that a new Z position measurement has been received.
         */
        new_measurement_available[2] = true;
    }

    /**
     * Calculates a thruster control signal with the current and desired state.
     *
     * @return A thruster packet calculated for the latest control state.
     */
    robosub_msgs::thruster ControlSystem::CalculateThrusterMessage()
    {
        /*
         * Force a reload of control system parameters incase they have been
         * updated since the last control cycle.
         */
        ReloadPIDParams();

        /*
         * Calculate the new motor controls. The result will be stored in the
         * internal total_control vector. Begin by calculating the error
         * between the current state and the goal. If the state is set to
         * error, override the error calculation to assume that our current
         * state is correct and no modification is required.
         */
        Vector3d rotation_goals;
        Vector3d translation_error = goals.segment<3>(0) -
                state_vector.segment<3>(0);
        for(int i = 0; i < 3; ++i)
        {
            if(goal_types[i] == robosub_msgs::control::STATE_ERROR)
            {
                translation_error[i] = goals[i];
                current_integral[i] = 0.0;
            }
        }

        for (int state_index = 3, i = 0; i < 3; ++i, ++state_index)
        {
            if(goal_types[state_index] == robosub_msgs::control::STATE_ERROR)
            {
                rotation_goals[i] = state_vector[state_index] +
                        goals[state_index];
                current_integral[state_index] = 0.0;
            }
            else
            {
                rotation_goals[i] = goals[state_index];
            }
        }

        Vector3d rotation_error = ir3D(
                r3D(state_vector.segment<3>(3)).transpose() *
                r3D(rotation_goals));

        /*
         * Have yaw pursue the supplement angle to the goal when yaw and roll
         * errors are both greater than +/- 90 degrees
         */
        if (fabs(rotation_error(2, 0)) > 90 && fabs(rotation_error(0, 0)) > 90)
        {
            if (rotation_error(2, 0) > 0)
            {
                rotation_error(2, 0) -= 180;
            }
            else
            {
                rotation_error(2, 0) += 180;
            }

            if (rotation_error(0, 0) > 0)
            {
                rotation_error(0, 0) -= 180;
            }
            else
            {
                rotation_error(0, 0) += 180;
            }
        }

        /*
         * Multiply the pitch error by the cosine (in degrees) of the yaw error
         * to use positive feedback for pitch to assist yaw when yawing more
         * than 90 degrees.
         */
        rotation_error(1, 0) *= cos(rotation_error(2, 0) * 3.1415 / 180);

        rotation_error(0, 0) *= cos(rotation_error(2, 0) * 3.1415 / 180);

        /*
         * Update the current error vector with the calculated errors.
         */
        current_error = Vector6d::Zero();
        current_error << translation_error, rotation_error;

        /*
         * Update and bound-check the integral terms.
         */
        const ros::Time now = ros::Time::now();
        for (int i = 0; i < 6; ++i)
        {
            if (new_measurement_available[i] && previous_error[i].size() >= 1)
            {
                current_integral[i] += current_error[i] *
                    (now - previous_error[i][0].time).toSec();
            }
        }

        for (int i = 0; i < 6; ++i)
        {
            if(fabs(current_integral[i]) > fabs(windup[i]))
            {
                current_integral[i] = windup[i] *
                        ((current_integral[i] < 0)? -1 : 1);
            }
        }

        /*
         * Nullify any controlling movements for proportional control if the
         * error is below the hysteresis threshold. Add in integral terms and
         * incorporate derivative terms.
         */
        Vector6d hist = (current_error.array().abs() >=
                hysteresis.array()).cast<double>();
        Vector6d m_accel = Vector6d::Zero();

        /*
         * If a new measurement came in during the update cycle, update the
         * previous error states. This prevents issues that occur when the
         * control update loop runs faster than a sensor is
         * reporting and prevents the miscalculation of the error
         * derivative.
         */
        for (int i = 0; i < 6; ++i)
        {
            if (new_measurement_available[i])
            {
                previous_error[i].push_front(
                        StateMeasurement(now, current_error[i]));

                if (previous_error[i].size() > 5)
                {
                    previous_error[i].pop_back();
                }
            }

            new_measurement_available[i] = false;
        }

        /*
         * Calculate the derivative of the error using linear
         * regression. Note that all time is relative to the earliest sampled
         * time. The following linear regression mode is used:
         *     Slope = (n(Sum(x*y)) - Sum(x)*Sum(y)) / (n(Sum(x^2) - Sum(x)^2))
         * Where y is the error and x is the time.
         */
        for (int j = 0; j < 6; ++j)
        {
            std::deque<StateMeasurement> &measurements = previous_error[j];

            const int number_elements = measurements.size();
            if (number_elements <= 1)
            {
                continue;
            }

            double sum_time = 0, sum_time_sq = 0;
            double sum_error = 0;
            double sum_error_time = 0;
            for (int i = 0; i < number_elements; ++i)
            {
                double current_time = (measurements[i].time -
                        measurements[number_elements - 1].time).toSec();
                sum_time += current_time;
                sum_time_sq += current_time * current_time;
                sum_error += measurements[i].val;
                sum_error_time += current_time * measurements[i].val;
            }

            current_derivative[j] = (number_elements * sum_error_time -
                    sum_time * sum_error) / (number_elements * sum_time_sq -
                    sum_time * sum_time);
        }

        m_accel += P.cwiseProduct(current_error).cwiseProduct(hist);
        m_accel += I.cwiseProduct(current_integral);
        m_accel += D.cwiseProduct(current_derivative);

        //saves the most recent calculated acceleration for publishing
        this->acceleration_estimate = m_accel;

        /*
         * Convert accelerations to force by multipling by masses.
         */
        Vector6d m_force = m_accel.cwiseProduct(sub_mass);

        /*
         * Add additional force to diving to offset buoyance force. This allows
         * use to use smaller PID gains to maintain depth and makes things
         * behave much nicer.
         */
        m_force[2] += buoyancy_offset;

        /*
         * Grab the current orientation of the submarine for rotating the
         * current translational goals. The order of this vector is roll,
         * pitch, and yaw.
         *
         * Yaw is set to 0 to use the relative reference frame when neither
         * the forward nor strafe state is absolute.
         */
        Vector3d current_orientation;
        current_orientation[0] = state_vector[3];
        current_orientation[1] = state_vector[4];

        if (goal_types[0] == robosub_msgs::control::STATE_ABSOLUTE ||
            goal_types[1] == robosub_msgs::control::STATE_ABSOLUTE)
        {
            //Use the global reference frame when either x or y
            // state is absolute
            current_orientation[2] = state_vector[5];
        }
        else
        {
            //Use the relative reference frame when neither x nor y
            // state is absolute.
            current_orientation[2] = 0;
        }

        /*
         * Normalize the translational forces based on the current orientation
         * of the submarine and calculate the translation control for each
         * thruster.
         */
        Vector6d translation_command = Vector6d::Zero();
        translation_command.segment<3>(0) =
            r3D(current_orientation).transpose() * m_force.segment<3>(0);
        translation_control = motors_inverted * translation_command;

        /*
         * Calculate the rotation control for each thruster.
         */
        Vector6d rotation_command = Vector6d::Zero();
        rotation_command.segment<3>(3) = m_force.segment<3>(3);
        rotation_control = motors_inverted * rotation_command;

        /*
         * Truncate any goals that are over thresholds.
         * TODO: Intelligently scale each portion of the thruster goal so that
         *       all goals are equally represented.
         */
        for (int i=0; i < num_thrusters; ++i)
        {
            if(fabs(translation_control(i)) > t_lim)
            {
                translation_control(i) = t_lim*((translation_control(i) < 0)?
                        -1 : 1);
            }
            if(fabs(rotation_control(i)) > r_lim)
            {
                rotation_control(i) = r_lim*((rotation_control(i) < 0)?
                        -1 : 1);
            }
        }

        /*
         * Sum together the translation and rotation goals to attain a final
         * control for each thruster.
         */
        total_control = translation_control + rotation_control;

        /*
         * Create a new thruster control message based upon the newly
         * calculated total_control vector.
         */
        robosub_msgs::thruster thruster_message;

        for (int i = 0; i < num_thrusters; ++i)
        {
            thruster_message.data.push_back(total_control[i]);
        }

        return thruster_message;
    }

    /**
     * Creates a thruster message that is all zeros
     *
     * @return A thruster message to be published.
     */
    robosub_msgs::thruster ControlSystem::GetZeroThrusterMessage()
    {
        robosub_msgs::thruster thruster_message;

        for (int i = 0; i < num_thrusters; ++i)
        {
            thruster_message.data.push_back(0);
        }

        return thruster_message;
    }

    /**
     * Create a control status message.
     *
     * @return A control message to be published.
     */
    robosub_msgs::control_status ControlSystem::GetControlStatus()
    {
        robosub_msgs::control_status current_state;

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

        /*
         * Load eigen vectors into the message.
         */
        for (int i = 0; i < num_thrusters; ++i)
        {
            current_state.translation_control.push_back(
                    translation_control[i]);
            current_state.rotation_control.push_back(rotation_control[i]);
        }

        return current_state;
    }

    /**
     * Transforms a state enumeration into a string.
     *
     * @param state The state enumeration to transform.
     *
     * @return A string describing the provided state value.
     */
    std::string ControlSystem::state_to_string(uint8_t state)
    {
        std::string ret = "Unknown State";
        switch (state)
        {
            case robosub_msgs::control::STATE_NONE:
                ret = "NONE";
                break;
            case robosub_msgs::control::STATE_ERROR:
                ret = "ERROR";
                break;
            case robosub_msgs::control::STATE_ABSOLUTE:
                ret = "ABSOLUTE";
                break;
            case robosub_msgs::control::STATE_RELATIVE:
                ret = "RELATIVE";
                break;
            default:
                break;
        }
        return ret;
    }

    /**
     * Calculates the motor control signal.
     *
     * @return None.
     */
    void ControlSystem::calculate_motor_control()
    {
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

    geometry_msgs::Accel ControlSystem::GetAccelerationEstimate()
    {
        geometry_msgs::Accel msg;

        msg.linear.x = this->acceleration_estimate[0];
        msg.linear.y = this->acceleration_estimate[1];
        msg.linear.z = this->acceleration_estimate[2];
        msg.angular.x = this->acceleration_estimate[3];
        msg.angular.y = this->acceleration_estimate[4];
        msg.angular.z = this->acceleration_estimate[5];

        return msg;
    }
}
