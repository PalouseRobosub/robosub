#include "movement/control_system.h"

#include <vector>
#include <string>

namespace robosub
{
    /**
     * Constructor.
     */
    ControlSystem::ControlSystem()
    {
        ReloadPIDParams();

        /*
         * Load parameters from the settings file.
         */
        int rate = 0;

        /*
         * All translational masses are the submarins total mass.
         */
        ROS_ERROR_COND(!ros::param::getCached("/control/mass", sub_mass[0]),
                "Failed to load mass of the submarine.");
        sub_mass[1] = sub_mass[2] = sub_mass[0];

        ROS_ERROR_COND(!ros::param::getCached("/control/inertia/psi",
                sub_mass[3]), "Failed to load inertial mass psi.");
        ROS_ERROR_COND(!ros::param::getCached("/control/inertia/phi",
                sub_mass[4]), "Failed to load inertial mass phi");
        ROS_ERROR_COND(!ros::param::getCached("/control/inertia/theta",
                sub_mass[5]), "Failed to load inertial mass theta.");
        ROS_ERROR_COND(!ros::param::getCached("/control/back_thrust_ratio",
                back_thrust_ratio), "Failed to load the back thrust ratio.");
        ROS_ERROR_COND(!ros::param::getCached("/control/limits/translation",
                t_lim), "Failed to load the translation control limit.");
        ROS_ERROR_COND(!ros::param::getCached("/control/limits/rotation",
                r_lim), "Failed to load the rotiation control limit.");
        ROS_ERROR_COND(!ros::param::getCached("/control/max_thrust",
                max_thrust), "Failed to load the max thrust output.");
        ROS_ERROR_COND(!ros::param::getCached("/control/rate", rate),
                "Failed to load the control system rate.");

        /*
         * Calculate the change in time between each call.
         * TODO: Replace dt with actual message timestamps.
         */
        if (rate <= 0)
        {
            ROS_FATAL("Control system rate is specified as zero or less.");
            exit(-1);
        }

        dt = 1.0/rate;

        /*
         * Initialize the state of the goals to error.
         */
        for(int i = 0; i < 6; ++i)
        {
            goal_types[i] = robosub::control::STATE_ERROR;
        }

        /*
         * Set the previous message queues to empty.
         */
        previous_quaternion_msgs.clear();
        previous_depth_msgs.clear();
        previous_quaternion_msgs_times.clear();
        previous_depth_msgs_times.clear();

        /*
         * Ensure that the commands and states are all set to zero initially.
         */
        state_vector = Vector12d::Zero();
        current_integral = Vector6d::Zero();

        /*
         * Scale thruster limits to be in terms of the backwards thruster
         * ratio.
         */
        t_lim *= back_thrust_ratio;
        r_lim *= back_thrust_ratio;

        /*
         * Load thruster node settings. A nodehandle reference is not
         * available, so make use of rosparam get.
         */
        XmlRpc::XmlRpcValue thruster_settings;
        if(!ros::param::getCached("thrusters", thruster_settings))
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

    /**
     * Reload PID parameters from the ROS parameter server.
     *
     * @return None.
     */
    void ControlSystem::ReloadPIDParams()
    {
        ros::param::getCached("/control/proportional/x", P[0]);
        ros::param::getCached("/control/proportional/y", P[1]);
        ros::param::getCached("/control/proportional/z", P[2]);
        ros::param::getCached("/control/proportional/psi", P[3]);
        ros::param::getCached("/control/proportional/phi", P[4]);
        ros::param::getCached("/control/proportional/theta", P[5]);

        ros::param::getCached("/control/integral/x", I[0]);
        ros::param::getCached("/control/integral/y", I[1]);
        ros::param::getCached("/control/integral/z", I[2]);
        ros::param::getCached("/control/integral/psi", I[3]);
        ros::param::getCached("/control/integral/phi", I[4]);
        ros::param::getCached("/control/integral/theta", I[5]);

        ros::param::getCached("/control/derivative/x", D[0]);
        ros::param::getCached("/control/derivative/y", D[1]);
        ros::param::getCached("/control/derivative/z", D[2]);
        ros::param::getCached("/control/derivative/psi", D[3]);
        ros::param::getCached("/control/derivative/phi", D[4]);
        ros::param::getCached("/control/derivative/theta", D[5]);

        ros::param::getCached("/control/windup/x", windup[0]);
        ros::param::getCached("/control/windup/y", windup[1]);
        ros::param::getCached("/control/windup/z", windup[2]);
        ros::param::getCached("/control/windup/psi", windup[3]);
        ros::param::getCached("/control/windup/phi", windup[4]);
        ros::param::getCached("/control/windup/theta", windup[5]);

        ros::param::getCached("/control/hysteresis/x", hysteresis[0]);
        ros::param::getCached("/control/hysteresis/y", hysteresis[1]);
        ros::param::getCached("/control/hysteresis/z", hysteresis[2]);
        ros::param::getCached("/control/hysteresis/psi", hysteresis[3]);
        ros::param::getCached("/control/hysteresis/phi", hysteresis[4]);
        ros::param::getCached("/control/hysteresis/theta", hysteresis[5]);
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
            robosub::control::ConstPtr& msg)
    {
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
                case robosub::control::STATE_ABSOLUTE:
                    this->goal_types[i] = robosub::control::STATE_ABSOLUTE;
                    this->goals[i] = control_values[i];
                    break;

                /*
                 * Note that all rotational goals must be wrapped according to
                 * the following bounding limits:
                 *   Roll (PSI): (-180, 180)
                 *   Pitch (PHI): (-90, 90)
                 *   Yaw (THETA): (-180, 180)
                 */
                case robosub::control::STATE_RELATIVE:
                    /*
                     * Set the current state to absolute, but update the goal
                     * by adding in the new goal to the current goal to make it
                     * relative to the current state.
                     */
                    this->goal_types[i] = robosub::control::STATE_ABSOLUTE;
                    if(i < 3)
                        this->goals[i] = state_vector[i] + control_values[i];
                    else if (i == 3)
                        this->goals[i] = wraparound(state_vector[i+3] +
                                control_values[i], -180.0, 180.0);
                    else if (i == 4)
                        this->goals[i] = wraparound(state_vector[i+3] +
                                control_values[i], -90.0, 90.0);
                    else if (i == 5)
                        this->goals[i] = wraparound(state_vector[i+3] +
                                control_values[i], -180.0, 180.0);
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
     * @brief Updates the current state vector S of the submarine given an
     *        input orientation message.
     *
     * @param quat_msg The input ROS Quaternion message that defines
     *        orientation.
     *
     * @return None.
     */
    void ControlSystem::InputOrientationMessage(
            const robosub::QuaternionStampedAccuracy::ConstPtr &quat_msg)
    {
        /*
         * Convert the Quaternion to roll, pitch, and yaw and store the result
         * into the state vector.
         */
        tf::Matrix3x3 m(tf::Quaternion(quat_msg->quaternion.x,
                    quat_msg->quaternion.y, quat_msg->quaternion.z,
                    quat_msg->quaternion.w));
        m.getRPY(state_vector[6], state_vector[7], state_vector[8]);

        /*
         * Note that input values are calculated to be radians, but all
         * internal values of the control system are represented in degrees.
         * Convert the radians to degrees.
         */
        state_vector[6] *= _180_OVER_PI;
        state_vector[7] *= _180_OVER_PI;
        state_vector[8] *= _180_OVER_PI;

        geometry_msgs::Vector3 current_orientation;
        current_orientation.x = state_vector[6];
        current_orientation.y = state_vector[7];
        current_orientation.z = state_vector[8];

        /*
         * Update orientation derivatives.
         */
        previous_quaternion_msgs.push_front(current_orientation);
        previous_quaternion_msgs_times.push_front(quat_msg->header.stamp);

        if (previous_quaternion_msgs.size() > 5)
        {
            previous_quaternion_msgs.pop_back();
            previous_quaternion_msgs_times.pop_back();
        }

        /*
         * The derivatives can be affected by discontinuities in the
         * wrapping of orientations. This section will un-wrap an
         * orientation if the derivative is too high. This should work okay
         * because the derivative of the sub can not change instantaneously
         * in water. Construct a copy of the measurement vector so that the
         * original can be maintained.
         */
        std::deque<geometry_msgs::Vector3> previous_orientation_msgs(
                previous_quaternion_msgs);
        for (unsigned int i = 0; i < previous_orientation_msgs.size() - 1; ++i)
        {
            double time_step = (previous_quaternion_msgs_times[i] -
                    previous_quaternion_msgs_times[i+1]).toSec();
            double roll_change = previous_orientation_msgs[i].x -
                    previous_orientation_msgs[i+1].x;
            double pitch_change = previous_orientation_msgs[i].y -
                    previous_orientation_msgs[i+1].y;
            double yaw_change = previous_orientation_msgs[i].z -
                    previous_orientation_msgs[i+1].z;

            double dyaw = yaw_change / time_step;
            double dpitch = pitch_change / time_step;
            double droll = roll_change / time_step;

            /*
             * The wrap threshold will be set at 100 degrees in a single
             * 20Hz cycle. This should not be possible under any condition.
             */
            const double d_threshold = 10.0 / (1.0/20.0);

            /*
             * A spike from 180 to -180 indicates the derivative is
             * actually positive for yaw and pitch. On the inverse occurance,
             * the derivative is actually negative. Always adjust following
             * readings to be normalized with the first reading in the queue.
             */
            if (dyaw > d_threshold)
            {
                previous_orientation_msgs[i+1].z += 360;
                ROS_INFO_STREAM("Wrapping yaw up.");
            }
            else if (dyaw < -1*d_threshold)
            {
                previous_orientation_msgs[i+1].z -= 360;
                ROS_INFO_STREAM("Wrapping yaw down.");
            }

            /*
             * Roll is bounded by the same characteristics as yaw. Wrap
             * similarly.
             */
            if (droll > d_threshold)
            {
                previous_orientation_msgs[i+1].x += 360;
                ROS_INFO_STREAM("Wrapping roll up.");
            }
            else if (droll < -1*d_threshold)
            {
                previous_orientation_msgs[i+1].x -= 360;
                ROS_INFO_STREAM("Wrapping roll down.");
            }

            /*
             * Pitch is bounded by +/- 90 degrees. Wrap with values of 180
             * instead of 360.
             */
            if (dpitch > d_threshold)
            {
                previous_orientation_msgs[i+1].y += 180;
                ROS_INFO_STREAM("Wrapping pitch up.");
            }
            else if (dpitch < -1*d_threshold)
            {
                previous_orientation_msgs[i+1].y -= 180;
                ROS_INFO_STREAM("Wrapping pitch down.");
            }
        }

        /*
         * To prevent a continuous increase in average yaw, pitch, or roll
         * values by continuous unwrapping in the same direction, the average
         * value of each reading will be found and subtracted from each element
         * to normalize the points around y = 0.
         */
        int number_readings = previous_orientation_msgs.size();
        double total_yaw = 0, total_pitch = 0, total_roll = 0;
        for (int i = 0; i < number_readings; ++i)
        {
            total_yaw += previous_orientation_msgs[i].z;
            total_pitch += previous_orientation_msgs[i].y;
            total_roll += previous_orientation_msgs[i].x;
        }

        double avg_yaw = total_yaw / number_readings, avg_pitch = total_pitch /
                number_readings, avg_roll = total_roll / number_readings;

        for (int i = 0; i < number_readings; ++i)
        {
            previous_orientation_msgs[i].z -= avg_yaw;
            previous_orientation_msgs[i].y -= avg_pitch;
            previous_orientation_msgs[i].x -= avg_roll;
        }

        /*
         * Calculate the derivatives of yaw, pitch, and roll using linear
         * regression. Note that all time is relative to the earliest sampled
         * time. The following linear regression mode is used:
         *     Slope = (n(Sum(x*y)) - Sum(x)*Sum(y)) / (n(Sum(x^2) - Sum(x)^2))
         * Where y is the yaw, pitch, or roll measurement and x is the time.
         */
        double sum_yaw = 0, sum_pitch = 0, sum_roll = 0, sum_time = 0,
               sum_time_sq = 0;
        double sum_yaw_time = 0, sum_pitch_time = 0, sum_roll_time = 0;
        for (int i = 0; i < number_readings; ++i)
        {
            double current_time = (previous_quaternion_msgs_times[i] -
                    previous_quaternion_msgs_times[
                    previous_orientation_msgs.size() - 1]).toSec();

            sum_roll += previous_orientation_msgs[i].x;
            sum_pitch += previous_orientation_msgs[i].y;
            sum_yaw += previous_orientation_msgs[i].z;

            sum_roll_time += previous_orientation_msgs[i].x *
                    current_time;
            sum_pitch_time += previous_orientation_msgs[i].y *
                    current_time;
            sum_yaw_time += previous_orientation_msgs[i].z *
                    current_time;

            sum_time += current_time;
            sum_time_sq += current_time * current_time;
        }

        /*
         * Finally, calculate the slope of the linear regression as the
         * derivative.
         */
        state_vector[9] = (number_readings * sum_roll_time - sum_time *
                sum_roll) / (number_readings * sum_time_sq - sum_time *
                sum_time);
        state_vector[10] = (number_readings * sum_pitch_time - sum_time *
                sum_pitch) / (number_readings * sum_time_sq - sum_time *
                sum_time);
        state_vector[11] = (number_readings * sum_yaw_time - sum_time *
                sum_yaw) / (number_readings * sum_time_sq - sum_time *
                sum_time);
    }

    /**
     * Update the depth of the submarine.
     *
     * @brief Updates the current state vector with an updated depth reading.
     *
     * @return None.
     */
    void ControlSystem::InputDepthMessage(const
            robosub::depth_stamped::ConstPtr& depth_msg)
    {
        /*
         * Update the X, Y, and Z positions of the state vector. Note that X
         * and Y position are currently unknown and always set to zero.
         */
        state_vector[2] = depth_msg->depth;

        previous_depth_msgs_times.push_front(depth_msg->header.stamp);
        previous_depth_msgs.push_front(depth_msg->depth);

        if (previous_depth_msgs.size() > 5)
        {
            previous_depth_msgs_times.pop_back();
            previous_depth_msgs.pop_back();
        }

        /*
         * Calculate the derivative of depth using linear regression. Note that
         * all time is relative to the earliest sampled time. The following
         * linear regression mode is used:
         *     Slope = (n(Sum(x*y)) - Sum(x)*Sum(y)) / (n(Sum(x^2) - Sum(x)^2))
         * Where y is the depth measurement and x is the time.
         */
        double sum_time = 0, sum_time_sq = 0, sum_depth_time = 0,
               sum_depth = 0;
        int number_readings = previous_depth_msgs.size();
        for (int i = 0; i < number_readings; ++i)
        {
            ros::Duration current_duration = previous_depth_msgs_times[i] -
                    previous_depth_msgs_times[previous_depth_msgs_times.size()
                    - 1];
            double current_time = current_duration.toSec();

            sum_time += current_time;
            sum_time_sq += current_time * current_time;
            sum_depth += previous_depth_msgs[i];
            sum_depth_time += previous_depth_msgs[i] * current_time;
        }

        state_vector[5] = (number_readings * sum_depth_time - sum_time *
                sum_depth) / ( number_readings * sum_time_sq - sum_time *
                sum_time);
    }

    /**
     * Calculates a thruster control signal with the current and desired state.
     *
     * @return A thruster packet calculated for the latest control state.
     */
    robosub::thruster ControlSystem::CalculateThrusterMessage()
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
                r3D(state_vector.segment<3>(6)).transpose() *
                r3D(rotation_goals));

        /*
         * Update the current error vector with the calculated errors.
         */
        current_error = Vector6d::Zero();
        current_error << translation_error, rotation_error;

        /*
         * Update and bound-check the integral terms.
         */
        current_integral += current_error * dt;
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

        previous_error.push_front(current_error);
        previous_error_times.push_front(ros::Time::now());

        if (previous_error.size() > 5)
        {
            previous_error.pop_back();
            previous_error_times.pop_back();
        }

        /*
         * Calculate the derivative of the error using linear regression.
         */
        Vector6d sum_error = Vector6d::Zero(),
                 sum_error_time = Vector6d::Zero();
        double sum_time = 0, sum_time_sq = 0;
        int number_elements = previous_error.size();
        for (int i = 0; i < number_elements; ++i)
        {
            double current_time = (previous_error_times[i] -
                    previous_error_times[number_elements - 1]).toSec();
            sum_time += current_time;
            sum_time_sq += current_time * current_time;
            sum_error += previous_error[i];
            sum_error_time += current_time * previous_error[i];
        }

        current_derivative = (number_elements * sum_error_time - sum_time *
                sum_error) / (number_elements * sum_time_sq - sum_time *
                sum_time);

        m_accel += P.cwiseProduct(current_error).cwiseProduct(hist);
        m_accel += I.cwiseProduct(current_integral);
        m_accel += D.cwiseProduct(current_derivative);

        /*
         * Convert accelerations to force by multipling by masses.
         */
        Vector6d m_force = m_accel.cwiseProduct(sub_mass);

        /*
         * Grab the current orientation of the submarine for rotating the
         * current translational goals. The order of this vector is roll,
         * pitch, and yaw.  Note that by setting the yaw to zero, all control
         * signals are relative.
         */
        Vector3d current_orientation;
        current_orientation[0] = state_vector[6];
        current_orientation[1] = state_vector[7];
        current_orientation[2] = 0;

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
         * control for each thruster. Scale any reverse directions by the
         * backward thruster ratio to achieve our desired backward thrust goal.
         */
        total_control = translation_control + rotation_control;
        for (int i = 0; i < num_thrusters; ++i)
        {
            if(total_control[i] < 0)
                total_control[i] /= back_thrust_ratio;
        }

        /*
         * Create a new thruster control message based upon the newly
         * calculated total_control vector.
         */
        robosub::thruster thruster_message;

        for (int i = 0; i < num_thrusters; ++i)
        {
            thruster_message.data.push_back(total_control[i]);
        }

        return thruster_message;
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
            case robosub::control::STATE_NONE:
                ret = "No State";
                break;
            case robosub::control::STATE_ERROR:
                ret = "Error State";
                break;
            case robosub::control::STATE_ABSOLUTE:
                ret = "Absolute State";
                break;
            case robosub::control::STATE_RELATIVE:
                ret = "Relative State";
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
}
