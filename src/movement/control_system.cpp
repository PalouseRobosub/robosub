#include "control_system.hpp"
#include "utility/math.hpp"



ControlSystem::ControlSystem(ros::NodeHandle *_nh, ros::Publisher *_pub)
{
    nh = new ros::NodeHandle("control");
    pub = _pub;

    ReloadPIDParams();

    // mass
    nh->getParamCached("mass", sub_mass[0]);
    nh->getParamCached("mass", sub_mass[1]);
    nh->getParamCached("mass", sub_mass[2]);

    // inertia
    nh->getParamCached("inertia/psi", sub_mass[3]);
    nh->getParamCached("inertia/phi", sub_mass[4]);
    nh->getParamCached("inertia/theta", sub_mass[5]);

    //ratio of backwards/fowards thruster strength
    nh->getParamCached("back_thrust_ratio", back_thrust_ratio);

    //thruster limits
    double limit_translation;
    double limit_rotation;
    nh->getParamCached("limits/translation", limit_translation);
    nh->getParamCached("limits/rotation", limit_rotation);
    t_lim = back_thrust_ratio * limit_translation;
    r_lim = back_thrust_ratio * limit_rotation;

    //max thrust
    nh->getParamCached("max_thrust", max_thrust);

    //offsets
    nh->getParamCached("control_offsets/x", offsets(0));
    nh->getParamCached("control_offsets/y", offsets(1));
    nh->getParamCached("control_offsets/z", offsets(2));
    nh->getParamCached("control_offsets/psi", offsets(3));
    nh->getParamCached("control_offsets/phi", offsets(4));
    nh->getParamCached("control_offsets/theta", offsets(5));

    // Thruster settings. Don't have nodehandle ref so must use ros::param::get
    XmlRpc::XmlRpcValue thruster_settings;

    if(!ros::param::get("thrusters", thruster_settings))
    {
        ROS_FATAL("thruster params failed to load");
        exit(1);
    }

    ROS_INFO_STREAM("Loaded " << thruster_settings.size() << " thrusters");

    num_thrusters = 0;
    position = MatrixXd(1,3);
    orientation = MatrixXd(1,3);
    for(int i=0; i < thruster_settings.size(); ++i)
    {
        position.conservativeResize(num_thrusters+1, NoChange_t());
        orientation.conservativeResize(num_thrusters+1, NoChange_t());
        string thruster_name = thruster_settings[i]["name"];


        position(num_thrusters,0) = thruster_settings[i]["position"]["x"];
        position(num_thrusters,1) = thruster_settings[i]["position"]["y"];
        position(num_thrusters,2) = thruster_settings[i]["position"]["z"];
        orientation(num_thrusters,0) = thruster_settings[i]["orientation"]["x"];
        orientation(num_thrusters,1) = thruster_settings[i]["orientation"]["y"];
        orientation(num_thrusters,2) = thruster_settings[i]["orientation"]["z"];

        ++num_thrusters;
    }

    int rate;
    nh->getParamCached("rate", rate);
    this->dt = 1.0/(double)rate;

    // calcs
    Matrix3d inertia_mat = Matrix3d::Zero();
    inertia_mat(0,0) = 1/sub_mass[3];
    inertia_mat(1,1) = 1/sub_mass[4];
    inertia_mat(2,2) = -1/sub_mass[5];

    motors = MatrixXd(6,num_thrusters);

    for (int i=0; i < num_thrusters; ++i)
    {
        motors.block<3,1>(0,i) = max_thrust * orientation.block<1,3>(i,0).transpose();
        motors.block<3,1>(3,i) = max_thrust*position.block<1,3>(i,0).cross(orientation.block<1,3>(i,0)).transpose();
    }

    //INFO("motor matrix:"); log << motors << endl;
    ROS_INFO_STREAM("motor_matrix:\n" << motors);
    ROS_INFO_STREAM("orientation:\n" << orientation);
    ROS_INFO_STREAM("position:\n" << position);

    if(motors.rows() == motors.cols())
    {
        M = motors.inverse();
    }
    else
    {
        M = pinv(motors);
    }

    state_vector = Vector12d::Zero();
    motor_commands = VectorXd::Zero(num_thrusters);
    integral_state = Vector6d::Zero();
    goals = Vector6d::Zero();
    for(int i=0; i < 6; ++i)
        goal_types[i] = robosub::control::STATE_ERROR;

    // Do these need initialization?
    //prev_quat_msg = 0;
    //prev_depth_msg = 0;
}

void ControlSystem::ReloadPIDParams()
{
    // proportional
    nh->getParamCached("proportional/x", P[0]);
    nh->getParamCached("proportional/y", P[1]);
    nh->getParamCached("proportional/z", P[2]);
    nh->getParamCached("proportional/psi", P[3]);
    nh->getParamCached("proportional/phi", P[4]);
    nh->getParamCached("proportional/theta", P[5]);

    // integral
    nh->getParamCached("integral/x", I[0]);
    nh->getParamCached("integral/y", I[1]);
    nh->getParamCached("integral/z", I[2]);
    nh->getParamCached("integral/psi", I[3]);
    nh->getParamCached("integral/phi", I[4]);
    nh->getParamCached("integral/theta", I[5]);

    // derivative
    nh->getParamCached("derivative/x", D[0]);
    nh->getParamCached("derivative/y", D[1]);
    nh->getParamCached("derivative/z", D[2]);
    nh->getParamCached("derivative/psi", D[3]);
    nh->getParamCached("derivative/phi", D[4]);
    nh->getParamCached("derivative/theta", D[5]);

    // windup
    nh->getParamCached("windup/x", windup[0]);
    nh->getParamCached("windup/y", windup[1]);
    nh->getParamCached("windup/z", windup[2]);
    nh->getParamCached("windup/psi", windup[3]);
    nh->getParamCached("windup/phi", windup[4]);
    nh->getParamCached("windup/theta", windup[5]);

    // hysteresis
    nh->getParamCached("hysteresis/x", hysteresis[0]);
    nh->getParamCached("hysteresis/y", hysteresis[1]);
    nh->getParamCached("hysteresis/z", hysteresis[2]);
    nh->getParamCached("hysteresis/psi", hysteresis[3]);
    nh->getParamCached("hysteresis/phi", hysteresis[4]);
    nh->getParamCached("hysteresis/theta", hysteresis[5]);
}

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
            case robosub::control::STATE_RELATIVE:
                this->goal_types[i] = robosub::control::STATE_ABSOLUTE;
                //calculate new absolute goal from current state plus control value
                if(i < 3)
                    goals[i] = state_vector[i] + control_values[i];
                // Roll: -180 - 180
                else if (i == 3)
                    goals[i] = wraparound(state_vector[i+3] + control_values[i], -180.0 * _PI_OVER_180, 180.0 * _PI_OVER_180);
                // Pitch: -90 - 90
                else if (i == 4)
                    goals[i] = wraparound(state_vector[i+3] + control_values[i], -90.0 * _PI_OVER_180, 90.0 * _PI_OVER_180);
                // Yaw: -180 - 180
                else if (i == 5)
                    goals[i] = wraparound(state_vector[i+3] + control_values[i], -180.0 * _PI_OVER_180, 180.0 * _PI_OVER_180);
                break;
            case robosub::control::STATE_ERROR:
                this->goal_types[i] = robosub::control::STATE_ERROR;
                this->goals[i] = control_values[i];
                break;
            case robosub::control::STATE_NONE: //don't update anything
                break;
            default:
                ROS_ERROR("received invalid control state");
                break;
        }
    }

    //CalculateThrusterMessage();
}

void ControlSystem::InputOrientationMessage(geometry_msgs::QuaternionStamped quat_msg)
{
    // Quaternion to roll pitch yaw
    // This is apparently the best way to do it with built in ros stuff
    tf::Quaternion q(quat_msg.quaternion.x, quat_msg.quaternion.y, quat_msg.quaternion.z, quat_msg.quaternion.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;

    //return yaw,pitch,roll in radians
    m.getRPY(roll, pitch, yaw);

    state_vector[6] = roll;
    state_vector[7] = pitch;
    state_vector[8] = yaw;

    // These are unused currently.
    /*
    state_vector[9] = _PI_OVER_180 * sp.droll; //roll'
    state_vector[10] = _PI_OVER_180 * sp.dpitch; //pitch'
    state_vector[11] = _PI_OVER_180 * sp.dyaw; //yaw'
    */
}
void ControlSystem::InputDepthMessage(robosub::depth_stamped depth_msg)
{
    state_vector[0] = 0; //x
    state_vector[1] = 0; //y
    state_vector[2] = depth_msg.depth; //z

    state_vector[3] = 0; //x'
    state_vector[4] = 0; //y'
    state_vector[5] = prev_depth_msg.depth - depth_msg.depth; //z'

    prev_depth_msg.depth = depth_msg.depth;

}

void ControlSystem::CalculateThrusterMessage()
{
    ReloadPIDParams();

    ROS_DEBUG_STREAM("state:\n" << state_vector);
    //control system shit
    motor_commands = motor_control(state_vector);

    //convert motor_commands eigen vector to stl vector
    std::vector<double> motor_commands_stl_vec(motor_commands.data(), motor_commands.data() + motor_commands.size());

    tp.data.clear();
    std::stringstream s;
    for(int i=0; i<motor_commands_stl_vec.size(); ++i)
    {
        s >> motor_commands_stl_vec[i];
        tp.data.push_back(motor_commands_stl_vec[i]);
    }
    //ROS_INFO_STREAM(s);
    //std::cout << s << std::endl;
}

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
void ControlSystem::PublishThrusterMessage()
{
    pub->publish(tp);
}

/*
string ControlSystem::statestring(control_packet::State state)
{
    string s;
    switch (state)
    {
        case robosub::control::STATE_NONE:     s = "NONE"; break;
        case robosub::control::STATE_ABSOLUTE: s = "ABSOLUTE"; break;
        case robosub::control::STATE_RELATIVE: s = "RELATIVE"; break;
        case robosub::control::STATE_ERROR:    s = "ERROR"; break;
        default: s = "UNKNOWN"; break;
    }

    return s;
}
*/

VectorXd ControlSystem::motor_control(Vector12d state)
{
    //INFO("running motor control function");
    //calcluate errors
    Vector3d err_translation = goals.segment<3>(0) - state.segment<3>(0);
    for(int i=0; i < 3; ++i)
    {
        //check if we should override the error calculation
        if(goal_types[i] == robosub::control::STATE_ERROR)
        {
            err_translation[i] = goals[i];
            //integral_state[i+3] = 0.0; 		//ccheck +3? is translation latter 3 terms?
            integral_state[i] = 0.0;		//fix seems correct
        }
    }

    Vector3d rotation_goals;
    for(int i=0; i < 3; ++i)
    {
        if(goal_types[i+3] == robosub::control::STATE_ERROR)
        {
            rotation_goals[i] = state_vector[i+6] + goals[i+3];
            //integral_state[i] = 0.0;
            integral_state[i+3] = 0.0;
        }
        else
        {
            rotation_goals[i] = goals[i+3];
        }
    }
    //INFO("cs: rotation_gls:"); log << rotation_goals << endl;
//		Vector3d R = ir3D( r3D(rotation_goals) * r3D(state.segment<3>(6)).transpose() );
    Vector3d R = ir3D( r3D(state.segment<3>(6)).transpose() * r3D(rotation_goals) ); //ccheck wrong order r3D' * r3D
    Vector3d err_rotation = R;
    //log << r3D(rotation_goals) << endl;
    //log << r3D(state.segment<3>(6)).transpose() << endl;
    //if roll and yaw are over 90 degrees either way,
    //a more efficient route exists

    //ccheck cos theta * pitch FIRST, before supplemental angle
    err_rotation(1) = err_rotation(1) * cos(err_rotation(2));

    //INFO("cs: R:"); log << R << endl;
/*											//ccheck is this the problem?
    if( (fabs(R(1)) > _PI_OVER_2) && (fabs(R(2)) > _PI_OVER_2) )
    {
        err_rotation(0) = Sgn(R(0))*_PI - R(0); //new roll is supplement angle
        err_rotation(2) = R(2) - Sgn(R(2))*_PI; //new yaw is supplement angle
        //pitch is unchanged
    }
*/
//		err_rotation.segment<2>(0) = err_rotation.segment<2>(0) * cos(err_rotation(2)); 	//ccheck both pitch and roll cos(theta)


    Vector6d error;
    error << err_translation, err_rotation;

    //INFO("cs: error:"); log << error << endl;

    //do PID for all the the motors
    //update error integral
    integral_state += error * dt; //where should dt come from?
    for(int i=0; i < 6; ++i)  //check for integral windup
    {
        if(fabs(integral_state(i)) > fabs(windup(i)))
            integral_state(i) = windup(i) * rs::math::sign(integral_state(i));
    }

    //perform hysteresis computation
    Vector6d hist;
    hist = (error.array().abs() >= hysteresis.array()).cast<double>();
    //INFO("cs: hys"); log << hist << endl;

    //use PID to compute the corrective actions
    Vector6d m_accel = Vector6d::Zero();
    m_accel = P.cwiseProduct(error).cwiseProduct(hist); //proportional
    //INFO("cs: m_accel1:"); log << m_accel << endl;

    Vector6d tmp6v = Vector6d::Zero(); //setup custom state vector for derivative
    tmp6v.segment<3>(0) = state.segment<3>(3);
    tmp6v.segment<3>(3) = state.segment<3>(9);
    m_accel += D.cwiseProduct(tmp6v); //derivative
    m_accel += I.cwiseProduct(integral_state); //integral
    m_accel += offsets; //add offset

//		INFO("cs: m_accel2:"); log << m_accel << endl;


    Vector6d m_force = m_accel.cwiseProduct(sub_mass); //convert to force
    //INFO("cs: m_force:"); log << m_force << endl;

    Vector3d tmp3v;
    tmp3v[0] = state[6];
    tmp3v[1] = state[7];
    tmp3v[2] = 0; 			//setting yaw=0 means x,y commands become forward/strafe commands

    tmp6v = Vector6d::Zero();
    tmp6v.segment<3>(0) = r3D(tmp3v).transpose() * m_force.segment<3>(0);  //uncomment this line to vectorize force
    //tmp6v.segment<3>(0) = m_force.segment<3>(0);
    VectorXd m_ctrl_t = M * tmp6v;
    //INFO("cs: tmp6v for translation:"); log << tmp6v << endl;

    tmp6v = Vector6d::Zero();
    tmp6v.segment<3>(3) = m_force.segment<3>(3);
    //INFO("cs: tmp6v for rotation:"); log << tmp6v << endl;
    VectorXd m_ctrl_r = M * tmp6v;
    //INFO("cs: Motor Matrix:"); log << M.inverse() << endl;


    //scale the motors down to limits --needs to be more intelligent: seperate  m_accel PID from offset, divide m_accel goals only
    for (int i=0; i < num_thrusters; ++i)
    {
        if(fabs(m_ctrl_t(i)) > t_lim)
            m_ctrl_t(i) = t_lim*rs::math::sign(m_ctrl_t(i));
        if(fabs(m_ctrl_r(i)) > r_lim)
            m_ctrl_r(i) = r_lim*rs::math::sign(m_ctrl_r(i));
    }

    VectorXd m_ctrl_sum = m_ctrl_t + m_ctrl_r;

    for (int i=0; i < num_thrusters; ++i)
    {
        if(m_ctrl_sum[i] < 0)
            m_ctrl_sum[i] /= back_thrust_ratio;
    }

	//INFO("cs: m_control_output:"); log << m_ctrl_sum << endl;
    return m_ctrl_sum;
}

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
