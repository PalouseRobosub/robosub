#include "control_system.hpp"

ControlSystem::ControlSystem(ros::NodeHandle *_nh, ros::Publisher *_pub)
{
    nh = _nh;
    pub = _pub;

    // mass
    nh->getParam("mass", sub_mass[0]);
    nh->getParam("mass", sub_mass[1]);
    nh->getParam("mass", sub_mass[2]);

    // inertia
    nh->getParam("inertia/psi", sub_mass[3]);
    nh->getParam("inertia/phi", sub_mass[4]);
    nh->getParam("inertia/theta", sub_mass[5]);

    // proportional
    nh->getParam("proportional/x", P[0]);
    nh->getParam("proportional/y", P[1]);
    nh->getParam("proportional/z", P[2]);
    nh->getParam("proportional/psi", P[3]);
    nh->getParam("proportional/phi", P[4]);
    nh->getParam("proportional/theta", P[5]);

    // integral
    nh->getParam("integral/x", I[0]);
    nh->getParam("integral/y", I[1]);
    nh->getParam("integral/z", I[2]);
    nh->getParam("integral/psi", I[3]);
    nh->getParam("integral/phi", I[4]);
    nh->getParam("integral/theta", I[5]);

    // derivative
    nh->getParam("derivative/x", D[0]);
    nh->getParam("derivative/y", D[1]);
    nh->getParam("derivative/z", D[2]);
    nh->getParam("derivative/psi", D[3]);
    nh->getParam("derivative/phi", D[4]);
    nh->getParam("derivative/theta", D[5]);

    // windup
    nh->getParam("windup/x", windup[0]);
    nh->getParam("windup/y", windup[1]);
    nh->getParam("windup/z", windup[2]);
    nh->getParam("windup/psi", windup[3]);
    nh->getParam("windup/phi", windup[4]);
    nh->getParam("windup/theta", windup[5]);

    // hysteresis
    nh->getParam("hysteresis/x", hysteresis[0]);
    nh->getParam("hysteresis/y", hysteresis[1]);
    nh->getParam("hysteresis/z", hysteresis[2]);
    nh->getParam("hysteresis/psi", hysteresis[3]);
    nh->getParam("hysteresis/phi", hysteresis[4]);
    nh->getParam("hysteresis/theta", hysteresis[5]);

    //ratio of backwards/fowards thruster strength
    nh->getParam("back_thrust_ratio", back_thrust_ratio);

    //thruster limits
    double limit_translation;
    double limit_rotation;
    nh->getParam("limits/translation", limit_translation);
    nh->getParam("limits/rotation", limit_rotation);
    t_lim = back_thrust_ratio * limit_translation;
    r_lim = back_thrust_ratio * limit_rotation;

    //max thrust
    nh->getParam("max_thrust", max_thrust);

    //offsets
    nh->getParam("control_offsets/x", offsets(0));
    nh->getParam("control_offsets/y", offsets(1));
    nh->getParam("control_offsets/z", offsets(2));
    nh->getParam("control_offsets/psi", offsets(3));
    nh->getParam("control_offsets/phi", offsets(4));
    nh->getParam("control_offsets/theta", offsets(5));

    // Thruster settings. Don't have nodehandle ref so must use ros::param::get
    XmlRpc::XmlRpcValue thruster_settings;

    if(ros::param::get("/thruster/", thruster_settings))
    {
        std::cout << "success" << std::endl;
    }
    else
    {
        std::cout << "failed" << std::endl;
    }

    num_thrusters = 0;
    position = MatrixXd(1,3);
    orientation = MatrixXd(1,3);
    for(XmlRpc::XmlRpcValue::iterator it = thruster_settings["thrusters"].begin();
            it != thruster_settings["thrusters"].end(); ++it)
    {
        std::cout << it->first << ":" << it->second << std::endl;

        position.conservativeResize(num_thrusters+1, NoChange_t());
        orientation.conservativeResize(num_thrusters+1, NoChange_t());
        string thruster_name = (it->first);

        XmlRpc::XmlRpcValue t = thruster_settings["thrusters"]["it->first"];
        position(num_thrusters,0) = t["position"]["x"];
        position(num_thrusters,1) = t["position"]["y"];
        position(num_thrusters,2) = t["position"]["z"];
        orientation(num_thrusters,0) = t["orientation"]["x"];
        orientation(num_thrusters,1) = t["orientation"]["y"];
        orientation(num_thrusters,2) = t["orientation"]["z"];

        ++num_thrusters;
    }


    // calcs
    Matrix3d inertia_mat = Matrix3d::Zero();
    inertia_mat(0,0) = 1/sub_mass[3];
    inertia_mat(1,1) = 1/sub_mass[4];
    inertia_mat(2,2) = -1/sub_mass[5];
    int rate;
    nh->getParam("rate", rate);
    this->dt = 1.0/(double)rate;

    motors = MatrixXd(6,num_thrusters);

    for (int i=0; i < num_thrusters; ++i)
    {
        motors.block<3,1>(0,i) = max_thrust * orientation.block<1,3>(i,0).transpose();
        motors.block<3,1>(3,i) = max_thrust*position.block<1,3>(i,0).cross(orientation.block<1,3>(i,0)).transpose();
    }

    //INFO("motor matrix:"); log << motors << endl;

    if(motors.rows() == motors.cols())
    {
        M = motors.inverse();
    }
    else
    {
        M = pinv(motors);
    }

    //INFO("M:"); log << M << endl;

    state_vector = Vector12d::Zero();
    motor_commands = VectorXd::Zero(num_thrusters);
    integral_state = Vector6d::Zero();
    goals = Vector6d::Zero();
    for(int i=0; i < 6; ++i)
        goal_types[i] = robosub::control::STATE_ERROR;
}

void ControlSystem::InputControlMessage(robosub::control msg)
{
    //INFO("got new control packet");
    Vector6d control_values;
    Matrix<uint8_t, 6, 1> control_states;

    control_states[0] = msg.forward_state;
    control_states[1] = msg.strafe_state;
    control_states[2] = msg.dive_state;
    control_states[3] = msg.roll_state;
    control_states[4] = msg.pitch_state;
    control_states[5] = msg.yaw_state;

    control_values[0] = msg.forward;
    control_values[1] = msg.strafe_right;
    control_values[2] = msg.dive;
    control_values[3] = msg.roll_right * _PI_OVER_180;
    control_values[4] = msg.pitch_up * _PI_OVER_180;
    control_values[5] = msg.yaw_right * _PI_OVER_180;

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

    //control system shit
    //only update pid when new motor commands
    //motor_commands = motor_control(state_vector,fps);

    update();
}

void ControlSystem::InputOrientationMessage(geometry_msgs::Quaternion msg)
{
    // TODO: Make sensor packet?
    /*
    //INFO("got new sensor packet");
    sensor_packet sp(m.value);
    state_vector[0] = 0; //x
    state_vector[1] = 0; //y
    state_vector[2] = sp.depth; //z

    state_vector[3] = 0; //x'
    state_vector[4] = 0; //y'
    state_vector[5] = sp.ddepth; //z'

    state_vector[6] = _PI_OVER_180 * sp.roll; //roll
    state_vector[7] = _PI_OVER_180 * sp.pitch; //pitch
    state_vector[8] = _PI_OVER_180 * sp.yaw; //yaw

    state_vector[9] = _PI_OVER_180 * sp.droll; //roll'
    state_vector[10] = _PI_OVER_180 * sp.dpitch; //pitch'
    state_vector[11] = _PI_OVER_180 * sp.dyaw; //yaw'

    //control system shit
    INFO("state_vector:"); log << state_vector << endl;
    motor_commands = motor_control(state_vector);

    */
    update();
}

void ControlSystem::update()
{
    /*
    // grab messages
    vector<string> msgs = com->receive_messages();
    for(string msg : msgs) //is this guarenteed to only get 1 control and/or 1 sensor?
    {
        message m(msg);
        if(m.mtype == "control")
        {
        }
        if(m.mtype == "sensor")
        {
        }

        if (m.mtype == "control_config") //This message will reconfigure the control parameters
        {
            control_configuration_packet ccp(m.value);

            if (ccp.mode == control_configuration_packet::RESET) //If this is an integral reset message
            {
                switch (ccp.axes)
                {
                    case control_configuration_packet::X:
                        integral_state[0] = 0;
                        break;
                    case control_configuration_packet::Y:
                        integral_state[1] = 0;
                        break;
                    case control_configuration_packet::Z:
                        integral_state[2] = 0;
                        break;
                    case control_configuration_packet::PSI:
                        integral_state[3] = 0;
                        break;
                    case control_configuration_packet::PHI:
                        integral_state[4] = 0;
                        break;
                    case control_configuration_packet::THETA:
                        integral_state[5] = 0;
                        break;
                }
            }
            else
            {
                int tmp = -1;
                //else the mode is CONFIG -> we should update PIDWH values
                switch (ccp.axes)
                {
                    case control_configuration_packet::X:
                        tmp = 0;
                        break;
                    case control_configuration_packet::Y:
                        tmp = 1;
                        break;
                    case control_configuration_packet::Z:
                        tmp = 2;
                        break;
                    case control_configuration_packet::PSI:
                        tmp = 3;
                        break;
                    case control_configuration_packet::PHI:
                        tmp = 4;
                        break;
                    case control_configuration_packet::THETA:
                        tmp = 5;
                        break;
                }
                if (tmp != -1)
                {
                    P[tmp] = ccp.p;
                    I[tmp] = ccp.i;
                    D[tmp] = ccp.d;
                    windup[tmp] = ccp.w;
                    hysteresis[tmp] = ccp.h;
                }
            }
        }
    }
    */

    // send thruster message
    //INFO("goals:"); log << goals << endl;
    //log << "goals: forward state: " << statestring(goal_types[0]) << " value: " << goals[0] << endl;
    //log << "goals: strafe state: " << statestring(goal_types[1]) << " value: " << goals[1] << endl;
    //log << "goals: dive state: " << statestring(goal_types[2]) << " value: " << goals[2] << endl;
    //log << "goals: roll state: " << statestring(goal_types[3]) << " value: " << goals[3] * (1.0 / _PI_OVER_180) << endl;
    //log << "goals: pitch state: " << statestring(goal_types[4]) << " value: " << goals[4] * (1.0 / _PI_OVER_180) << endl;
    //log << "goals: yaw state: " << statestring(goal_types[5]) << " value: " << goals[5] * (1.0 / _PI_OVER_180) << endl;

    /*
    // send a control_gui messages
    //States
    //INFO("Sending states to control GUI");
    control_packet control_gui_packet(goal_types[0], goal_types[1], goal_types[2],
            goal_types[3], goal_types[4], goal_types[5],
            goals[0], goals[1], goals[2],	goals[3], goals[4], goals[5]);
    com->send_message(message("control", "control_gui", "control_state", control_gui_packet.whole).whole);

    //Integrals
    //INFO("Sending Integrals to control GUI");
    integral_packet ip(integral_state[0], integral_state[1], integral_state[2], integral_state[3], integral_state[4], integral_state[5]);
    com->send_message(message("control", "control_gui", "integral", ip.whole).whole);

    //Configurations
    //INFO("Sending 6 configuration messages to controls GUI");
    for (int i = 0; i < 6; i++)
    {
        control_configuration_packet ccp(P[i], I[i], D[i], windup[i], hysteresis[i], (control_configuration_packet::Axes)i);
        com->send_message(message("control", "control_gui", "control_parameters", ccp.whole).whole);
    }
    */

    //convert motor_commands eigen vector to stl vector
    std::vector<double> motor_commands_stl_vec(motor_commands.data(), motor_commands.data() + motor_commands.size());

    robosub::thruster tp;
    tp.data = motor_commands_stl_vec;
    //thruster_packet tp(motor_commands_stl_vec);
    //com->send_message(message("control", "thruster", "thruster", tp.whole).whole);

    //INFO("motor_commands:"); log << motor_commands << endl;
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

//note that this function will need to be modified when
//we move to 8 thrusters
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
    if( (abs(R(1)) > _PI_OVER_2) && (abs(R(2)) > _PI_OVER_2) )
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
        if(abs(integral_state(i)) > abs(windup(i)))
            integral_state(i) = windup(i) * Sgn(integral_state(i));
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
        if(abs(m_ctrl_t(i)) > t_lim)
            m_ctrl_t(i) = t_lim*Sgn(m_ctrl_t(i));
        if(abs(m_ctrl_r(i)) > r_lim)
            m_ctrl_r(i) = r_lim*Sgn(m_ctrl_r(i));
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
