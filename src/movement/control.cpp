#include "ros/ros.h"
#include "robosub/thruster.h"
#include "robosub/control.h"
#include "geometry_msgs/Quaternion.h"
robosub::control current_control_commmand;
geometry_msgs::Quaternion current_orientation;

void controlCallback(const robosub::control::ConstPtr& msg)
{




}
void orientationCallback(const geometry_msgs::Quaternion::ConstPtr& msg)
{
	current_orientation = *msg;




}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "control");
	ros::NodeHandle n;
	ros::Subscriber controlsub = n.subscribe("control", 1, controlCallback);
	ros::Subscriber orientationsub = n.subscribe("orientation", 1, orientationCallback);
  	ros::Publisher pub = n.advertise<robosub::thruster>("thruster", 1);
	robosub::thruster thruster_command;
	int rate;
	n.getParam("control/rate", rate);
	ros::Rate r(rate);

	ROS_INFO("INITIALIZED");
	while (ros::ok())
	{

		pub.publish(thruster_command);

		ros::spinOnce();
		r.sleep();
	}


    return 0;
}
//#include "modules/ControlModule.hpp"
//
//
//namespace rs
//{
//	ControlModule::ControlModule()
//	{
//
//	}
//
//	void ControlModule::init()
//	{
//		try
//		{
//			// mass
//			sub_mass[0] = sub_mass[1] = sub_mass[2] = settings["mass"].GetDouble();
//
//			// inertia
//			sub_mass[3] = settings["inertia"]["psi"].GetDouble();
//			sub_mass[4] = settings["inertia"]["phi"].GetDouble();
//			sub_mass[5] = settings["inertia"]["theta"].GetDouble();
//
//			// proportional
//			P[0] = settings["P"]["x"].GetDouble();
//			P[1] = settings["P"]["y"].GetDouble();
//			P[2] = settings["P"]["z"].GetDouble();
//			P[3] = settings["P"]["psi"].GetDouble();
//			P[4] = settings["P"]["phi"].GetDouble();
//			P[5] = settings["P"]["theta"].GetDouble();
//
//			// integral
//			I[0] = settings["I"]["x"].GetDouble();
//			I[1] = settings["I"]["y"].GetDouble();
//			I[2] = settings["I"]["z"].GetDouble();
//			I[3] = settings["I"]["psi"].GetDouble();
//			I[4] = settings["I"]["phi"].GetDouble();
//			I[5] = settings["I"]["theta"].GetDouble();
//
//			// derivative
//			D[0] = settings["D"]["x"].GetDouble();
//			D[1] = settings["D"]["y"].GetDouble();
//			D[2] = settings["D"]["z"].GetDouble();
//			D[3] = settings["D"]["psi"].GetDouble();
//			D[4] = settings["D"]["phi"].GetDouble();
//			D[5] = settings["D"]["theta"].GetDouble();
//
//			// windup
//			windup[0] = settings["windup"]["x"].GetDouble();
//			windup[1] = settings["windup"]["y"].GetDouble();
//			windup[2] = settings["windup"]["z"].GetDouble();
//			windup[3] = settings["windup"]["psi"].GetDouble();
//			windup[4] = settings["windup"]["phi"].GetDouble();
//			windup[5] = settings["windup"]["theta"].GetDouble();
//
//			// hysteresis
//			hysteresis[0] = settings["hysteresis"]["x"].GetDouble();
//			hysteresis[1] = settings["hysteresis"]["y"].GetDouble();
//			hysteresis[2] = settings["hysteresis"]["z"].GetDouble();
//			hysteresis[3] = settings["hysteresis"]["psi"].GetDouble();
//			hysteresis[4] = settings["hysteresis"]["phi"].GetDouble();
//			hysteresis[5] = settings["hysteresis"]["theta"].GetDouble();
//
//
//			//ratio of backwards/fowards thruster strength
//			back_thrust_ratio = settings["back_thrust_ratio"].GetDouble();
//
//
//			//thruster limits
//			t_lim = back_thrust_ratio * settings["limits"]["translation"].GetDouble();
//			r_lim = back_thrust_ratio * settings["limits"]["rotation"].GetDouble();
//
//			//max thrust
//			max_thrust = settings["max_thrust"].GetDouble();
//
//
//
//			//offsets
//			offsets(0) = settings["control_offsets"]["x"].GetDouble();
//			offsets(1) = settings["control_offsets"]["y"].GetDouble();
//			offsets(2) = settings["control_offsets"]["z"].GetDouble();
//			offsets(3) = settings["control_offsets"]["psi"].GetDouble();
//			offsets(4) = settings["control_offsets"]["phi"].GetDouble();
//			offsets(5) = settings["control_offsets"]["theta"].GetDouble();
//
//			//read thruster settings file to load thrusters
//
//			string thruster_settings_file = "settings/modules/" +
//											string(settings["thruster_file"].GetString()) +
//											".json";
//			FILE *pFile = fopen(thruster_settings_file.c_str(), "r");
//			if(pFile == NULL)
//			{
//				EXIT("could not open: " + thruster_settings_file);
//			}
//			char buf[4096];
//			FileReadStream is(pFile, buf, sizeof(buf));
//			Document thruster_settings_doc;
//			if(thruster_settings_doc.ParseStream<0>(is).HasParseError())
//			{
//				EXIT("ParseStream had a parse error, exiting");
//			}
//			fclose(pFile);
//
//			Value thruster_settings; thruster_settings = thruster_settings_doc["thrusters"];
//
//			num_thrusters = 0;
//			position = MatrixXd(1,3);
//			orientation = MatrixXd(1,3);
//			for (auto iter = thruster_settings.MemberBegin(); iter != thruster_settings.MemberEnd(); ++iter)
//			{
//				position.conservativeResize(num_thrusters+1, NoChange_t());
//				orientation.conservativeResize(num_thrusters+1, NoChange_t());
//				string thruster_name = (iter->name).GetString();
//				Value thruster_dom; thruster_dom = thruster_settings[thruster_name.c_str()];
//				position(num_thrusters,0) = thruster_dom["position"]["x"].GetDouble();
//				position(num_thrusters,1) = thruster_dom["position"]["y"].GetDouble();
//				position(num_thrusters,2) = thruster_dom["position"]["z"].GetDouble();
//				orientation(num_thrusters,0) = thruster_dom["orientation"]["x"].GetDouble();
//				orientation(num_thrusters,1) = thruster_dom["orientation"]["y"].GetDouble();
//				orientation(num_thrusters,2) = thruster_dom["orientation"]["z"].GetDouble();
//
//				++num_thrusters;
//			}
//
//			INFO("loaded "); log << num_thrusters << " thrusters" << endl;
//
//			INFO("position:"); log << position << endl;
//
//
//			INFO("orientation:"); log << orientation << endl;
//
//
//			INFO("settings parse successful");
//
//		} catch(exception &e) {
//			EXIT("Control module init failed: " + string(e.what()));
//		}
//
//
//		// calcs
//		Matrix3d inertia_mat = Matrix3d::Zero();
//		inertia_mat(0,0) = 1/sub_mass[3];
//		inertia_mat(1,1) = 1/sub_mass[4];
//		inertia_mat(2,2) = -1/sub_mass[5];
//		this->dt = 1.0/this->fps;
//		motors = MatrixXd(6,num_thrusters);
//
//		for (int i=0; i < num_thrusters; ++i)
//		{
//			motors.block<3,1>(0,i) = max_thrust * orientation.block<1,3>(i,0).transpose();
//			motors.block<3,1>(3,i) = max_thrust*position.block<1,3>(i,0).cross(orientation.block<1,3>(i,0)).transpose();
//		}
//
//		INFO("motor matrix:"); log << motors << endl;
//
//		if(motors.rows() == motors.cols())
//		{
//			M = motors.inverse();
//		}
//		else
//		{
//			M = pinv(motors);
//		}
//
//		INFO("M:"); log << M << endl;
//
//		state_vector = Vector12d::Zero();
//		motor_commands = VectorXd::Zero(num_thrusters);
//		integral_state = Vector6d::Zero();
//		goals = Vector6d::Zero();
//		for(int i=0; i < 6; ++i)
//			goal_types[i] = control_packet::State::ERROR;
//
//	}
//
//	void ControlModule::update()
//	{
//		// grab messages
//		vector<string> msgs = com->receive_messages();
//		for(string msg : msgs) //is this guarenteed to only get 1 control and/or 1 sensor?
//		{
//			message m(msg);
//			if(m.mtype == "control")
//			{
//				//INFO("got new control packet");
//				control_packet cm(m.value);
//				Vector6d control_values;
//				Matrix<control_packet::State,6,1> control_states;
//
//
//				control_states[0] = cm.forward_s;
//				control_states[1] = cm.strafe_s;
//				control_states[2] = cm.dive_s;
//				control_states[3] = cm.roll_s;
//				control_states[4] = cm.pitch_s;
//				control_states[5] = cm.yaw_s;
//
//				control_values[0] = cm.forward;
//				control_values[1] = cm.strafe_right;
//				control_values[2] = cm.dive;
//				control_values[3] = cm.roll_right * _PI_OVER_180;
//				control_values[4] = cm.pitch_up * _PI_OVER_180;
//				control_values[5] = cm.yaw_right * _PI_OVER_180;
//
//				for(int i=0; i < 6; ++i)
//				{
//					switch(control_states[i])
//					{
//						case control_packet::State::ABSOLUTE:
//							this->goal_types[i] = control_packet::State::ABSOLUTE;
//							this->goals[i] = control_values[i];
//							break;
//						case control_packet::State::RELATIVE:
//							this->goal_types[i] = control_packet::State::ABSOLUTE;
//							//calculate new absolute goal from current state plus control value
//							if(i < 3)
//								goals[i] = state_vector[i] + control_values[i];
//							// Roll: -180 - 180
//							else if (i == 3)
//								goals[i] = wraparound(state_vector[i+3] + control_values[i], -180.0 * _PI_OVER_180, 180.0 * _PI_OVER_180);
//							// Pitch: -90 - 90
//							else if (i == 4)
//								goals[i] = wraparound(state_vector[i+3] + control_values[i], -90.0 * _PI_OVER_180, 90.0 * _PI_OVER_180);
//							// Yaw: -180 - 180
//							else if (i == 5)
//								goals[i] = wraparound(state_vector[i+3] + control_values[i], -180.0 * _PI_OVER_180, 180.0 * _PI_OVER_180);
//							break;
//						case control_packet::State::ERROR:
//							this->goal_types[i] = control_packet::State::ERROR;
//							this->goals[i] = control_values[i];
//							break;
//						case control_packet::State::NONE: //don't update anything
//								break;
//						default:
//							ERROR("received invalid control state");
//							break;
//					}
//				}
//
//								//control system shit
//				//only update pid when new motor commands
//				//motor_commands = motor_control(state_vector,fps);
//			}
//			if(m.mtype == "sensor")
//			{
//				//INFO("got new sensor packet");
//				sensor_packet sp(m.value);
//				state_vector[0] = 0; //x
//				state_vector[1] = 0; //y
//				state_vector[2] = sp.depth; //z
//
//				state_vector[3] = 0; //x'
//				state_vector[4] = 0; //y'
//				state_vector[5] = sp.ddepth; //z'
//
//				state_vector[6] = _PI_OVER_180 * sp.roll; //roll
//				state_vector[7] = _PI_OVER_180 * sp.pitch; //pitch
//				state_vector[8] = _PI_OVER_180 * sp.yaw; //yaw
//
//				state_vector[9] = _PI_OVER_180 * sp.droll; //roll'
//				state_vector[10] = _PI_OVER_180 * sp.dpitch; //pitch'
//				state_vector[11] = _PI_OVER_180 * sp.dyaw; //yaw'
//
//				//control system shit
//				INFO("state_vector:"); log << state_vector << endl;
//				motor_commands = motor_control(state_vector);
//			}
//
//			if (m.mtype == "control_config") //This message will reconfigure the control parameters
//			{
//				control_configuration_packet ccp(m.value);
//
//				if (ccp.mode == control_configuration_packet::RESET) //If this is an integral reset message
//				{
//					switch (ccp.axes)
//					{
//						case control_configuration_packet::X:
//							integral_state[0] = 0;
//							break;
//						case control_configuration_packet::Y:
//							integral_state[1] = 0;
//							break;
//						case control_configuration_packet::Z:
//							integral_state[2] = 0;
//							break;
//						case control_configuration_packet::PSI:
//							integral_state[3] = 0;
//							break;
//						case control_configuration_packet::PHI:
//							integral_state[4] = 0;
//							break;
//						case control_configuration_packet::THETA:
//							integral_state[5] = 0;
//							break;
//					}
//				}
//				else
//				{
//					int tmp = -1;
//					//else the mode is CONFIG -> we should update PIDWH values
//					switch (ccp.axes)
//					{
//						case control_configuration_packet::X:
//							tmp = 0;
//							break;
//						case control_configuration_packet::Y:
//							tmp = 1;
//							break;
//						case control_configuration_packet::Z:
//							tmp = 2;
//							break;
//						case control_configuration_packet::PSI:
//							tmp = 3;
//							break;
//						case control_configuration_packet::PHI:
//							tmp = 4;
//							break;
//						case control_configuration_packet::THETA:
//							tmp = 5;
//							break;
//					}
//					if (tmp != -1)
//					{
//						P[tmp] = ccp.p;
//						I[tmp] = ccp.i;
//						D[tmp] = ccp.d;
//						windup[tmp] = ccp.w;
//						hysteresis[tmp] = ccp.h;
//					}
//				}
//			}
//		}
//
//		// send thruster message
//		//INFO("goals:"); log << goals << endl;
//        log << "goals: forward state: " << statestring(goal_types[0]) << " value: " << goals[0] << endl;
//        log << "goals: strafe state: " << statestring(goal_types[1]) << " value: " << goals[1] << endl;
//        log << "goals: dive state: " << statestring(goal_types[2]) << " value: " << goals[2] << endl;
//        log << "goals: roll state: " << statestring(goal_types[3]) << " value: " << goals[3] * (1.0 / _PI_OVER_180) << endl;
//        log << "goals: pitch state: " << statestring(goal_types[4]) << " value: " << goals[4] * (1.0 / _PI_OVER_180) << endl;
//        log << "goals: yaw state: " << statestring(goal_types[5]) << " value: " << goals[5] * (1.0 / _PI_OVER_180) << endl;
//
//		// send a control_gui messages
//		//States
//		//INFO("Sending states to control GUI");
//		control_packet control_gui_packet(goal_types[0], goal_types[1], goal_types[2],
//										  goal_types[3], goal_types[4], goal_types[5],
//										  goals[0], goals[1], goals[2],	goals[3], goals[4], goals[5]);
//		com->send_message(message("control", "control_gui", "control_state", control_gui_packet.whole).whole);
//
//		//Integrals
//		//INFO("Sending Integrals to control GUI");
//		integral_packet ip(integral_state[0], integral_state[1], integral_state[2], integral_state[3], integral_state[4], integral_state[5]);
//		com->send_message(message("control", "control_gui", "integral", ip.whole).whole);
//
//		//Configurations
//		//INFO("Sending 6 configuration messages to controls GUI");
//		for (int i = 0; i < 6; i++)
//		{
//			control_configuration_packet ccp(P[i], I[i], D[i], windup[i], hysteresis[i], (control_configuration_packet::Axes)i);
//			com->send_message(message("control", "control_gui", "control_parameters", ccp.whole).whole);
//		}
//
//		//convert motor_commands eigen vector to stl vector
//		vector<double> motor_commands_stl_vec(motor_commands.data(), motor_commands.data() + motor_commands.size());
//		thruster_packet tp(motor_commands_stl_vec);
//		com->send_message(message("control", "thruster", "thruster", tp.whole).whole);
//
//		INFO("motor_commands:"); log << motor_commands << endl;
//
//	}
//
//	void ControlModule::shutdown()
//	{
//
//	}
//
//	string ControlModule::statestring(control_packet::State state)
//	{
//		string s;
//		switch (state)
//		{
//			case control_packet::State::NONE:     s = "NONE"; break;
//			case control_packet::State::ABSOLUTE: s = "ABSOLUTE"; break;
//			case control_packet::State::RELATIVE: s = "RELATIVE"; break;
//			case control_packet::State::ERROR:    s = "ERROR"; break;
//			default: s = "UNKNOWN"; break;
//		}
//
//		return s;
//	}
//
//	//note that this function will need to be modified when
//	//we move to 8 thrusters
//	VectorXd ControlModule::motor_control(Vector12d state)
//	{
//		//INFO("running motor control function");
//		//calcluate errors
//		Vector3d err_translation = goals.segment<3>(0) - state.segment<3>(0);
//		for(int i=0; i < 3; ++i)
//		{
//			//check if we should override the error calculation
//			if(goal_types[i] == control_packet::State::ERROR)
//			{
//				err_translation[i] = goals[i];
//				//integral_state[i+3] = 0.0; 		//ccheck +3? is translation latter 3 terms?
//				integral_state[i] = 0.0;		//fix seems correct
//			}
//		}
//
//		Vector3d rotation_goals;
//		for(int i=0; i < 3; ++i)
//		{
//			if(goal_types[i+3] == control_packet::State::ERROR)
//			{
//				rotation_goals[i] = state_vector[i+6] + goals[i+3];
//				//integral_state[i] = 0.0;
//				integral_state[i+3] = 0.0;
//			}
//			else
//			{
//				rotation_goals[i] = goals[i+3];
//			}
//		}
//		INFO("cs: rotation_gls:"); log << rotation_goals << endl;
////		Vector3d R = ir3D( r3D(rotation_goals) * r3D(state.segment<3>(6)).transpose() );
//		Vector3d R = ir3D( r3D(state.segment<3>(6)).transpose() * r3D(rotation_goals) ); //ccheck wrong order r3D' * r3D
//		Vector3d err_rotation = R;
//		log << r3D(rotation_goals) << endl;
//		log << r3D(state.segment<3>(6)).transpose() << endl;
//		//if roll and yaw are over 90 degrees either way,
//		//a more efficient route exists
//
//		//ccheck cos theta * pitch FIRST, before supplemental angle
//		err_rotation(1) = err_rotation(1) * cos(err_rotation(2));
//
//		INFO("cs: R:"); log << R << endl;
///*											//ccheck is this the problem?
//		if( (abs(R(1)) > _PI_OVER_2) && (abs(R(2)) > _PI_OVER_2) )
//		{
//			err_rotation(0) = Sgn(R(0))*_PI - R(0); //new roll is supplement angle
//			err_rotation(2) = R(2) - Sgn(R(2))*_PI; //new yaw is supplement angle
//			//pitch is unchanged
//		}
//*/
////		err_rotation.segment<2>(0) = err_rotation.segment<2>(0) * cos(err_rotation(2)); 	//ccheck both pitch and roll cos(theta)
//
//
//		Vector6d error;
//		error << err_translation, err_rotation;
//
//		//INFO("cs: error:"); log << error << endl;
//
//		//do PID for all the the motors
//		//update error integral
//		integral_state += error * dt; //where should dt come from?
//		for(int i=0; i < 6; ++i)  //check for integral windup
//		{
//			if(abs(integral_state(i)) > abs(windup(i)))
//				integral_state(i) = windup(i) * Sgn(integral_state(i));
//		}
//
//		//perform hysteresis computation
//		Vector6d hist;
//		hist = (error.array().abs() >= hysteresis.array()).cast<double>();
//		//INFO("cs: hys"); log << hist << endl;
//
//		//use PID to compute the corrective actions
//		Vector6d m_accel = Vector6d::Zero();
//		m_accel = P.cwiseProduct(error).cwiseProduct(hist); //proportional
//		//INFO("cs: m_accel1:"); log << m_accel << endl;
//
//		Vector6d tmp6v = Vector6d::Zero(); //setup custom state vector for derivative
//		tmp6v.segment<3>(0) = state.segment<3>(3);
//		tmp6v.segment<3>(3) = state.segment<3>(9);
//		m_accel += D.cwiseProduct(tmp6v); //derivative
//		m_accel += I.cwiseProduct(integral_state); //integral
//		m_accel += offsets; //add offset
//
////		INFO("cs: m_accel2:"); log << m_accel << endl;
//
//
//		Vector6d m_force = m_accel.cwiseProduct(sub_mass); //convert to force
//		INFO("cs: m_force:"); log << m_force << endl;
//
//		Vector3d tmp3v;
//		tmp3v[0] = state[6];
//		tmp3v[1] = state[7];
//		tmp3v[2] = 0; 			//setting yaw=0 means x,y commands become forward/strafe commands
//
//		tmp6v = Vector6d::Zero();
//		tmp6v.segment<3>(0) = r3D(tmp3v).transpose() * m_force.segment<3>(0);  //uncomment this line to vectorize force
//		//tmp6v.segment<3>(0) = m_force.segment<3>(0);
//		VectorXd m_ctrl_t = M * tmp6v;
//		INFO("cs: tmp6v for translation:"); log << tmp6v << endl;
//
//		tmp6v = Vector6d::Zero();
//		tmp6v.segment<3>(3) = m_force.segment<3>(3);
//		INFO("cs: tmp6v for rotation:"); log << tmp6v << endl;
//		VectorXd m_ctrl_r = M * tmp6v;
//		//INFO("cs: Motor Matrix:"); log << M.inverse() << endl;
//
//
//		//scale the motors down to limits --needs to be more intelligent: seperate  m_accel PID from offset, divide m_accel goals only
//		for (int i=0; i < num_thrusters; ++i)
//		{
//			if(abs(m_ctrl_t(i)) > t_lim)
//				m_ctrl_t(i) = t_lim*Sgn(m_ctrl_t(i));
//			if(abs(m_ctrl_r(i)) > r_lim)
//				m_ctrl_r(i) = r_lim*Sgn(m_ctrl_r(i));
//		}
//
//		VectorXd m_ctrl_sum = m_ctrl_t + m_ctrl_r;
//
//		for (int i=0; i < num_thrusters; ++i)
//		{
//			if(m_ctrl_sum[i] < 0)
//				m_ctrl_sum[i] /= back_thrust_ratio;
//		}
//
////		INFO("cs: m_control_output:"); log << m_ctrl_sum << endl;
//		return m_ctrl_sum;
//	}
//
//	double ControlModule::wraparound(double x, double min, double max)
//	{
//		double var;
//		if (x > max)
//			var = min + (x - max);
//		else if (x < min)
//			var = max + (x + max);
//		else
//			var = x;
//		return var;
//	}
//};
//
//using namespace rs;
//
//int main(int argc, char **argv)
//{
//	ControlModule control;
//
//	control.Init("control");
//
//	while(1)
//	{
//		control.Update();
//	}
//
//	return 0;
//}
