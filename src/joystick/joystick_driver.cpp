#include "joystick_driver.hpp"

JoystickDriver::JoystickDriver(ros::NodeHandle *nh)
{
    node = nh;
    AXIS_MAX = 32767;

    if(!(node->getParam("device", device)))
    {
        device = "/dev/input/js0";
    }
    ROS_INFO("device: %s", device.c_str());

    fd = open(device.c_str(), O_RDONLY | O_NONBLOCK);

    if (fd < 0)
    {
        ROS_DEBUG("could not open joystick: %s", device.c_str());
        exit(1);
        return;
    }
}

robosub::joystick JoystickDriver::GetJoystickMessage()
{
    while(read(fd, &e, sizeof(e)) > 0)
    {
        parse_event();
    }
    if (errno != EAGAIN) //check for any errors
    {
        //std::cerr << "JOYSTICK READ ERROR: " << string(strerror(errno)) << std::endl;
        ROS_DEBUG("JOYSTICK READ ERROR: %s\n", std::string(strerror(errno)).c_str());
        exit(1);
    }

    // Create joystick msg
    robosub::joystick js_msg;
    //ROS_DEBUG("joystick_data.axisX: %f\n", joystick_data.axisX);

    js_msg.axisX = (double)joystick_data.axisX;
    js_msg.axisY = (double)joystick_data.axisY;
    js_msg.axisZ = (double)joystick_data.axisZ;

    js_msg.hatX = joystick_data.hatX;
    js_msg.hatY = joystick_data.hatY;

    js_msg.throttle = (double)joystick_data.throttle;

    for(int i=0; i<12; ++i)
    {
        js_msg.buttons[i] = joystick_data.button[i];
    }

    //normalize all of the values
    js_msg.axisX = js_msg.axisX / (double)AXIS_MAX;
    js_msg.axisY = js_msg.axisY / (double)AXIS_MAX;
    js_msg.axisZ = js_msg.axisZ / (double)AXIS_MAX;
    js_msg.throttle = (((double)joystick_data.throttle/AXIS_MAX)+1)/2;
    js_msg.hatX /= AXIS_MAX;
    js_msg.hatY /= AXIS_MAX;

    js_msg.axisX *= -1.0;
    js_msg.axisX = (js_msg.axisX == -0.0) ? 0.0 : js_msg.axisX;

    return js_msg;
}

void JoystickDriver::parse_event()
{
    switch(e.type & ~JS_EVENT_INIT)
    {
        case JS_EVENT_BUTTON:
            joystick_data.button[e.number] = e.value;
            break;
        case JS_EVENT_AXIS:
            switch(e.number)
            {
                case 0: //left_right_axis, left is negative
                    joystick_data.axisY = e.value;
                    break;
                case 1: //forward_back_axis, forward is negative
                    joystick_data.axisX = e.value;
                    break;
                case 2: //twist axis, left is negative
                    joystick_data.axisZ = e.value;
                    break;
                case 3: //throttle, up is negative
                    joystick_data.throttle = e.value;
                    break;
                case 4: //hat left-right, left is negative
                    joystick_data.hatY = e.value;
                    break;
                case 5: //hat up-down, forward is negative
                    joystick_data.hatX = e.value;
                    break;
            }
            break;
    }
}

void JoystickDriver::shutdown()
{
    close(fd);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joystick_driver");
    ros::NodeHandle nh;

    ros::Publisher pub;
    pub = nh.advertise<robosub::joystick>("joystick_driver", 1);

    ros::Rate r(10);

    JoystickDriver joystick(&nh);
    while(ros::ok())
    {
        pub.publish(joystick.GetJoystickMessage());
        ros::spinOnce();
        r.sleep();
    }

    joystick.shutdown();

    return 0;
}
