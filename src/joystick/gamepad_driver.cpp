#include "gamepad_driver.hpp"
#include <string>

std::ostream& operator<<(std::ostream& os, const js_event& e)
{
    os << "time:  " << e.time << std::endl;
    os << "value: " << e.value << std::endl;
    os << "type:  ";

    switch (e.type & ~JS_EVENT_INIT)
    {
        case JS_EVENT_BUTTON:
            os << "BUTTON";
            break;
        case JS_EVENT_AXIS:
            os << "AXIS";
        default:
            os << "UNKNOWN";
    }

    if (e.type & JS_EVENT_INIT)
    {
        os << " (INIT)";
    }
    os << std::endl;
    os << "number:  " << (int) e.number << std::endl << std::endl;

    return os;
}

std::ostream& operator<<(std::ostream& os, GAMEPAD_STATE& state)
{
    os << "axisX: " << state.axisX << std::endl;
    os << "axisY: " << state.axisY << std::endl;
    os << "axisZ: " << state.axisZ << std::endl;

    os << "hatX:  " << state.hatX << std::endl;
    os << "hatY:  " << state.hatY << std::endl;

    os << "throttle: " << state.throttle << std::endl;

    for (unsigned int i = 0; i < 12; ++i)
    {
        os << "button: " << i+1 << ":  " << state.button[i] << std::endl;
    }

    return os;
}

GamepadDriver::GamepadDriver(ros::NodeHandle *nh)
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
        ROS_DEBUG("could not open gamepad: %s", device.c_str());
        exit(1);
        return;
    }
}

robosub::joystick GamepadDriver::GetGamepadMessage()
{
    while(read(fd, &e, sizeof(e)) > 0)
    {
        parse_event();
    }
    if (errno != EAGAIN) //check for any errors
    {
        ROS_DEBUG("GAMEPAD READ ERROR: %s\n",
                  std::string(strerror(errno)).c_str());
        exit(1);
    }

    // Create joystick msg
    robosub::joystick js_msg;
    //ROS_DEBUG("gamepad_data.axisX: %f\n", gamepad_data.axisX);

    js_msg.axisX = static_cast<double>(gamepad_data.axisX);
    js_msg.axisY = static_cast<double>(gamepad_data.axisY);
    js_msg.axisZ = static_cast<double>(gamepad_data.axisZ);

    js_msg.hatX = gamepad_data.hatX;
    js_msg.hatY = gamepad_data.hatY;

    js_msg.throttle = static_cast<double>(gamepad_data.throttle);

    for(int i = 0; i < 12; ++i)
    {
        js_msg.buttons[i] = gamepad_data.button[i];
    }

    //normalize all of the values
    js_msg.axisX = js_msg.axisX / static_cast<double>(AXIS_MAX);
    js_msg.axisY = js_msg.axisY / static_cast<double>(AXIS_MAX);
    js_msg.axisZ = js_msg.axisZ / static_cast<double>(AXIS_MAX);
    js_msg.throttle = ((static_cast<double>(gamepad_data.throttle)/AXIS_MAX)
                       +1)/2;
    js_msg.hatX /= AXIS_MAX;
    js_msg.hatY /= AXIS_MAX;

    js_msg.axisX *= -1.0;
    js_msg.axisX = (js_msg.axisX == -0.0) ? 0.0 : js_msg.axisX;

    return js_msg;
}

void GamepadDriver::parse_event()
{
    switch(e.type & ~JS_EVENT_INIT)
    {
    case JS_EVENT_BUTTON:
        gamepad_data.button[e.number] = e.value;
        break;
    case JS_EVENT_AXIS:
        switch(e.number)
        {
        case 0: //left_right_axis, left is negative
            gamepad_data.axisY = e.value;
            break;
        case 1: //forward_back_axis, forward is negative
            gamepad_data.axisX = e.value;
            break;
        case 2: //twist axis, left is negative
            gamepad_data.axisZ = e.value;
            break;
        case 3: //throttle, up is negative
            gamepad_data.throttle = e.value;
            break;
        case 4: //hat left-right, left is negative
            gamepad_data.hatY = e.value;
            break;
        case 5: //hat up-down, forward is negative
            gamepad_data.hatX = e.value;
            break;
        }
        break;
    }
}

void GamepadDriver::shutdown()
{
    close(fd);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gamepad_driver");
    ros::NodeHandle nh;

    ros::Publisher pub;
    pub = nh.advertise<robosub::joystick>("gamepad_driver", 1);

    int rate;

    // Set default rate to 10 if param fails to load
    if(!nh.getParam("rate", rate))
    {
        rate = 10;
    }

    ros::Rate r(rate);
    ROS_INFO("rate: %d", rate);

    GamepadDriver gamepad(&nh);
    while(ros::ok())
    {
        pub.publish(gamepad.GetGamepadMessage());
        ros::spinOnce();
        r.sleep();
    }

    gamepad.shutdown();

    return 0;
}
