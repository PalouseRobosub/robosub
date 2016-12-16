#include "gamepad_driver.hpp"
#include <string>

GamepadDriver::GamepadDriver(ros::NodeHandle *nh)
{
    node = nh;
    AXIS_MAX = 32767;

    if(!(node->getParam("device", device)))
    {
        device = "/dev/input/js0";
        ROS_WARN("Unable to open device from params");
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

robosub::gamepad GamepadDriver::GetGamepadMessage()
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

    // Create gamepad msg
    robosub::gamepad gp_msg;

    gp_msg.axisX = static_cast<double>(gamepad_data.axisX);
    gp_msg.axisY = static_cast<double>(gamepad_data.axisY);
    gp_msg.axisZ = static_cast<double>(gamepad_data.axisZ);

    gp_msg.axisRX = static_cast<double>(gamepad_data.axisRX);
    gp_msg.axisRY = static_cast<double>(gamepad_data.axisRY);
    gp_msg.axisRZ = static_cast<double>(gamepad_data.axisRZ);

    gp_msg.hatX = gamepad_data.hatX;
    gp_msg.hatY = gamepad_data.hatY;

    for(int i = 0; i < 11; ++i)
    {
        gp_msg.buttons[i] = gamepad_data.button[i];
    }

    //normalize all of the values
    gp_msg.axisX = gp_msg.axisX / static_cast<double>(AXIS_MAX);
    gp_msg.axisY = gp_msg.axisY / static_cast<double>(AXIS_MAX);
    gp_msg.axisZ = gp_msg.axisZ / static_cast<double>(AXIS_MAX);

    gp_msg.axisRX = gp_msg.axisRX / static_cast<double>(AXIS_MAX);
    gp_msg.axisRY = gp_msg.axisRY / static_cast<double>(AXIS_MAX);
    gp_msg.axisRZ = gp_msg.axisRZ / static_cast<double>(AXIS_MAX);

    gp_msg.hatX /= AXIS_MAX;
    gp_msg.hatY /= AXIS_MAX;

    return gp_msg;
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
        case 0: //Left stick: left-right-axis, left is negative
            gamepad_data.axisX = e.value;
            break;
        case 1: //Left stick: forward-back-axis, forward is negative
            gamepad_data.axisY = e.value;
            break;
        case 2: //left trigger
            gamepad_data.axisZ = e.value;
            break;
        case 3: //Right stick: left-right-axis, left is negative
            gamepad_data.axisRX = e.value;
            break;
        case 4: //Right stick: forward-back-axis, forward is negative
            gamepad_data.axisRY = e.value;
            break;
        case 5: //right trigger
            gamepad_data.axisRZ = e.value;
            break;
        case 6: //dpad left-right, left is negative
            gamepad_data.hatY = e.value;
            break;
        case 7: //dpad up-down, forward is negative
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
    pub = nh.advertise<robosub::gamepad>("gamepad_driver", 1);
    nh = ros::NodeHandle("gamepad_driver");

    int rate;

    // Set default rate to 10 if param fails to load
    if(!nh.getParam("rate", rate))
    {
        rate = 10;
        ROS_WARN("Unable to get rate from params");
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