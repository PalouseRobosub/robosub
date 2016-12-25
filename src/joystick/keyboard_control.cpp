#include "ros/ros.h"
#include "robosub/control.h"
#include <termios.h>

uint8_t getKey(void)
{
    return fgetc(stdin);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_control");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<robosub::control>("control", 1);

    robosub::control base_msg;

    base_msg.pitch_state   = robosub::control::STATE_ABSOLUTE;
    base_msg.roll_state    = robosub::control::STATE_ABSOLUTE;
    base_msg.forward_state = robosub::control::STATE_ERROR;
    base_msg.strafe_state  = robosub::control::STATE_ERROR;
    base_msg.dive_state    = robosub::control::STATE_RELATIVE;
    base_msg.yaw_state     = robosub::control::STATE_RELATIVE;

    //backup terminal settings
    struct termios attr_backup;
    tcgetattr(fileno(stdin), &attr_backup);

    //setup new terminal settings
    struct termios attr_new;
    memcpy(&attr_new, &attr_backup, sizeof(struct termios));
    attr_new.c_lflag &= ~(ECHO|ICANON);
    attr_new.c_cc[VTIME] = 0;
    attr_new.c_cc[VMIN] = 0;

    tcsetattr(fileno(stdin), TCSANOW, &attr_new);

    ros::Rate rate(10);

    while(ros::ok())
    {
        robosub::control msg = base_msg;
        uint8_t key = getKey();
        tcflush(fileno(stdin), TCIFLUSH);
        switch (key)
        {
            case 'w':
                  msg.forward = 1;
                  break;
            case 's':
                  msg.forward = -1;
                  break;
            case 'a':
                 msg.strafe_left = 1;
                 break;
            case 'd':
                 msg.strafe_left = -1;
                 break;
            case 'q':
                 msg.yaw_left = 10;
                 break;
            case 'e':
                 msg.yaw_left = -10;
                 break;
            case 'z':
                 msg.dive = -0.25;
                 break;
            case 'c':
                  msg.dive = 0.25;
                  break;
        }

        pub.publish(msg);
        rate.sleep();
    }

    tcsetattr(fileno(stdin), TCSANOW, &attr_backup);

    return 0;
}
