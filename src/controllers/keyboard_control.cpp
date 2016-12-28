#include "ros/ros.h"
#include "robosub/control.h"
#include <termios.h>
#include <vector>
#include <ostream>

using std::vector;

// Used for printing the key codes to the screen for debugging
std::ostream& operator<<(std::ostream& os, vector<uint8_t> v)
{
    for (auto iter = v.begin(); iter < v.end(); ++iter)
    {
        os << static_cast<int>(*iter) << ' ';
    }

    return os;
}

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
        uint8_t key;
        vector<uint8_t> keys;

        // Get all of the codes for the key combination
        // 255 denotes the end of the combintation
        do
        {
            key = getKey();
            keys.push_back(key);
        } while(255 != key);

        tcflush(fileno(stdin), TCIFLUSH);

        // If problems are occuring check the key codes
        ROS_DEBUG_STREAM(keys);

        switch (keys[0])
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
            case 27:  // Escape Code for special keys
                switch(keys[2])
                {
                    case 'A':  // Up Key pressed
                        msg.forward = 1;
                        break;
                    case 'B':  // Down Key pressed
                        msg.forward = -1;
                        break;
                    case 'C':  // Right key pressed
                        msg.strafe_left = -1;
                        break;
                    case 'D':  // Left key pressed
                        msg.strafe_left = 1;
                        break;
                    case '5':  // Page up Key pressed
                        msg.dive = 0.25;
                        break;
                    case '6':  // Page down Key pressed
                        msg.dive = -0.25;
                        break;
                    case '1':  // Multi-key combination
                        // Currently not discriminating on which one:
                        // Can add features later by checking keys[4] for
                        // Ctrl, shift, or alt, or a combination of them
                        // Use `showkey -a` to see the differences
                        switch(keys[5])
                        {
                            case 'A':  // Up Key
                                msg.pitch_down = 10;
                                break;
                            case 'B':  // Down Key
                                msg.pitch_down = -10;
                                break;
                            case 'C':  // Right Key
                                msg.yaw_left = -10;
                                break;
                            case 'D':  // Left Key
                                msg.yaw_left = 10;
                                break;
                        }
                }
                break;
        }

        pub.publish(msg);
        rate.sleep();
    }

    tcsetattr(fileno(stdin), TCSANOW, &attr_backup);

    return 0;
}
