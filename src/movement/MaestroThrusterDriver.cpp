#include "MaestroThrusterDriver.h"
#include <map>

namespace rs
{
    /**
     * Constructor.
     */
    MaestroThrusterDriver::MaestroThrusterDriver() :
        _is_initialized(false),
        _port(nullptr),
        _max_speed(),
        _post_reset_delay_ms(185),
        _next_reset()
    {
    }

    /**
     * Deconstructor.
     */
    MaestroThrusterDriver::~MaestroThrusterDriver()
    {
    }

    /**
     * initializes the thruster controller. Must be called before using any
     * other functions
     *
     * @param thrusters The number of thrusters controlled by the controller.
     * @param max_speed The maximum normalized speed in the forward and reverse
     *        directions that should be allowed by the controller.
     * @param delay_ms Delay specified in milliseconds to continue sending
     *        zeroing signal to continue thruster operation.
     * @param port The serial port to be used for communicating with the
     *        thrusters. This port may be changed during runtime.
     */
    int MaestroThrusterDriver::init(
                            std::map<uint8_t, double> max_speed,
                            Serial *port,
                            const double post_reset_delay_ms)
    {
        if(setPort(port) != 0)
        {
            return -1;
        }

        /*
         * Initialize all data maps to indicate that resets need to occur
         * immediately (for arming signals) and initialize max speed to zero.
         */
        ros::Time now = ros::Time::now();
        for(uint8_t i = 0; i < max_thrusters; ++i)
        {
            _next_reset[i] = now;
            _max_speed[i] = 0.0;
        }

        /*
         * Properly overwrite the max speed of zero with provided max speed
         * values.
         */
        for (auto iter = max_speed.begin(); iter != max_speed.end(); iter++)
        {
            const uint8_t thruster_key = iter->first;
            _max_speed[thruster_key] = max_speed[thruster_key];
        }

        _post_reset_delay_ms = post_reset_delay_ms;
        if (post_reset_delay_ms < min_post_reset_delay_ms)
        {
            ROS_ERROR("Thruster millisecond delay is too low.");
            return -1;
        }

        _is_initialized = true;
        return 0;
    }

    /**
     * Sets the serial port used to communicate with the thrusters.
     *
     * @param port A pointer to a configured and initialized serial port to be
     *        used for communicating with the Maestro.
     *
     * @return Zero upon success and -1 upon failure.
     */
    int MaestroThrusterDriver::setPort(Serial *port)
    {
        _port = port;
        if (_port == nullptr)
        {
            ROS_ERROR("Serial port pointer supplied is not a valid pointer.");
            return -1;
        }

        uint8_t detect_byte = 0xAA;
        if (port->Write(&detect_byte, 1) != 1)
        {
            ROS_ERROR("Serial port failed to write baud-detection bit.");
            return -1;
        }

        return 0;
    }

    /**
     * Sets the speed of a thruster.
     *
     * @param speeds The normalized speed to set the thruster to.
     *
     * @return 0 on success and -1 on failure.
     */
    int MaestroThrusterDriver::set(double speed, const uint8_t &channel)
    {
        if(_is_initialized != true)
        {
            ROS_ERROR("motor controller used before being initialized");
            return -1;
        }

        /*
         * Create an array of bytes. A thruster command requires 4 bytes
         */
        uint8_t command[4];

        command[0] = static_cast<uint8_t>(MaestroCommands::SetTarget);
        command[1] = channel;

        /*
         * Check to see if it's currently time to send a reset signal.
         * Send the reset signal first if necessary
         */
        ros::Time now = ros::Time::now();
        if (now > _next_reset[channel])
        {
            ROS_DEBUG_STREAM("Sending thruster reset signal.");
            if (parseNormalized(0, command[3], command[2]))
            {
                ROS_ERROR("Parse Normalized encountered abnormal "
                          "thruster speed.");
                return -1;
            }

            if (_port->Write(command, sizeof(command)) != sizeof(command))
            {
                ROS_ERROR("Serial port failed to write entire command.");
                return -1;
            }

            _next_reset[channel] = ros::Time::now() +
                    ros::Duration(reset_timeout);

            /*
             * Sleep to ensure that the zero pulse has propogated to the ESC.
             * Any value less than 185ms may result in the ESC not receiving
             * the zero pulse, which will cause it to malfunction
             */
            ros::Duration(
                    static_cast<double>(_post_reset_delay_ms/1000)).sleep();
        }

        /*
         * Send thruster command down the to the Maestro.
         */
        if (speed < -1 || speed > 1)
        {
            ROS_ERROR("Thruster speed out of range.");
            return -1;
        }

        if(fabs(speed) > _max_speed[channel])
        {
            ROS_INFO("Software-limiting thruster speed.");
            speed = _max_speed[channel] * ((speed < 0)? -1 : 1);
        }

        if (parseNormalized(speed, command[3], command[2]))
        {
            ROS_ERROR("Parse Normalized encountered abnormal thruster speed.");
            return -1;
        }

        /*
         * Write the data down the serial port.
         */
        return (_port->Write(command, sizeof(command)) != sizeof(command));
    }

    /**
     * Parses a normalized thrust value into a Maestro-compatible
     * command.
     *
     * @param speed The normalized speed to parse. Values must fall
     *        within the range [-1,1] and represent a ratio of the
     *        total thruster force desired.
     * @param[out] msb The location to store the most significant bit
     *             of the result.
     * @param[out] lsb The location to store the least significant bit
     *             of the result.
     *
     * @return Zero on success and -1 on failure.
     */
     int MaestroThrusterDriver::parseNormalized(const double speed,
                                                    uint8_t &msb, uint8_t &lsb)
     {
         if (speed < -1 || speed > 1) return -1;
         /*
          * To convert the normalized speed into a thruster
          * command, the two characteristic equations found for
          * either positive or negative force need to be applied
          * to the desired thrust output.
          */
         uint16_t signal;
         if (speed < 0)
         {
            signal = a_negative * pow(speed, 3) + pow(b_negative, 2) * speed +
                c_negative * speed + d_negative;
         }
         else if (speed > 0)
         {
            signal = a_positive * pow(speed, 3) + pow(b_positive, 2) * speed +
                c_positive * speed + d_positive;
         }
         else
         {
            signal = 1500;
         }

         /*
          * Store results into supplied locations.
          */
         msb = signal >> 7;
         lsb = signal & 0x7F;

         return 0;
     }
}
