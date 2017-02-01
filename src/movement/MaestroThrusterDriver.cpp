#include "MaestroThrusterDriver.h"
#include <map>

namespace rs
{
    /**
     * Constructor.
     */
    MaestroThrusterDriver::MaestroThrusterDriver() :
        _is_initialized(false)
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
                            Serial *port,
                            const double post_reset_delay_ms)
    {
        if(setPort(port) != 0)
        {
            return -1;
        }

        /*
         * Load the maximum thruster force and the back thrust
         * ratio from the settings.
         */
        double max_thrust_newtons = 0;
        if (ros::param::get("thrusters/max_thrust", max_thrust_newtons) ==
                false)
        {
            ROS_ERROR("Failed to load maximum thruster force.");
            return -1;
        }

        /*
         * The scaling factor for converting newtons to KgF is
         * 0.101972. Convert the Newtonian thrust into KgF.
         */
        _max_thrust_kgf = max_thrust_newtons * 0.101972;
        ROS_INFO_STREAM("Maximum thrust in KgF: " << _max_thrust_kgf);

        /*
         * Initialize all data maps to indicate that resets need to occur
         * immediately (for arming signals) and initialize max speed to zero.
         */
        ros::Time now = ros::Time::now();
        for(uint8_t i = 0; i < max_thrusters; ++i)
        {
            _next_reset[i] = now;
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
            if (parseNormalized(0, command[3], command[2]) < 0)
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

        const int signal = parseNormalized(speed, command[3], command[2]);

        if (signal < 0)
        {
            ROS_ERROR("Parse Normalized encountered abnormal thruster speed.");
            return -1;
        }

        /*
         * The BasicESC has a signal deadband of +/- 25
         * microseconds on the signal pulse. Print out information
         * if the signal is in the dead-band to inform the
         * operator that the thruster should not spin.
         */
        if (signal != 1500 && abs(signal - 1500) < 25)
        {
            ROS_WARN("Parsed signal is in thruster dead-band.");
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
     * @param normalized_force The normalized speed to parse.
     *        Values must fall within the range [-1,1] and
     *        represent a ratio of the total thruster force
     *        desired.
     * @param[out] msb The location to store the most significant bit
     *             of the result.
     * @param[out] lsb The location to store the least significant bit
     *             of the result.
     *
     * @return The thruster command signal on success and -1 on
     *         failure.
     */
    int MaestroThrusterDriver::parseNormalized(const double normalized_force,
                                                   uint8_t &msb, uint8_t &lsb)
    {
        if (normalized_force < -1 || normalized_force > 1)
        {
            return -1;
        }

        /*
         * To convert the normalized speed into a thruster
         * command, the two characteristic equations found for
         * either positive or negative force need to be applied
         * to the desired thrust output. The characteristic
         * equations take in the thrust as KgF, so convert the
         * normalized thrust value to KgF. The result of this polynomial will
         * be the signal to send to the thruster. Note that
         * thruster signals are centered around 1500, going down
         * to 1100 for negative and up to 1900 for positive
         * signals.
         */
        uint16_t signal = 0;
        double force_kgf = normalized_force * _max_thrust_kgf;

        if (std::fabs(force_kgf) < _minimum_thrust_kgf)
        {
            force_kgf = 0;
        }

        ROS_DEBUG_STREAM("Force (KgF): " << force_kgf);

        if (force_kgf > 0)
        {
            signal = a_positive * pow(force_kgf, 3) +
                     b_positive * pow(force_kgf, 2) +
                     c_positive * force_kgf +
                     d_positive;
        }
        else if (force_kgf < 0)
        {
            signal = a_negative * pow(force_kgf, 3) +
                     b_negative * pow(force_kgf, 2) +
                     c_negative * force_kgf +
                     d_negative;
        }
        else
        {
            signal = 1500;
        }

        /*
         * Implement a software hard limit on the signals being sent to the
         * thrusters to ensure that they do not draw over 10 Amps of current.
         */
        if (signal > 1780)
        {
            signal = 1780;
            ROS_WARN("Software hard-limitting thrusters forward.");
        }
        else if (signal < 1230)
        {
            signal = 1230;
            ROS_WARN("Software hard-limitting thrusters in reverse.");
        }

        /*
         * Store results into supplied locations.
         */
        msb = signal >> 7;
        lsb = signal & 0x7F;

        return signal;
    }
}
