#include "movement/ThrusterController.hpp"
namespace rs
{
    /**
     * Constructor.
     *
     * @param thrusters The number of thrusters controlled by the controller.
     * @param max_speed The maximum normalized speed in the forward and reverse
     *        directions that should be allowed by the controller.
     * @param delay_ms Delay specified in milliseconds to continue sending
     *        zeroing signal to continue thruster operation.
     * @param port The serial port to be used for communicating with the
     *        thrusters. This port may be changed during runtime.
     */
    ThrusterController::ThrusterController(const int thrusters,
                                           const double max_speed,
                                           const Serial *port = nullptr,
                                           const double delay_ms = 185.00) :
            _port(port),
            _thrusters(_thrusters),
            _max_speed(_max_speed),
            _delay_ms(delay_ms)
    {
        if (delay_ms < 185.00)
        {
            ROS_ERROR("Thruster millisecond delay is too low.");
        }
    }

    /**
     * Deconstructor.
     *
     * @brief This function sends zeroing commands to the maestro on class
     *        destruction.
     */
    ThrusterController::~ThrusterController()
    {
        if (_port != nullptr)
        {
            vector<double> zero(_thrusters, 0.0);
            set(zero);
        }
    }

    /**
     * Sets the serial port used to communicate with the thrusters.
     *
     * @param port A pointer to a configured and initialized serial port to be
     *        used for communicating with the Maestro.
     *
     * @return Zero upon success and -1 upon failure.
     */
    int ThrusterController::setPort(const Serial *port)
    {
        _port = port;
        if (_port != nullptr)
        {
            uint8_t detect_byte = 0xAA;
            if (port->Write(&detect_byte, 1) != 1)
            {
                ROS_ERROR("Serial port failed to write baud-detection bit.");
                return -1;
            }
            return 0;
        }
        ROS_ERROR("Serial port pointer supplied is not a valid pointer.");
        return -1;
    }

    /**
     * Sets the speed of a particular thruster.
     *
     * @param speeds The normalized speeds to set all thrusters to.
     *
     * @return 0 on success and -1 on failure.
     */
    int ThrusterController::set(vector<double> &speeds)
    {
        if (speeds.size() != _thrusters || _port == nullptr) return -1;

        /*
         * Create an array of bytes. Each thruster command requires 2 bytes and
         * 3 bytes are used for the header.
         */
        uint8_t command[thrusters*2+3];
        command[0] = static_cast<uint8_t>(MaestroCommands::SetMultipleTargets);
        command[1] = _thrusters;
        command[2] = 0;

        /*
         * Check to see if it's currently time to send a reset signal.
         */
        ros::Time now = ros::Time::now();
        if (now > _next_reset)
        {
            ROS_INFO_STREAM("Sending thruster reset signal.");
            for (int current_thruster = 0; current_thruster < _thrusters;
                    ++current_thruster)
            {
                if (parseNormalized(0, command[current_thruster*2+4],
                            command[current_thruster*2+3]))
                {
                    ROS_ERROR("Parse Normalized encountered abnormal thruster
                            speed.");
                    return -1;
                }
            }

            if (port->Write(command, sizeof(command)) != sizeof(command))
            {
                ROS_ERROR("Serial port failed to write entire command.");
                return -1;
            }

            _next_reset = ros::Time::now() + ros::Duration(reset_delay);

            /*
             * Sleep to ensure that the zero pulse has propogated to the ESC.
             * Any value less than 185ms may result in the ESC malfunctioning.
             */
            usleep(_delay_ms*1000);
        }

        /*
         * Send thruster commands down the to the Maestro.
         */
        for (int thruster = 0; thruster < speeds.size(); ++thruster)
        {
            double current_speed = speeds[thruster];
            if (current_speed < -1 || current_speed > 1)
            {
                ROS_ERROR("Thruster speed out of range.");
                return -1;
            }

            if (current_speed > max_speed)
            {
                ROS_INFO("Software-limiting thruster speed.");
                current_speed = max_speed;
            }
            if (current_speed < -1*max_speed)
            {
                ROS_INFO("Software-limiting thruster reverse speed.");
                current_speed = -1*max_speed;
            }
            if (parseNormalized(current_speed, command[2*thruster+4],
                        command[2*thruster+3]))
            {
                ROS_ERROR("Parse Normalized encountered abnormal thruster
                        speed.");
                return -1;
            }
        }

        /*
         * Write the data down the serial port.
         */
        return (port->Write(command, sizeof(command)) != sizeof(command));
    }

    /**
     * Parses a normalized speed value into a Maestro-compatible
     * number.
     *
     * @param speed The normalized speed to parse. Values must fall
     *        within the range [-1,1].
     * @param[out] msb The location to store the most significant bit
     *             of the result.
     * @param[out] lsb The location to store the least significant bit
     *             of the result.
     *
     * @return Zero on success and -1 on failure.
     */
     int ThrusterController::parseNormalized(const double speed, uint8_t &msb,
             uint8_t &lsb)
     {
         if (speed < -1 || speed > 1) return -1;

         const uint16_t speed_microseconds = speed*400;
         const uint16_t quarter_microseconds = (1500+speed_microseconds)*4;

         /*
          * Store results into supplied locations.
          */
         msb = quarter_microseconds >> 7; lsb = quarter_microseconds & 0x7F;

         return 0;
     }
}
