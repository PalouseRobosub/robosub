#include "movement/maestro_thruster_controller.hpp"

namespace rs
{
    /**
     * Constructor.
     *
     */
    MaestroThrusterController::MaestroThrusterController()
    {
        _is_initialized = false;


    }


    /**
     * Deconstructor.
     *
     */
    MaestroThrusterController::~MaestroThrusterController()
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
    int MaestroThrusterController::init(
                            std::map<uint8_t, double> max_speed,
                            Serial *port,
                            const int post_reset_delay_ms)
    {
        if(setPort(port) != 0)
        {
            return -1;
        }


        //initialze thruster settings
        //loop over all possible channels (0-11)
        ros::Time now = ros::Time::now();
        for(uint8_t i = 0; i < max_thrusters; ++i)
        {
            //initialze reset time
            _next_reset[i] = now;

            //default max thruster speeds to zero
            _max_speed[i] = 0.0;

        }
        for (auto iter = max_speed.begin(); iter != max_speed.end(); iter++)
        {
            //set max speed for thrusters
            uint8_t thruster_key;
            thruster_key = iter->first;
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
    int MaestroThrusterController::setPort(Serial *port)
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
     * Sets the speed of a thruster.
     *
     * @param speeds The normalized speed to set the thruster to.
     *
     * @return 0 on success and -1 on failure.
     */
    int MaestroThrusterController::set(double speed, const uint8_t &channel)
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
                ROS_ERROR( "Parse Normalized encountered abnormal thruster speed.");
                return -1;
            }

            if (_port->Write(command, sizeof(command)) != sizeof(command))
            {
                ROS_ERROR("Serial port failed to write entire command.");
                return -1;
            }

            _next_reset[channel] = ros::Time::now() + ros::Duration(reset_timeout);

            /*
             * Sleep to ensure that the zero pulse has propogated to the ESC.
             * Any value less than 185ms may result in the ESC not receiving
             * the zero pulse, which will cause it to malfunction
             */
            ros::Duration(float(_post_reset_delay_ms)/1000).sleep();
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
     int MaestroThrusterController::parseNormalized(const double speed, uint8_t &msb,
             uint8_t &lsb)
     {
         //General process for determining the value:
         //The thruster ESC expects a pulse-width value in the range of 1100-1900us
         //with 1100us being full-reverse, 1900us full-forward, and 1500us stop.
         //to translate -1 to 1 to this range, use the following equation:
         //    pulse_width = (1500 + 400*speed)
         //once the pulse-width is known, the value to send down to the maestro
         //is calculated as follows:
         //     msb = (pulse_width*4) >> 7
         //     lsb = (pulse_width*4) & 0x7F
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
