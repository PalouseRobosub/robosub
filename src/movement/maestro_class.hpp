/**
 * @author Ryan Summers
 * @date 10-25-2016
 *
 * @brief Provides declaration of the thruster controller class.
 *
 * @note Any documentation specified in this file refers to information located
 *       at https://www.pololu.com/docs/0J40. Information is formatted into
 *       section and subsection. For example, (4.5) refers to Section 4
 *       subsection 5.
 */

#ifndef THRUSTER_CONTROLLER_H
#define THRUSTER_CONTROLLER_H

#include "utility/serial.hpp"
#include <arpa/inet.h>
#include <ros/ros.h>
#include <vector>

using std::vector;

namespace rs {
    /**
     * Thruster controlling class.
     *
     * @brief This class provides an interface for communicating with the
     *        Pololu Maestro servo controller. These servo commands are then
     *        translated into a thruster power by the ESC (electronic speed
     *        controller).
     *
     * @tparam thrusters The number of thrusters to be controlled by the class.
     * @tparam max_speed The software limit on the maximum speed that any
     *         thruster may be set to.
     */
    class ThrusterController
    {
        static constexpr double reset_delay = 120.0;
        public:
            /**
             * Constructor.
             *
             * @param _thrusters The number of thrusters controlled by the
             *        controller.
             * @param _max_speed The maximum normalized speed inthe forward and
             *        reverse directions that should be allowed by the
             *        controller.
             */
            ThrusterController(const int _thrusters, const float _max_speed) :
                        port(NULL),
                        thrusters(_thrusters),
                        max_speed(_max_speed)
            {
            }

            /**
             * Sets the serial port used to communicate with the thrusters.
             *
             * @param _port A configured and initialized serial port to be used
             *        for communicating with the Maestro.
             */
            void setPort(Serial *_port)
            {
                if (port != NULL)
                {
                    port = _port;
                    uint8_t detect_byte = 0xAA;
                    port->Write(&detect_byte, 1);
                }
            }

            /**
             * Deconstructor.
             *
             * @brief This function sends zeroing commands to the maestro on
             *        class destruction.
             */
            ~ThrusterController()
            {
                if (port != NULL)
                {
                    vector<float> zero;
                    for (int i = 0; i < thrusters; ++i)
                    {
                        zero.push_back(0.0);
                    }
                    set(zero);
                }
            }

            /**
             * Sets the speed of a particular thruster.
             *
             * @param speeds The normalized speeds to set all thrusters to.
             *
             * @return 0 on success and -1 on failure.
             */
            int set(vector<float> &speeds)
            {
                if (speeds.size() != thrusters || port == NULL) return -1;

                uint8_t command[thrusters*2+3];
                command[0] = static_cast<uint8_t>(MaestroCommands::SetMultipleTargets);
                command[1] = thrusters;
                command[2] = 0;

                /*
                 * Check to see if it's currently time to send a reset signal.
                 */
                ros::Time now = ros::Time::now();
                if (now > nextReset)
                {
                    ROS_INFO_STREAM("Sending thruster reset signal.");
                    for (int thruster = 0; thruster < thrusters; ++thruster)
                    {
                        parseNormalized(0, command[thruster*2+4],
                                command[thruster*2+3]);
                    }

                    if (port->Write(command, sizeof(command)) !=
                            sizeof(command))
                        return -1;

                    nextReset = ros::Time::now() + ros::Duration(reset_delay);

                    /*
                     * Sleep for 175ms (8 50Hz cycles + 3/4 of one cycle) to
                     * ensure that the zero pulse has propogated to the ESC.
                     * Experimental tests have found this value. Any value less
                     * is insufficient.
                     */
                    usleep(185000);
                }

                /*
                 * Send thruster commands down the to the maestro.
                 */
                for (int thruster = 0; thruster < speeds.size(); ++thruster)
                {
                    const double speed = speeds[thruster];
                    if (speed < 0 || speed > 1) return -1;

                    if (speed > max_speed) speed = max_speed;
                    if (speed < -1*max_speed) speed = -1*max_speed;
                    parseNormalized(speed, command[2*thruster+4],
                            command[2*thruster+3]);
                }

                /*
                 * Write the data down the serial port.
                 */
                return (port->Write(command, sizeof(command)) !=
                        sizeof(command));
            }

        private:
            /*
             * Pointer to serial port to be used for communication with the
             * Maestro.
             */
            Serial *port;

            /*
             * The maximum speed setting that a thruster may be set to.
             */
            const float max_speed;

            /*
             * The number of thrusters to be controlled.
             */
            const int thrusters;

            /*
             * Time of the next reset command. The Maestro requires an arming
             * command of 1500 microseconds atleast once every 2 minutes and 35
             * seconds.
             */
            ros::Time nextReset;

            /**
             * Maestro-defined serial bytes that have special meaning to the
             * Maestro as control signals (5.c) (5.e). This class makes use of
             * the compact protocol.
             *
             * @note The commands all require the second byte to be the channel
             *       number, then bits 7-0 of target, and then bits 13-7 of
             *       target (repeating for multiple target commands).
             *       Exceptions to this are documented for each command.
             */
            enum class MaestroCommands : uint8_t
            {
                /*
                 * Sets the target of the servo. If the servo is configured as a
                 * servo, this setting specifies the number of quarters of a
                 * microsecond to send as a pulse. Otherwise, the servo is
                 * operating as digital output. In this case, a value >= 6000
                 * represents a '1' and a value of less than 6000 represents a '0'.
                 */
                SetTarget = 0x84,

                /*
                 * Identical to SetTarget, except it allows for any number of
                 * targets to be set. The channel number represents the starting
                 * channel and is incremented by 1 for each target sent.
                 */
                SetMultipleTargets = 0x9F,

                /*
                 * Sets the speed at which a change in the target is propogated to
                 * the servo. Units expressed are quarts of a microsecond per 10
                 * milliseconds. For example, setting a value of 140 means that the
                 * speed is 3.5 microseconds/millisecond. If the target is changed
                 * from 1000 microseconds to 1350 microseconds, it will take 100 ms
                 * for that adjustment to propogate. Speed has no effect on digital
                 * inputs or outputs.
                 */
                SetSpeed = 0x87,

                /*
                 * The acceleration represents how quickly the servo ramps up to
                 * maximum speed. This represents 0.25 microseconds/10ms/80ms.
                 * Values must be from 0 to 255.
                 */
                SetAcceleration = 0x89,

                /*
                 * Sets a specified servo channel to a PWM output. The 2 bytes
                 * following the channel number indicate the on-time of the PWM and
                 * the two bytes following that specify the period of the PWM.
                 * These values are represented as 1/48microseconds.
                 */
                SetPWM = 0x8A,

                /*
                 * Gets the current position value of a channel. The meaning of
                 * this value depends on the mode of the servo. In servo mode, it
                 * returns the on-time of the servo. Response is sent as a 2-byte
                 * response immediately after transmission.
                 */
                GetPosition = 0x90,

                /*
                 * Returns a 0 if the servos are transitioning to their set
                 * positions and a 1 if servos are at their final positions.
                 */
                GetMovingState = 0x93,

                /*
                 * Returns two bytes of error codes (LSB first) representing any
                 * pending errors that the Maestro has detected.
                 */
                GetErrors = 0xA1,

                /*
                 * Sets all servos to go to the home position. This command accepts
                 * no following bytes.
                 */
                GoHome = 0xA2
            };

            /**
             * Parses a normalized speed value into a Maestro-compatible number.
             *
             * @param speed The normalized speed to parse.
             * @param[out] msb The location to store the most significant bit of
             *             the result.
             * @param[out] lsb The location to store the least significant bit of
             *             the result.
             *
             * @return None.
             */
            void  parseNormalized(const double speed, uint8_t &msb, uint8_t &lsb)
            {
                const uint16_t speed_microseconds = speed*400;
                const uint16_t quarter_microseconds = (1500+speed_microseconds)*4;

                /*
                 * Convert the microseconds to network endianness and store them
                 * into the supplied locations.
                 */
<<<<<<< Updated upstream
                const uint16_t net_order = htons(quarter_microseconds);

                msb = net_order >> 7;
                lsb = net_order & 0x7F;
=======
                msb = quarter_microseconds >> 7;
                lsb = quarter_microseconds & 0x7F;
>>>>>>> Stashed changes
            }
    };
}

#endif
