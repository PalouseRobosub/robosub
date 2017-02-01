/**
 * @author Ryan Summers
 * @date 10-25-2016
 *
 * @brief Provides declaration and definitions of the thruster driver class.
 *
 * @note Any documentation specified in this file refers to information located
 *       at https://www.pololu.com/docs/0J40. Information is formatted into
 *       section and subsection. For example, (4.5) refers to Section 4
 *       subsection 5.
 */

#ifndef MAESTROTHRUSTERDRIVER_H
#define MAESTROTHRUSTERDRIVER_H

#include <map>
#include <ros/ros.h>
#include <vector>

#include "utility/serial.hpp"

namespace rs
{
/**
 * Maestro thruster driver class.
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
class MaestroThrusterDriver
{
    /*
     * Specifies the delay in seconds after which a reset signal (1500
     * microsecond pulse) should be sent to the ESC. This must occur
     * periodically (at least once every 2:35 minutes), or the ESC will
     * stop responding to commands.
     */
    static constexpr double reset_timeout = 120.0;

    static constexpr double _minimum_thrust_kgf = 0.01;

    /*
     * The number of thrusters to be controlled by the Maestro.
     */
    static constexpr int max_thrusters = 12;

    /*
     * the minimum time (in milliseconds) needed for reset signal to
     * propagate from the maestro to the ESC, determined emperically
     */
    static constexpr double min_post_reset_delay_ms = 185;

public:
    MaestroThrusterDriver();

    ~MaestroThrusterDriver();

    int init(Serial *port,
            const double post_reset_delay_ms = min_post_reset_delay_ms);

    int set(double speed, const uint8_t &channel);

private:
    /*
     * Specified true if the thrusters have been initialized.
     */
    bool _is_initialized;

    /*
     * Pointer to the serial port to be used for communication with the
     * Maestro.
     */
    Serial *_port;

    /*
     * The delay (in milliseconds) to sleep after every reset cycle.  This
     * value must be greater than, or equal, to 185ms.
     */
    double _post_reset_delay_ms;

    /*
     * Represents the time of the next reset command for each thruster. The
     * Maestro requires an arming command of 1500 microseconds at least once
     * every 2 minutes and 35 seconds. This is unspecified behavior in the
     * datasheet and has been confirmed as a defect from the manufacturer.
     * Experimentally, it has been found that this reset signal must be
     * continued for atleast 185ms.
     */
    std::map<uint8_t, ros::Time> _next_reset;

    /*
     * Specifies the maximum thrust that may be applied in either
     * direction in units of kgf.
     */
    double _max_thrust_kgf;

    /*
     * These values describe the characteristic polynomials fit
     * to the BlueRobotics T200 thruster data for both positive
     * thrust and negative thrust respectively. The equation
     * takes the following form:
     *     (signal) = A * thrust^3 + B * thrust^2 + C * thrust + D
     *
     * A plot of these polynomials can be seen in Robosub issue
     * 163. The values were derived using numpy on positive and
     * negative data from BlueRobotics separately with a cubic
     * polynomial fit.
     */
    const double a_negative = 3.13223349;
    const double b_negative = 24.45656912;
    const double c_negative = 135.16956211;
    const double d_negative = 1466.0982307;

    const double a_positive = 1.23903743;
    const double b_positive = -13.2491571;
    const double c_positive = 108.242813;
    const double d_positive = 1532.70802;

    /**
     * Maestro-defined serial bytes that have special meaning to the
     * Maestro as control signals (5.c) (5.e). This class makes use of the
     * compact protocol.
     *
     * @note The commands all require the second byte to be the channel
     *       number, then bits 7-0 of target, and then bits 13-7 of target
     *       (repeating for multiple target commands).  Exceptions to this
     *       are documented for each command.
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
         * channel and is incremented by 1 for each target sent. The first
         * following byte represents the starting servo index and the
         * second byte represents the number of servos to configure. All
         * following bytes are the incrementing target values.
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
         * maximum speed. Each value represents 0.25
         * microseconds/10ms/80ms. Values must be from 0 to 255.
         */
        SetAcceleration = 0x89,

        /*
         * Sets a specified servo channel to a PWM output. The 2 bytes
         * following the channel number indicate the on-time of the PWM and
         * the two bytes following those two specify the period of the PWM.
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
         * pending errors that the Maestro has detected. This command
         * accepts no following bytes.
         */
        GetErrors = 0xA1,

        /*
         * Sets all servos to go to the home position. This command accepts
         * no following bytes.
         */
        GoHome = 0xA2
    };

    int parseNormalized(const double speed, uint8_t &msb, uint8_t
            &lsb);

    int setPort(Serial *port);
};
}

#endif // MAESTROTHRUSTERDRIVER_H
