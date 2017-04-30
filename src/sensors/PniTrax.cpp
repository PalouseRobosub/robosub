#include "sensors/PniTrax.h"

namespace robosub
{
/**
 * Constructor.
 */
PniTrax::PniTrax() :
    serial_port(),
    calibrating(false),
    automatic_calibration(false),
    output_format(Format::None)
{
}

/**
 * Initializes the TRAX sensor to output data.
 *
 * @param serial_port_name The name of the serial port to use to talk to the
 *        sensor.
 * @param mode The mode in which the sensor should operate in.
 *
 * @return Zero upon success and -1 upon error.
 */
int PniTrax::init(const string serial_port_name, Mode mode)
{
    Command response;

    if (serial_port.init(serial_port_name.c_str(), B38400))
    {
        return -1;
    }

    /*
     * Verify that the TRAX is functioning in big-endian mode as anticipated
     * by reading the device ID.
     */
    if (write_command(Command::kGetModInfo, NULL, 0))
    {
        return -1;
    }

    uint8_t data[256] = {0};
    if (read_command(response, data, 256) || response != Command::kGetModInfoResp)
    {
        return -1;
    }

    if (string(data, 4) != "TRAX")
    {
        return -1;
    }

    /*
     * Set the polled acquisition parameters.
     */
    float delay = 0;
    data[0] = 1;
    data[1] = true;
    memcpy(&data[6], &delay, sizeof(delay));

    if (write_command(Command::kSetAcqParams, data, 10))
    {
        return -1;
    }

    if (read_command(response, 0 NULL) || response != Command::kSetAcqParamsDone)
    {
        return -1;
    }

    continuous = _ continuous;

    if (setOutput(Format::Euler))
    {
        return -1;
    }

    return 0;
}

/**
 * Configures the sensor with euler and raw data output formats.
 *
 * @note The IMU has a custom data output interface. This function sets up two
 *       custom interfaces used by the class.
 *
 * @param format The specified format required.
 *
 * @return Zero upon success and -1 upon error.
 */
int PniTrax::setOutput(const Output format)
{
    if (format == Format::Euler && output_format != Format::Euler)
    {
        uint8_t data[6] = {0};

        data[0] = 5;
        data[1] = reinterpret_cast<uint8_t>(Component::kHeading);
        data[2] = reinterpret_cast<uint8_t>(Component::kPitch);
        data[3] = reinterpret_cast<uint8_t>(Component::kRoll);
        data[4] = reinterpret_cast<uint8_t>(Component::kHeadingStatus);
        data[5] = reinterpret_cast<uint8_t>(Component::kCalStatus);

        if (write_command(Command::kSetDataComponents, data, 6))
        {
            return -1;
        }

        output_format = Format::Euler;
    }
    else if (format == Format::Raw && output_format != Format::Raw)
    {
        uint8_t data[10] = {0};

        data[0] = 9;
        data[1] = reinterpret_cast<uint8_t>(Component::kAccelX);
        data[2] = reinterpret_cast<uint8_t>(Component::kAccelY);
        data[3] = reinterpret_cast<uint8_t>(Component::kAccelZ);
        data[4] = reinterpret_cast<uint8_t>(Component::kMagX);
        data[5] = reinterpret_cast<uint8_t>(Component::kMagY);
        data[6] = reinterpret_cast<uint8_t>(Component::kMagZ);
        data[7] = reinterpret_cast<uint8_t>(Component::kGyroX);
        data[8] = reinterpret_cast<uint8_t>(Component::kGyroY);
        data[9] = reinterpret_cast<uint8_t>(Component::kGyroZ);

        if (write_command(Command::kSetDataComponents, data, 10))
        {
            return -1;
        }

        output_format = Format::Raw;
    }

    return 0;
}

/**
 * Reads the roll, pitch, and yaw from the IMU.
 *
 * @note The units returned for roll, pitch, and yaw are in degrees.
 *
 * @param[out] roll The location to store the roll.
 * @param[out] pitch The location to store the roll.
 * @param[out] yaw The location to store the roll.
 * @param[out] yaw_accuracy A measure of the yaw's accuracy. 1 := accurate to 2
 *             degrees, 2:= 2-10 degrees of uncertainty, and 3:= greater than
 *             10 degrees of uncertainty.
 * @param[out] calibrated Zero means not user calibrated, and non-zero indicates
 *             calibration is completed.
 *
 * @return Zero upon success and -1 upon error.
 */
int PniTrax::getRPY(float &roll, float &pitch, float &yaw,
        uint8_t &yaw_accuracy, bool &calibrated)
{
    if (setOutput(Format::Euler))
    {
        return -1;
    }

    if (write_command(Command::kGetData, NULL, 0))
    {
        return -1;
    }

    uint8_t data[18] = {0};
    if (read_command(response, data, 18) || response != Command::kGetDataResp)
    {
        return -1;
    }

    memcpy(&yaw, &data[0], 4);
    memcpy(&pitch, &data[4], 4);
    memcpy(&roll, &data[8], 4);
    yaw_accuracy = data[12];
    calibrated = data[13];

    return 0;
}

/**
 * Reads raw data for each of the sensors within the TRAX sensor.
 *
 * @note Units for the accelerometer are in Gs, gyroscope is in rad/sec, and the
 *       magnetometer is recorded in microTeslas.
 *
 * @param accel_x Location to store the X-axis of the accelerometer.
 * @param accel_y Location to store the Y-axis of the accelerometer.
 * @param accel_z Location to store the Z-axis of the accelerometer.
 * @param gyro_x Location to store the X-axis of the gyroscope.
 * @param gyro_y Location to store the Y-axis of the gyroscope.
 * @param gyro_z Location to store the Z-axis of the gyroscope.
 * @param mag_x Location to store the X-axis of the magnetometer.
 * @param mag_y Location to store the Y-axis of the magnetometer.
 * @param mag_z Location to store the Z-axis of the magnetometer.

 * @return Zero upon success and -1 upon error.
 */
int PniTrax::getRawReadings(float &accel_x, float &accel_y, float &accel_z,
        float &gyro_x, float &gyro_y, float &gyro_z, float &mag_x, float &mag_y,
        float &mag_z)
{
    /*
     * Ensure that the output format is in raw mode.
     */
    if (setOutput(Format::Raw))
    {
        return -1;
    }

    /*
     * Request a sample from the IMU.
     */
    if (write_command(Command::kGetData, NULL, 0))
    {
        return -1;
    }

    /*
     * Read the data response from the sensor.
     */
    uint8_t data[36];
    if (read_command(response, data, 36) || response != Command::kGetDataResp)
    {
        return -1;
    }

    /*
     * Move the data from the buffer into the respective memory locations.
     */
    memcpy(&accel_x, &data[0], 4);
    memcpy(&accel_y, &data[4], 4);
    memcpy(&accel_z, &data[8], 4);

    memcpy(&mag_x, &data[12], 4);
    memcpy(&mag_y, &data[16], 4);
    memcpy(&mag_z, &data[20], 4);

    memcpy(&gyro_x, &data[24], 4);
    memcpy(&gyro_y, &data[28], 4);
    memcpy(&gyro_z, &data[32], 4);

    return 0;
}

/**
 * Starts a calibration of the TRAX.
 *
 * @param type The type of calibration to start.
 * @param auto_sample Specified true if the TRAX should automatically take
 *        points. Automatic sampling is not recommended.
 *
 * @return Zero upon success and -1 upon error.
 */
int PniTrax::startCalibration(const Calibraton type, const bool auto_sample)
{
    uint8_t data[2] = {reinterpret_cast<uint8_t>(Config::kUserCalAutoSampling),
        (auto_sample)? 1 : 0};
    if (write_command(Command:kSetConfig, data, 2))
    {
        return -1;
    }

    autonomatic_calibration = auto_sample;

    uint8_t cal_type = reinterpret_cast<uint8_t>(type);
    if (write_command(Command::kStartCal, &cal_type, 1))
    {
        return -1;
    }

    return 0;
}

/**
 * Forces a stop of the current calibration profile.
 *
 * @return Zero upon success and -1 upon error.
 */
int PniTrax::StopCalibration(float &mag_score, float &accel_score,
        float &distribution_error, float &tilt_error, float &tilt_range)
{
    if (write_command(Command::kStopCal, NULL, 0))
    {
        return -1;
    }

    return 0;
}

/**
 * Acquires information about a final calibration once it has completed.
 *
 * @note This function can only succeed after all data points have been
 *       collected.  The default number of data points is 12.
 *
 * @param[out] mag_score The magnetometer score. Acceptable range is <=1 for
 *             full-range calibration and <= 2 for any other.
 * @param[out] accel_score The accelerometer score. Acceptable range is <= 1.
 * @param[out] distribution_error Should be zero. Non-zero indicates the points
 *             were not good for calculating a calibration.
 * @param[out] tilt_error Should be zero. Non-zero indicates insufficient tilt
 *             was reached during calibration.
 * @param[out] tilt_range This returns the total maximum range of tilt as
 *             averaged over the maximum positive and negative pitch and roll.
 *             For full-range and hard-iron calibration, this should be >= 45.
 *             For other calibrations, it should be approximately 2.
 *
 * @return Zero upon success and -1 upon error.
 */
int PniTrax::finishCalibration(float &mag_score, float &accel_score,
        float &distribution_error, float &tilt_error, float &tilt_range)
{
    /*
     * Attempt to read the reply from a completed calibration.
     */
    uint8_t data[24];
    Command response;
    if (read_command(response, data, 24) || response != Command::kUserCalScore)
    {
        return -1;
    }

    /*
     * Copy the parameter fields from the buffer.
     */
    memcpy(mag_score, &data[0], 4);
    memcpy(accel_score, &data[8], 4);
    memcpy(distribution_error, &data[12], 4);
    memcpy(tilt_error, &data[16], 4);
    memcpy(tilt_range, &data[20], 4);

    return 0;
}

/**
 * Takes a manual calibration data point.
 *
 * @note This will fail if the calibration profile is set to automatic mode.
 *
 * @return Zero upon success and -1 upon error.
 */
int PniTrax::takeCalibrationPoint()
{
    if (automatic_calibration)
    {
        return -1;
    }

    if (write_command(Command::kTakeUserCalSample, NULL, 0))
    {
        return -1;
    }

    if (ackCalibrationPoint())
    {
        return -1;
    }

    return 0;
}

/**
 * Acknowledges a calibration point has been read.
 *
 * @note This is done automatically when operating in non-automatic sample mode.
 *
 * @return Zero upon success and -1 upon error.
 */
int PniTrax::ackCalibrationPoint()
{
    Command response;
    if (read_command(response, NULL, 0) ||
            response != Command::kUserCalSampleCount)
    {
        return -1;
    }

    return 0;
}

/**
 * Resets the magnetometer field reference.
 *
 * @note Only call this function when the sensor is in a magnetically-pure
 *       environment.
 *
 * @return Zero upon success and -1 upon error.
 */
int PniTrax::resetMagnetometerReference()
{
    if (write_command(Command::kSetResetRef, NULL, 0))
    {
        return -1;
    }

    return 0;
}

/**
 * Saves the current configuration into the PNI TRAX memory.
 *
 * @return Zero upon success and -1 upon error.
 */
int PniTrax::save_configuration()
{
    if (write_command(Command::kSave, NULL, 0))
    {
        return -1;
    }

    return 0;
}

/**
 * Flushes the serial port to the sensor.
 *
 * @return Zero upon success and -1 upon error.
 */
int PniTrax::flush()
{
    return ((serial_port.Flush())? -1 : 0);
}

/**
 * Writes a command to the PNI TRAX using PNI Binary protocol.
 *
 * @param cmd The command to write to the sensor.
 * @param payload The payload of the command to write.
 * @param payload_len The length of data in bytes pointed to by the payload.
 *
 * @param Return zero upon success and -1 upon error.
 */
int PniTrax::write_command(const Command cmd, const uint8_t *payload,
        const uint16_t payload_len)
{
    /*
     * Construct the PNI Binary packet protocol.
     */
    int ret = 0;
    uint16_t packet_len = payload_len + 5;
    uint8_t *packet = new uint8_t[packet_len];

    /*
     * Write the byte length into the first two bytes of the packet, and then
     * write the command FRAME_ID byte.
     */
    memcpy(packet, &packet_len, 2);
    packet[2] = reinterpret_cast<uint8_t>(cmd);

    /*
     * Follow the packet data with the payload for the command.
     */
    memcpy(&packet[3], payload, payload_len);

    /*
     * Calculate the CRC-16 for the packet and append it to the end.
     */
    const uint16_t crc = crc16(packet, packet_len - 2);
    memcpy(&packet[packet_len - 3], &crc, 2);

    /*
     * Write the data down the serial port.
     */
    if (serial_port.Write(packet, packet_len) != packet_len)
    {
        ret = -1;
    }

    delete [] packet;

    return ret;
}

/**
 * Reads a command from the sensor with a PNI Binary format.
 *
 * @param[out] resp The location to store the resulting response.
 * @param[out] payload The location to store the payload of the command.
 * @param max_payload_length The maximum data that can be written to payload.
 *
 * @return Zero upon success and -1 upon error.
 */
int PniTrax::read_command(Command &resp, uint8_t *payload,
        const uint8_t max_payload_len)
{
    /*
     * Read the packet header (packet length stored as uint16_t)
     */
    uint8_t data[4096];
    if (serial_port.read(data, 2) != 2)
    {
        return -1;
    }

    uint16_t packet_len;
    memcpy(&packet_len, data, 2);

    /*
     * Packets are limited to 4096 bytes in length.
     */
    if (packet_len > 4096)
    {
        return -1;
    }

    /*
     * Read the remainder of the packet.
     */
    if (serial_port.Read(&data[2], packet_len - 2) != packet_len - 2)
    {
        return -1;
    }

    /*
     * Calculate and check the crc16 on the packet.
     */
    uint16_t crc;
    memcpy(&data[packet_len - 3], &crc, 2);

    if (crc != crc16(data, packet_len))
    {
        return -1;
    }

    /*
     * Ensure that the data will fit within the provided buffer.
     */
    if (packet_len - 5 > max_payload_length)
    {
        return -1;
    }

    /*
     * Copy the fields into the user-provided buffers.
     */
    memcpy(&resp, &data[2], 1);
    memcpy(payload, &data[3], packet_len - 5);

    return 0;
}

/**
 * Checksum calculator.
 *
 * @param data A pointer to the buffer to be summed.
 * @param length The length of data in bytes pointed to by data.
 *
 * @return The final checksum value.
 */
uint16_t crc16(uint8_t *data, const int length)
{
    uint16_t checksum = 0;
    for (int i = 0; i < length; ++i)
    {
        checksum += data[i];
    }

    return checksum;
}
}
