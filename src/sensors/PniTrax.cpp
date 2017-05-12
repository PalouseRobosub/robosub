#include "sensors/PniTrax.h"

#include <string>
#include <netinet/in.h>

using std::string;

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
 * Copies a float from network endianness to host endianness.
 *
 * @param data A float stored in network endianness.
 *
 * @return A float stored in host endianness.
 */
float PniTrax::ntohf(float data)
{
    float result;

    /*
     * Check to see if host system is big-endian. If so, memcpy is functional.
     */
    if (htonl(47) == 47)
    {
        memcpy(&result, &data, 4);
    }
    else
    {
        char *result_ptr = reinterpret_cast<char *>(&result);
        char *data_ptr = reinterpret_cast<char *>(&data);
        result_ptr[0] = data_ptr[3];
        result_ptr[1] = data_ptr[2];
        result_ptr[2] = data_ptr[1];
        result_ptr[3] = data_ptr[0];
    }

    return result;
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

    if (serial_port.Open(serial_port_name.c_str(), B38400))
    {
        return -1;
    }

    usleep(500000);
    if (flush())
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
    if (read_command(response, data, 256) ||
            response != Command::kGetModInfoResp)
    {
        return -1;
    }

    string product(reinterpret_cast<char *>(data), 4);
    if (product != "TRAX")
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

    if (read_command(response, NULL, 0) ||
            response != Command::kSetAcqParamsDone)
    {
        return -1;
    }

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
int PniTrax::setOutput(const Format format)
{
    /*
     * Ignore redundant changes in output format.
     */
    if (output_format == format)
    {
        return 0;
    }

    if (format == Format::Euler)
    {
        uint8_t data[6] = {0};

        data[0] = 5;
        data[1] = static_cast<uint8_t>(Component::kHeading);
        data[2] = static_cast<uint8_t>(Component::kPitch);
        data[3] = static_cast<uint8_t>(Component::kRoll);
        data[4] = static_cast<uint8_t>(Component::kHeadingStatus);
        data[5] = static_cast<uint8_t>(Component::kCalStatus);

        if (write_command(Command::kSetDataComponents, data, 6))
        {
            return -1;
        }
    }
    else if (format == Format::Raw)
    {
        uint8_t data[10] = {0};

        data[0] = 9;
        data[1] = static_cast<uint8_t>(Component::kAccelX);
        data[2] = static_cast<uint8_t>(Component::kAccelY);
        data[3] = static_cast<uint8_t>(Component::kAccelZ);
        data[4] = static_cast<uint8_t>(Component::kMagX);
        data[5] = static_cast<uint8_t>(Component::kMagY);
        data[6] = static_cast<uint8_t>(Component::kMagZ);
        data[7] = static_cast<uint8_t>(Component::kGyroX);
        data[8] = static_cast<uint8_t>(Component::kGyroY);
        data[9] = static_cast<uint8_t>(Component::kGyroZ);

        if (write_command(Command::kSetDataComponents, data, 10))
        {
            return -1;
        }
    }

    output_format = format;
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

    uint8_t data[200] = {0};
    Command response;
    if (read_command(response, data, 200) || response != Command::kGetDataResp)
    {
        return -1;
    }

    if (data[0] != 5 || data[1] != static_cast<uint8_t>(Component::kHeading) ||
        data[6] != static_cast<uint8_t>(Component::kPitch) ||
        data[11] != static_cast<uint8_t>(Component::kRoll) ||
        data[16] != static_cast<uint8_t>(Component::kHeadingStatus) ||
        data[18] != static_cast<uint8_t>(Component::kCalStatus))
    {
        output_format = Format::None;
        return -1;
    }

    float temp;
    memcpy(&temp, &data[2], 4);
    yaw = ntohf(temp);
    memcpy(&temp, &data[7], 4);
    pitch = ntohf(temp);
    memcpy(&temp, &data[12], 4);
    roll = ntohf(temp);

    yaw_accuracy = data[17];
    calibrated = data[19];

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
    uint8_t data[200];
    Command response;
    if (read_command(response, data, 200) || response != Command::kGetDataResp)
    {
        return -1;
    }

    if (data[0] != 9 || data[1] != static_cast<uint8_t>(Component::kAccelX) ||
                        data[6] != static_cast<uint8_t>(Component::kAccelY) ||
                        data[11] != static_cast<uint8_t>(Component::kAccelZ) ||
                        data[16] != static_cast<uint8_t>(Component::kMagX) ||
                        data[21] != static_cast<uint8_t>(Component::kMagY) ||
                        data[26] != static_cast<uint8_t>(Component::kMagZ) ||
                        data[31] != static_cast<uint8_t>(Component::kGyroX) ||
                        data[36] != static_cast<uint8_t>(Component::kGyroY) ||
                        data[41] != static_cast<uint8_t>(Component::kGyroZ))
    {
        output_format = Format::None;
        return -1;
    }

    /*
     * Move the data from the buffer into the respective memory locations.
     */
    memcpy(&accel_x, &data[2], 4);
    memcpy(&accel_y, &data[7], 4);
    memcpy(&accel_z, &data[12], 4);

    memcpy(&mag_x, &data[17], 4);
    memcpy(&mag_y, &data[22], 4);
    memcpy(&mag_z, &data[27], 4);

    memcpy(&gyro_x, &data[32], 4);
    memcpy(&gyro_y, &data[37], 4);
    memcpy(&gyro_z, &data[42], 4);

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
int PniTrax::startCalibration(const Calibration type, const bool auto_sample)
{
    uint8_t data[2] = { static_cast<uint8_t>(Config::kUserCalAutoSampling),
        auto_sample};

    if (write_command(Command::kSetConfig, data, 2))
    {
        return -1;
    }

    automatic_calibration = auto_sample;

    uint8_t cal_type = static_cast<uint8_t>(type);
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
int PniTrax::stopCalibration()
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
    memcpy(&mag_score, &data[0], 4);
    memcpy(&accel_score, &data[8], 4);
    memcpy(&distribution_error, &data[12], 4);
    memcpy(&tilt_error, &data[16], 4);
    memcpy(&tilt_range, &data[20], 4);

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
 *       environment. Aka, when the sensor is only within Earth's magnetic field
 *       and will experience no other distortions.
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
    return write_command(Command::kSave, NULL, 0);
}

/**
 * Flushes the serial port to the sensor.
 *
 * @return Zero upon success and -1 upon error.
 */
int PniTrax::flush()
{
    return serial_port.Flush();
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
    const uint16_t packet_len_ne = htons(packet_len);
    memcpy(packet, &packet_len_ne, 2);
    packet[2] = static_cast<uint8_t>(cmd);

    /*
     * Follow the packet data with the payload for the command.
     */
    memcpy(&packet[3], payload, payload_len);

    /*
     * Calculate the CRC-16 for the packet and append it to the end.
     */
    const uint16_t crc = htons(crc16(packet, packet_len - 2));
    memcpy(&packet[packet_len - 2], &crc, 2);

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
        const uint16_t max_payload_length)
{
    /*
     * Read the packet header (packet length stored as uint16_t)
     */
    uint8_t data[4096];
    if (serial_port.Read(data, 2) != 2)
    {
        return -1;
    }

    uint16_t packet_len;
    memcpy(&packet_len, data, 2);
    packet_len = ntohs(packet_len);

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
    memcpy(&crc, &data[packet_len - 2], 2);
    crc = ntohs(crc);

    if (crc != crc16(data, packet_len - 2))
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
 * CRC16 CCITT Calculator.
 *
 * @note Code was taken from:
 * https://github.com/gtrafimenkov/pycrc16/blob/master/src/_crc16module.c
 *
 * @param data A pointer to the buffer to be summed.
 * @param length The length of data in bytes pointed to by data.
 *
 * @return The final checksum value.
 */
static const uint16_t crc_table[256] = {
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
        0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
        0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
        0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
        0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
        0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
        0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
        0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
        0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
        0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
        0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
        0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
        0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
        0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
        0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
        0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
        0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
        0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
        0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
        0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
        0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
        0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
        0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
        0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
        0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
        0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
        0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
        0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
        0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
        0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
        0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
        0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};
uint16_t PniTrax::crc16(uint8_t *data, int length)
{
    uint16_t crc = 0x0000;
    for (int i = 0; i < length; ++i)
    {
        crc = ((crc << 8) & 0xFF00) ^ (crc_table[((crc >> 8) & 0xFF)
                ^ data[i]]);
    }

    return crc & 0xFFFF;
}
