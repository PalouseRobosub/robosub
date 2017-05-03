#ifndef PNITRAX_H
#define PNITRAX_H

#include <string>
#include "utility/serial.hpp"

using std::string;
using namespace rs;

class PniTrax
{
public:
    /**
     * Specifies different commands for the TRAX to execute.
     *
     * These commands and their descriptions can be found in the
     * TRAX AHRS user manual [Table 7-2].
     */
    enum class Command : uint8_t
    {
        kGetModInfo = 0x01,
        kGetModInfoResp = 0x02,
        kSetDataComponents = 0x03,
        kGetData = 0x04,
        kGetDataResp = 0x05,
        kSetConfig = 0x06,
        kGetConfig = 0x07,
        kGetConfigResp = 0x08,
        kSave = 0x09,
        kStartCal = 0x0A,
        kStopCal = 0x0B,
        ksetFIRFilters = 0x0C,
        kGetFIRFilters = 0x0D,
        kGetFIRFiltersResp = 0x0E,
        kPowerDown = 0x0F,
        kSaveDone = 0x10,
        kUserCalSampleCount = 0x11,
        kUserCalScore = 0x12,
        kSetConfigDone = 0x13,
        kSetFIRFiltersDone = 0x14,
        kStartContinuousMode = 0x15,
        kStopContinuousMode = 0x16,
        kPowerUpDone = 0x17,
        kSetAcqParams = 0x18,
        kGetAcqParams = 0x19,
        kSetAcqParamsDone = 0x1A,
        kGetAcqParamsResp = 0x1B,
        kPowerDownDone = 0x1C,
        kFactoryMagCoeff = 0x1D,
        kFactoryMagCoeffDone = 0x1E,
        kTakeUserCalSample = 0x1F,
        kFactoryAccelCoeff = 0x24,
        kFactoryAccelCoeffDone = 0x25,
        kSetFunctionalMode = 0x4F,
        kGetFunctionalMode = 0x50,
        kGetFunctionalModeResp = 0x51,
        kSetResetRef = 0x6E,
        kSetMagThruthMethod = 0x77,
        kGetMagThruthMethod = 0x78,
        kGetMagThruthMethodResp = 0x79
    };

    enum class Config : uint8_t
    {
        kUserCalAutoSampling = 13
    };

    enum class Component : uint8_t
    {
        kHeading = 0x05,
        kPitch = 0x18,
        kRoll = 0x19,
        kHeadingStatus = 0x4F,
        kCalStatus = 0x09,
        kAccelX = 0x15,
        kAccelY = 0x16,
        kAccelZ = 0x17,
        kMagX = 0x1B,
        kMagY = 0x1C,
        kMagZ = 0x1D,
        kGyroX = 0x4B,
        kGyroY = 0x4C,
        kGyroZ = 0x4D
    };

    enum class Format
    {
        None,
        Raw,
        Euler
    };

    /**
     * Specifies the sensor data output mode.
     */
    enum class Mode : uint8_t
    {
        Compass = 0,
        AHRS = 1
    };

    /**
     * Specifies the type of calibration that should be performed.
     */
    enum class Calibration : uint8_t
    {
        FullRange = 0x0A,
        TwoD = 0x14,
        HardIron = 0x1E,
        LimitedTilt = 0x28,
        AccelOnly = 0x64,
        MagAccel = 0x6E
    };

    PniTrax();

    int init(const string serial_port_name, const Mode mode = Mode::AHRS);

    int setMode(const Mode mode);

    int getRPY(float &roll, float &pitch, float &yaw,
               uint8_t &yaw_accuracy, bool &calibrated);

    int getRawReadings(float &accel_x, float &accel_y, float &accel_z,
               float &gyro_x, float &gyro_y, float &gyro_z,
               float &mag_x, float &mag_y, float &mag_z);

    int startCalibration(const Calibration type,
            const bool auto_sample = false);

    int stopCalibration();

    int takeCalibrationPoint();

    int ackCalibrationPoint();

    int finishCalibration(float &mag_score, float &accel_score,
            float &distribution_error, float &tilt_error, float &tilt_range);

    int resetMagnetometerReference();

    int setOutput(const Format format);

    int save_configuration();

    int flush();

private:
    /*
     * The serial port object used for communicating with the sensor.
     */
    Serial serial_port;

    /*
     * Specified true if the sensor is currently in user-calibration mode.
     */
    bool calibrating;

    /*
     * Specified true if the sensor is automatically collecting calibration
     * points.
     */
    bool automatic_calibration;

    /*
     * Specifies the output format of the sensor.
     */
    Format output_format;

    uint16_t crc16(uint8_t *data, int length);

    float ntohf(float data);

    int write_command(const Command cmd, const uint8_t *payload,
            const uint16_t payload_length);

    int read_command(Command &cmd, uint8_t *payload,
            const uint16_t max_payload_length);
};

#endif // PNITRAX_H

