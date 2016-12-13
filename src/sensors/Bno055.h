/**
 * Bno055 Class declaration
 * @author Ryan Summers
 * @date 12-1-2016
 *
 * @brief Provides class declarations for the Bno055 sensor driver.
 * @note All references refer to section and subsection (e.g. [5.3]) of the
 *       datasheet for the Bno055 sensor located at:
 *       https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST_BNO055_DS000_14.pdf
 */

#ifndef BNO055_H
#define BNO055_H

#include "utility/serial.hpp"
#include <vector>

using std::vector;

/**
 * Handle expressions for automatic return on failure.
 *
 * @param x Expression to evaluate.
 * @param ret The value to return when the expression evaluates to false.
 *
 * @return ret if the expression x evaluates false.
 */
#define AbortIfNot(x, ret) { int t = (x); if(!t) return ret; }

/**
 * Handle expressions for automatic return on success.
 *
 * @param x The expression to evaluate.
 *
 * @return x if the expression evaluates true.
 */
#define AbortIf(x) { int t = (x); if(t) return t; }

#define BreakIf(x) { int t = (x); if(t) break; }

#define ContinueIf(x) { int t = (x); if(t) continue; }

namespace rs
{
/**
 * Bno055 sensor class declaration
 */
class Bno055
{
    /*
     * The maximum number of times to reattempt a register read or write.
     */
    static constexpr int max_retries = 3;
    /*
     * Defines the times (in milliseconds) required to switch from config
     * mode to another mode and from any mode to config mode [Table 3-6].
     */
    static constexpr uint8_t to_config_switch_time_ms = 19;
    static constexpr uint8_t from_config_switch_time_ms = 7;

    /*
     * Defines the specific error codes and header responses from the Bno
     * sensor [4.7].
     */
    static constexpr uint8_t read_success_header = 0xBB;
    static constexpr uint8_t acknowledge_header = 0xEE;
    static constexpr uint8_t request_header = 0xAA;

    /*
     * Defines the Bno055's chip ID stored in the Bno memory.
     */
    static constexpr uint8_t chip_id = 0xA0;

    /**
     * Defines register address locations.
     *
     * @note Register values are only a single byte long, however there
     *       exist two separate pages. The 16th bit specifies the page in
     *       this enumeration and the lowest 16 bits specify the register
     *       address on the respective page [4.2].
     */
    enum class Register : uint16_t
    {
        MAG_RADIUS_MSB = 0x6a,
        MAG_RADIUS_LSB = 0x69,
        ACC_RADIUS_MSB = 0x68,
        ACC_RADIUS_LSB = 0x67,
        GYR_OFFSET_Z_MSB = 0x66,
        GYR_OFFSET_Z_LSB = 0x65,
        GYR_OFFSET_Y_MSB = 0x64,
        GYR_OFFSET_Y_LSB = 0x63,
        GYR_OFFSET_X_MSB = 0x62,
        GYR_OFFSET_X_LSB = 0x61,
        MAG_OFFSET_Z_MSB = 0x60,
        MAG_OFFSET_Z_LSB = 0x5f,
        MAG_OFFSET_Y_MSB = 0x5e,
        MAG_OFFSET_Y_LSB = 0x5d,
        MAG_OFFSET_X_MSB = 0x5c,
        MAG_OFFSET_X_LSB = 0x5b,
        ACC_OFFSET_Z_MSB = 0x5a,
        ACC_OFFSET_Z_LSB = 0x59,
        ACC_OFFSET_Y_MSB = 0x58,
        ACC_OFFSET_Y_LSB = 0x57,
        ACC_OFFSET_X_MSB = 0x56,
        ACC_OFFSET_X_LSB = 0x55,
        AXIS_MAP_SIGN = 0x42,
        AXIS_MAP_CONFIG = 0x41,
        TEMP_SOURCE = 0x40,
        SYS_TRIGGER = 0x3F,
        PWR_MODE = 0x3E,
        OPR_MODE = 0x3D,
        UNIT_SEL = 0x3B,
        SYS_ERROR = 0x3A,
        SYS_STATUS = 0x39,
        SYS_CLK_STATUS = 0x38,
        INT_STA = 0x37,
        ST_RESULT = 0x36,
        CALIB_STAT = 0x35,
        TEMP = 0x34,
        GRV_Data_Z_MSB = 0x33,
        GRV_Data_Z_LSB = 0x32,
        GRV_Data_Y_MSB = 0x31,
        GRV_Data_Y_LSB = 0x30,
        GRV_Data_X_MSB = 0x2f,
        GRV_Data_X_LSB = 0x2e,
        LIA_Data_Z_MSB = 0x2d,
        LIA_Data_Z_LSB = 0x2c,
        LIA_Data_Y_MSB = 0x2b,
        LIA_Data_Y_LSB = 0x2a,
        LIA_Data_X_MSB = 0x29,
        LIA_Data_X_LSB = 0x28,
        QUAT_Data_z_MSB = 0x27,
        QUAT_Data_z_LSB = 0x26,
        QUAT_Data_y_MSB = 0x25,
        QUAT_Data_y_LSB = 0x24,
        QUAT_Data_x_MSB = 0x23,
        QUAT_Data_x_LSB = 0x22,
        QUAT_Data_w_MSB = 0x21,
        QUAT_Data_w_LSB = 0x20,
        EUL_Pitch_MSB = 0x1F,
        EUL_Pitch_LSB = 0x1E,
        EUL_Roll_MSB = 0x1D,
        EUL_Roll_LSB = 0x1C,
        EUL_Heading_MSB = 0x1B,
        EUL_Heading_LSB = 0x1A,
        GYR_DATA_Z_MSB = 0x19,
        GYR_DATA_Z_LSB = 0x18,
        GYR_DATA_Y_MSB = 0x17,
        GYR_DATA_Y_LSB = 0x16,
        GYR_DATA_X_MSB = 0x15,
        GYR_DATA_X_LSB = 0x14,
        MAG_DATA_Z_MSB = 0x13,
        MAG_DATA_Z_LSB = 0x12,
        MAG_DATA_Y_MSB = 0x11,
        MAG_DATA_Y_LSB = 0x10,
        MAG_DATA_X_MSB = 0x0f,
        MAG_DATA_X_LSB = 0x0e,
        ACC_DATA_Z_MSB = 0x0d,
        ACC_DATA_Z_LSB = 0x0c,
        ACC_DATA_Y_MSB = 0x0b,
        ACC_DATA_Y_LSB = 0x0a,
        ACC_DATA_X_MSB = 0x09,
        ACC_DATA_X_LSB = 0x08,
        PAGE_ID = 0x07,
        BL_Rev_ID = 0x06,
        SW_REV_ID_MSB = 0x05,
        SW_REV_ID_LSB = 0x04,
        GYR_ID = 0x03,
        MAG_ID = 0x02,
        ACC_ID = 0x01,
        CHIP_ID = 0x00,

        /*
         * Special note: UNIQUE_ID is 16 bytes long. This is the starting
         * location of this register.
         */
        UNIQUE_ID = 0x150,
        GYR_AM_SET = 0x11f,
        GYR_AM_THRES = 0x11e,
        GYR_DUR_Z = 0x11d,
        GYR_HR_Z_SET = 0x11c,
        GYR_DUR_Y = 0x11b,
        GYR_HR_Y_SET = 0x11a,
        GYR_DUR_X = 0x11d,
        GYR_HR_X_SET = 0x118,
        GYR_INT_SETTING = 0x117,
        ACC_NM_SET = 0x116,
        ACC_NM_THRES = 0x115,
        ACC_HG_THRES = 0x114,
        ACC_HG_DURATION = 0x113,
        ACC_INT_Settings = 0x112,
        ACC_AM_THRES = 0x111,
        INT_EN = 0x110,
        INT_MASK = 0x10f,
        GYR_Sleep_Config = 0x10d,
        ACC_Sleep_Config = 0x10c,
        GYR_Config_1 = 0x10b,
        GYR_Config_0 = 0x10a,
        MAG_Config = 0x109,
        ACC_Config = 0x108
    };

public:
    /**
     * Defines operating mode that the sensor can run in [3.3].
     */
    enum class OperationMode : uint8_t
    {
        Config = 0b0000,
        Imu = 0b1000,
        Compass = 0b1001,
        M4G = 0b1010,
        NdofFmcOff = 0b1011,
        Ndof = 0b1100
    };

    /**
     * Defines the different axes available for reorientation of the
     * sensor [3.4]. The MSB is the sign bit and the following two bits
     * correspond to the axis specificatio The MSB is the sign bit and the
     * following two bits correspond to the axis specification.
     */
    enum class Axis : uint8_t
    {
        X = 0b000,
        Y = 0b001,
        Z = 0b010,
        NegativeX = 0b100,
        NegativeY = 0b1000,
        NegativeZ = 0b10000
    };

    /**
     * Enumeration of each of the different sensors present on the Bno055
     * System-In-Package (SiP).
     */
    enum class Sensor
    {
        Accelerometer,
        Magnetometer,
        Gyroscope,
        Thermometer,
        Fusion
    };

    /**
     * Constructor.
     *
     * @param port An initialized port running at 115200 baud to
     *             communicate with the Bno055.
     */
    Bno055(Serial &port) :
        _port(port),
        _current_mode(OperationMode::Config),
        _page(0)
    {
    }

    int init();

    int reset();

    int setOperationMode(OperationMode mode);

    int getSystemStatus(uint8_t &status, uint8_t &error);

    int getSystemCalibration(uint8_t &calibration);

    int getSensorCalibration(Sensor sensor, uint8_t &calibration);

    int readEuler(double &roll, double &pitch, double &yaw);

    int readQuaternion(double &w, double &x, double &y, double &z);

    int readTemperature(uint16_t &temp);

    int remapAxes(Axis x, Axis y, Axis z);

    int writeOffsets(Sensor sensor, int16_t offset_x, int16_t offset_y,
            int16_t offset_z);

    int readOffsets(Sensor sensor, int16_t &offset_x, int16_t &offset_y,
            int16_t &offset_z);

    int writeRadii(int16_t accelerometer_radius,
            int16_t magnometer_radius);
    int readRadii(int16_t &accelerometer_radius,
            int16_t &magnometer_radius);

private:
    int write_register(Bno055::Register start, vector<uint8_t> data);

    int write_register(Bno055::Register start, uint8_t data);

    int read_register(Bno055::Register start, vector<uint8_t> &data,
            uint8_t len);

    int read_register(Bno055::Register start, uint8_t &data);

    int set_page(uint8_t id);

    /*
     * Serial port to be utilized for communicating with the sensor.
     */
    Serial &_port;

    /*
     * Definition of the current operating mode of the sensor.
     */
    OperationMode _current_mode;

    /*
     * Defines the current register page that the sensor is set to.
     */
    uint8_t _page;
};
}
#endif // BNO055_H
