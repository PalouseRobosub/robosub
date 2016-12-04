#include "Bno055.h"

namespace rs
{
    /**
     * Initialize the Bno to a default configuration state.
     *
     * @note This function needs a minimum of 1.15 seconds to execute because
     *       the Bno055 self-test requires time to complete and reset requires
     *       650ms.
     *
     * @return Zero upon success and non-zero upon failure.
     */
    int Bno055::init()
    {
        uint8_t self_test_result, chip_id, clock_status, clock_config;
        AbortIf(set_page(0));

        /*
         * Perform a software reset to start off in a default state.
         */
        AbortIf(reset());

        /*
         * Sanity check that this is the chip we expect it to be. Result
         * derived from [4.3.1].
         */
        AbortIf(read_register(Bno055::Register::CHIP_ID, chip_id));
        AbortIfNot(chip_id == Bno055::chip_id, -1);

        /*
         * Validate that the power on self-test passed for all sensors.
         */
        AbortIf(read_register(Bno055::Register::ST_RESULT, self_test_result));
        AbortIfNot(self_test_result  == 0x0F, -1);

        /*
         * Begin a built-in self-test of the device and wait for it to
         * complete.
         */
        uint8_t sys_trigger_reg;
        AbortIf(read_register(Bno055::Register::SYS_TRIGGER, sys_trigger_reg));
        AbortIf(write_register(Bno055::Register::SYS_TRIGGER,
                sys_trigger_reg | 1));
        usleep(500000);
        AbortIf(read_register(Bno055::Register::ST_RESULT,
                self_test_result));
        AbortIfNot(self_test_result == 0x0F, -1);

        /*
         * Read and configure the clock to utilize the external oscillator if
         * the clock is able to be configured.
         */
        AbortIf(read_register(Bno055::Register::SYS_CLK_STATUS, clock_status));
        if (clock_status & 0b1)
        {
            AbortIf(write_register(Bno055::Register::SYS_TRIGGER, 1<<7));
        }

        /*
         * Set the power and operating mode to configuration defaults.
         */
        AbortIf(setPowerMode(Bno055::PowerMode::Normal));
        AbortIf(setOperationMode(Bno055::OperationMode::Config));

        return 0;
    }

    /**
     * Sets the current power setting of the sensor.
     *
     * @param mode The power mode to set the sensor to.
     *
     * @return Zero upon success or a non-zero error code returned by the
     *         sensor.
     */
    int Bno055::setPowerMode(PowerMode mode)
    {
        AbortIf(write_register(Bno055::Register::PWR_MODE,
                static_cast<uint8_t>(mode)));

        return 0;
    }

    /**
     * Sets the current operating mode of the sensor.
     *
     * @param mode The mode to switch to.
     *
     * @return Zero upon success or a non-zero error code supplied by the
     *         sensor.
     */
    int Bno055::setOperationMode(Bno055::OperationMode mode)
    {
        AbortIf(write_register(Bno055::Register::OPR_MODE,
                static_cast<uint8_t>(mode)));
        const uint8_t wait_time_ms =
            (mode == Bno055::OperationMode::Config)? to_config_switch_time_ms :
            from_config_switch_time_ms;

        if (mode != _current_mode)
        {
            usleep(1000 * wait_time_ms);
        }
        _current_mode = mode;

        return 0;
    }

    /**
     * Sets the pitch mode to follow Android or Windows OS pitch definitions.
     *
     * @param mode The pitch convention to use.
     *
     * @return Zero upon success or a non-zero error code supplied by the
     *         sensor.
     */
    int Bno055::setPitchMode(Bno055::PitchMode mode)
    {
        /*
         * Read the current unit selection register, set the pitch mode bit to
         * a one or zero, and write the value back to the sensor.
         */
        uint8_t unit_select;
        AbortIf(read_register(Bno055::Register::UNIT_SEL, unit_select));
        unit_select &= ~(1<<7);
        unit_select |= static_cast<uint8_t>(mode);
        AbortIf(write_register(Bno055::Register::UNIT_SEL, unit_select));

        return 0;
    }

    /**
     * Sets the output format units for a specific sensor.
     *
     * @param sensor The sensor to configure units for. The magnometer units are not adjustable.
     * @param format The specified format to update the specified sensor to.
     *
     * @return Zero upon success or a non-zero error code.
     */
    int Bno055::setOutputFormat(Bno055::Sensor sensor, Bno055::Format format)
    {
        uint8_t unit_select;
        AbortIf(read_register(Bno055::Register::UNIT_SEL, unit_select));
        switch (sensor)
        {
            case Bno055::Sensor::Accelerometer:
                AbortIfNot(format == Bno055::Format::MetersPerSecondSquared ||
                        format == Bno055::Format::MilliG, -1);
                unit_select &= ~(1<<1);
                unit_select |= static_cast<uint8_t>(format);
                break;
            case Bno055::Sensor::Gyroscope:
                AbortIfNot(format == Bno055::Format::DegreesPerSec ||
                        format == Bno055::Format::RadiansPerSec, -1);
                unit_select &= ~(1<<2);
                unit_select |= static_cast<uint8_t>(format);
                break;
            case Bno055::Sensor::Fusion:
                AbortIfNot(format == Bno055::Format::EulerDegrees ||
                        format == Bno055::Format::EulerRadians, -1);
                unit_select &= ~(1<<3);
                unit_select |= static_cast<uint8_t>(format);
                euler_units = format;
                break;
            case Bno055::Sensor::Thermometer:
                AbortIfNot(format == Bno055::Format::TempC ||
                        format == Bno055::Format::TempF, -1);
                unit_select &= ~(1<<5);
                unit_select |= static_cast<uint8_t>(format);
                break;
            default:
                return -1;
                break;
        }

        AbortIf(write_register(Bno055::Register::UNIT_SEL, unit_select));
        return 0;
    }

    /**
     * Read raw accelerometer data.
     *
     * @note Units are set separately and are defined in [Table 3-17].
     *
     * @param[out] x Location to store x result.
     * @param[out] y Location to store y result.
     * @param[out] z Location to store z result.
     *
     * @return Zero upon success or non-zero error code.
     */
    int Bno055::readAccelerometer(int16_t &x, int16_t &y, int16_t &z)
    {
        vector<uint8_t> data;
        AbortIf(read_register(Bno055::Register::ACC_DATA_X_LSB, data, 6));
        x = data[0] | static_cast<uint16_t>(data[1]) << 8;
        y = data[2] | static_cast<uint16_t>(data[3]) << 8;
        z = data[4] | static_cast<uint16_t>(data[5]) << 8;

        return 0;
    }

    /**
     * Read raw magnometer data.
     *
     * @note Units are in microTeslas.
     *
     * @param[out] x The location to store x result.
     * @param[out] y The location to store y result.
     * @param[out] z The location to store z result.
     *
     * @return Zero upon success or non-zero error code.
     */
    int Bno055::readMagnometer(int16_t &x, int16_t &y, int16_t &z)
    {
        vector<uint8_t> data;
        AbortIf(read_register(Bno055::Register::MAG_DATA_X_LSB, data, 6));
        x = data[0] | static_cast<uint16_t>(data[1]) << 8;
        y = data[2] | static_cast<uint16_t>(data[3]) << 8;
        z = data[4] | static_cast<uint16_t>(data[5]) << 8;

        return 0;
    }

    /**
     * Read raw gyroscope data.
     *
     * @note Units are set separately and are defined in [Table 3-22].
     *
     * @param[out] x The location to store x result.
     * @param[out] y The location to store y result.
     * @param[out] z The location to store z result.
     *
     * @return Zero upon success or non-zero error code.
     */
    int Bno055::readGyroscope(int16_t &x, int16_t &y, int16_t &z)
    {
        vector<uint8_t> data;
        AbortIf(read_register(Bno055::Register::GYR_DATA_X_LSB, data, 6));
        x = data[0] | static_cast<uint16_t>(data[1]) << 8;
        y = data[2] | static_cast<uint16_t>(data[3]) << 8;
        z = data[4] | static_cast<uint16_t>(data[5]) << 8;

        return 0;
    }

    /**
     * Read the euler orientation information.
     *
     * @note The units of these outputs are specified in [Table 3-29] and set
     *       by the setFormat function.
     *
     * @param[out] roll Location to store roll reading.
     * @param[out] pitch Location to store pitch reading.
     * @param[out] yaw Location to store yaw reading.
     */
    int Bno055::readEuler(double &roll, double &pitch, double &yaw)
    {
        vector<uint8_t> data;
        int16_t raw_yaw, raw_roll, raw_pitch;
        AbortIf(read_register(Bno055::Register::EUL_Heading_LSB, data, 6));
        raw_yaw = data[0] | static_cast<uint16_t>(data[1]) << 8;
        raw_roll = data[2] | static_cast<uint16_t>(data[3]) << 8;
        raw_pitch = data[4] | static_cast<uint16_t>(data[5]) << 8;

        /*
         * Scale raw readings by scale factors defined in [Table 3-28].
         */
        roll = raw_roll * ((euler_units == Bno055::Format::EulerDegrees)?
                1.0/16 : 1.0/900);
        yaw = raw_yaw * ((euler_units == Bno055::Format::EulerDegrees)?
                1.0/16 : 1.0/900);
        pitch = raw_pitch * ((euler_units == Bno055::Format::EulerDegrees)?
                1.0/16 : 1.0/900);

        return 0;
    }

    /**
     * Read the quaternion orientation information.
     *
     * @param[out] w Location to store w reading.
     * @param[out] x Location to store x reading.
     * @param[out] y Location to store y reading.
     * @param[out] z Location to store z reading.
     */
    int Bno055::readQuaternion(double &w, double &x, double &y, double &z)
    {
        vector<uint8_t> data;
        AbortIf(read_register(Bno055::Register::QUAT_Data_w_LSB, data, 8));

        /*
         * Scale quaternion output into normalized form by the factor defined
         * in [Table 3-31].
         */
        w = static_cast<int16_t>(data[0] |
                static_cast<uint16_t>(data[1]) << 8) * 1.0 / (1<<14);
        x = static_cast<int16_t>(data[2] |
                static_cast<uint16_t>(data[3]) << 8) * 1.0 / (1<<14);
        y = static_cast<int16_t>(data[4] |
                static_cast<uint16_t>(data[5]) << 8) * 1.0 / (1<<14);
        z = static_cast<int16_t>(data[6] |
                static_cast<uint16_t>(data[7]) << 8) * 1.0 / (1<<14);

        return 0;
    }

    /**
     * Read the temperature sensor.
     *
     * @note Units of this reading are set elsewhere and are described in
     *       [Table 3-37].
     *
     * @param[out] temp Location to store temperature reading.
     *
     * @return Zero upon success or non-zero error code.
     */
    int Bno055::readTemperature(uint16_t &temp)
    {
        uint8_t temperature;
        AbortIf(read_register(Bno055::Register::TEMP, temperature));
        temp = temperature;
        return 0;
    }

    /**
     * Remap the sensor axes definitions.
     *
     * @note A description of the remap definitions is available in [3.4].
     *
     * @param x Axis to remap X to.
     * @param y Axis to remap Y to.
     * @param z Axis to remap Z to.
     *
     * @return Zero upon sucess or non-zero error code.
     */
    int Bno055::remapAxes(Axis x, Axis y, Axis z)
    {
        vector<uint8_t> axis_data = {0, 0};
        axis_data[0] = static_cast<uint8_t>(z) & 0b11 << 4 |
                static_cast<uint8_t>(y) & 0b11 << 2 |
                static_cast<uint8_t>(x) & 0b11;
        axis_data[1] = ((static_cast<uint8_t>(x) > 0b10)? 1 << 2 : 0) |
                ((static_cast<uint8_t>(y) > 0b10)? 1 << 1 : 0) |
                ((static_cast<uint8_t>(x) > 0b10)? 1 : 0);
        AbortIf(write_register( Bno055::Register::AXIS_MAP_CONFIG, axis_data));

        return 0;
    }

    /**
     * Writes a sensor offset from a calibration profile to the sensor.
     *
     * @param sensor The sensor offset to modify. This value can only be the
     *        Gyroscope, Magnometer, or Accelerometer.
     * @param offset_x Offset of the x axis.
     * @param offset_y Offset of the y axis.
     * @param offset_z Offset of the z axis.
     *
     * @return Zero upon success or a non-zero error code.
     */
    int Bno055::writeOffsets(Sensor sensor, int16_t offset_x, int16_t offset_y,
            int16_t offset_z)
    {
        vector<uint8_t> offsets(6);
        offsets[0] = static_cast<uint8_t>(offset_x);
        offsets[1] = static_cast<uint8_t>(offset_x >> 8);
        offsets[2] = static_cast<uint8_t>(offset_y);
        offsets[3] = static_cast<uint8_t>(offset_y >> 8);
        offsets[4] = static_cast<uint8_t>(offset_z);
        offsets[5] = static_cast<uint8_t>(offset_z >> 8);
        switch (sensor)
        {
            case Bno055::Sensor::Accelerometer:
                AbortIf(write_register(Bno055::Register::ACC_OFFSET_X_LSB,
                        offsets));
                break;
            case Bno055::Sensor::Gyroscope:
                AbortIf(write_register(Bno055::Register::GYR_OFFSET_X_LSB,
                        offsets));
                break;
            case Bno055::Sensor::Magnometer:
                AbortIf(write_register(Bno055::Register::MAG_OFFSET_X_LSB,
                        offsets));
                break;
            default:
                return -1;
                break;
        }

        return 0;
    }

    /**
     * Read offsets calculated by a sensor calibration.
     *
     * @param sensor The sensor whose calibration information should be read.
     *        This value can only by the Gyroscope, Magnometer, or the
     *        Accelerometer.
     * @param[out] offset_x Location to store the offset along the x axis.
     * @param[out] offset_y Location to store the offset along the y axis.
     * @param[out] offset_z Location to store the offset along the z axis.
     *
     * @return Zero upon success or non-zero error code.
     */
    int Bno055::readOffsets(Sensor sensor, int16_t &offset_x,
            int16_t &offset_y, int16_t &offset_z)
    {
        vector<uint8_t> offsets(6);
        switch (sensor)
        {
            case Bno055::Sensor::Accelerometer:
                AbortIf(read_register(Bno055::Register::ACC_OFFSET_X_LSB,
                        offsets, 6));
                break;
            case Bno055::Sensor::Gyroscope:
                AbortIf(read_register(Bno055::Register::GYR_OFFSET_X_LSB,
                        offsets, 6));
                break;
            case Bno055::Sensor::Magnometer:
                AbortIf(read_register(Bno055::Register::MAG_OFFSET_X_LSB,
                        offsets, 6));
                break;
            default:
                return -1;
                break;
        }
        offset_x = static_cast<uint16_t>(offsets[1]) << 8 | offsets[0];
        offset_y = static_cast<uint16_t>(offsets[3]) << 8 | offsets[2];
        offset_z = static_cast<uint16_t>(offsets[5]) << 8 | offsets[4];

        return 0;
    }

    /**
     * Write radii to a sensor calibration.
     *
     * @param sensor The sensor whose calibration information should be read.
     *        This value can only be set for the Magnometer or the
     *        Accelerometer.
     * @param radius Radius to store in memory.
     *
     * @return Zero upon success or non-zero error code.
     */
    int Bno055::writeRadius(Sensor sensor, int16_t radius)
    {
        vector<uint8_t> _radius(2);
        _radius[0] = static_cast<uint8_t>(radius);
        _radius[1] = static_cast<uint8_t>(radius >> 8);
        switch (sensor)
        {
            case Bno055::Sensor::Accelerometer:
                AbortIf(write_register(Bno055::Register::ACC_RADIUS_LSB,
                        _radius));
                break;
            case Bno055::Sensor::Magnometer:
                AbortIf(write_register(Bno055::Register::MAG_RADIUS_LSB,
                        _radius));
                break;
            default:
                return -1;
                break;
        }

        return 0;
    }

    /**
     * Read radii calculated by a sensor calibration.
     *
     * @param sensor The sensor whose calibration information should be read.
     *        This value can only be the Magnometer or the Accelerometer.
     * @param[out] radius Location to store the radius.
     *
     * @return Zero upon success or non-zero error code.
     */
    int Bno055::readRadius(Sensor sensor, int16_t &radius)
    {
        vector<uint8_t> _radius(2);
        switch (sensor)
        {
            case Bno055::Sensor::Accelerometer:
                AbortIf(read_register(Bno055::Register::ACC_RADIUS_LSB,
                        _radius, 2));
                break;
            case Bno055::Sensor::Magnometer:
                AbortIf(read_register(Bno055::Register::MAG_RADIUS_LSB,
                        _radius, 2));
                break;
            default:
                return -1;
                break;
        }
        radius = static_cast<uint16_t>(_radius[1]) << 8 | _radius[0];

        return 0;
    }

    /**
     * Get the current calibration status of the system.
     *
     * @param[out] calibration The location to store the calibration status of
     *             the system.
     * @return Zero upon success or a non-zero error code.
     */
    int Bno055::getSystemCalibration(uint8_t &calibration)
    {
        uint8_t calib_stat_reg, calib_stat;
        AbortIf(read_register(Bno055::Register::CALIB_STAT,
            calib_stat_reg));
        calibration = ((calib_stat_reg >> 6) & 0b11);
        return 0;
    }

    /**
     * Get the current calibration status of a sensor.
     *
     * @param sensor The sensor to check calibration status on.
     * @param[out] calibration The location to store the detected calibration
     *             status.
     *
     * @return Zero upon success, otherwise a non-zero error code is returned.
     */
    int Bno055::getSensorCalibration(Sensor sensor, uint8_t &calibration)
    {
        uint8_t calib_stat_reg = 0;
        AbortIfNot(sensor == Bno055::Sensor::Accelerometer ||
                sensor == Bno055::Sensor::Magnometer ||
                sensor == Bno055::Sensor::Gyroscope, -1);

        AbortIf(read_register(Bno055::Register::CALIB_STAT,
                calib_stat_reg));
        switch (sensor)
        {
            case Bno055::Sensor::Gyroscope:
                calibration = (calib_stat_reg >> 4) & 0b11;
                break;
            case Bno055::Sensor::Accelerometer:
                calibration = (calib_stat_reg >> 2) & 0b11;
                break;
            case Bno055::Sensor::Magnometer:
                calibration = (calib_stat_reg >> 0) & 0b11;
                break;
            default:
                return -1;
                break;
        }

        return 0;
    }

    /**
     * Resets the sensor to power-on default values.
     *
     * @note This function requires a 650ms wait to enter normal power mode
     *       after a reset.
     *
     * @return Zero upon success or a non-zero error code.
     */
    int Bno055::reset()
    {
        uint8_t chip_id = 0;
        AbortIf(write_register(Bno055::Register::SYS_TRIGGER, 1<<5));
        while (chip_id != Bno055::chip_id)
        {
            read_register(Bno055::Register::CHIP_ID, chip_id);
        }

        return 0;
    }

    /**
     * Write a specific register within the BNO.
     *
     * @param start The register to write to..
     * @param data The value to write to the register.
     *
     * @return Zero on success and non-zero on error.
     */
    int Bno055::write_register(Bno055::Register start, uint8_t data)
    {
        /*
         * Ensure that the correct page is set for the specified register.
         */
        uint16_t register_address = static_cast<uint16_t>(start);
        if ((register_address & 0x100>>2) != _page)
        {
            AbortIf(set_page(register_address & 0x100 >> 2));
        }

        /*
         * Construct the message and transmit the write request.
         */
        vector<uint8_t> msg = {Bno055::request_header, 0x00,
                static_cast<uint8_t>(start), 1, data};
        AbortIfNot(_port.Write(msg.data(), msg.size()) == msg.size(), -1);

        /*
         * If a reset was just triggered, a reply is not given from the sensor.
         * It is necessary to wait for the POR to trigger.
         */
        if (start == Bno055::Register::SYS_TRIGGER && data == 1<<5)
        {
            /*
             * Wait a ms for the device to fully enter reset. After that, read
             * all bytes in the buffer. The Bno cannot send a whole reply
             * packet if a reset is triggered.
             */
            usleep(1000);
            uint8_t bytes_avail = _port.QueryBuffer();
            vector<uint8_t> response(bytes_avail);
            AbortIfNot(_port.Read(response.data(), bytes_avail) ==
                    bytes_avail, -1);

            /*
             * Ensure only a partial message or a success was returned. Then,
             * wait the necessary 650ms to be successfully powered up again.
             */
            AbortIfNot(bytes_avail == 0 ||
                    response[0] == Bno055::write_response_header, -1);
            AbortIfNot(bytes_avail <= 1 || response[1] != 1, -1);
            return 0;
        }

        /*
         * Verify that the write succeeded with the Bno's reply.
         */
        uint8_t response[2];
        AbortIfNot(_port.Read(response, 2) == 2, -1);

        AbortIfNot(response[0] == Bno055::write_response_header, -1);

        return ((response[1] == 0x1)? 0 : response[1]);
    }

    /**
     * Write to specific registers within the BNO.
     *
     * @note Beware that it is illegal to write a reset using this function and
     *       doing so will result in undefined behavior.
     *
     * @param start The register to begin writing to.
     * @param data A vector containing the data to begin writing at register
     *        start.
     *
     * @return Zero on success and non-zero on error.
     */
    int Bno055::write_register(Bno055::Register start, vector<uint8_t> data)
    {
        /*
         * Ensure that the correct page is set for the specified register.
         */
        uint16_t register_address = static_cast<uint16_t>(start);
        if ((register_address & 0x100>>2) != _page)
        {
            AbortIf(set_page(register_address & 0x100 >> 2));
        }

        /*
         * Verify that the write is of proper length, construct the message,
         * and transmit the write request.
         */
        uint8_t write_length = data.size();
        AbortIfNot(write_length < 128, -1);

        vector<uint8_t> msg = {Bno055::request_header, 0x00,
                static_cast<uint8_t>(start), write_length};
        msg.insert(msg.end(), data.begin(), data.end());
        AbortIfNot(_port.Write(msg.data(), msg.size()) == msg.size(), -1);

        /*
         * Verify that the write succeeded with the Bno's reply.
         */
        uint8_t response[2];
        AbortIfNot(_port.Read(response, 2) == 2, -1);

        AbortIfNot(response[0] == Bno055::write_response_header, -1);

        return ((response[1] == 0x1)? 0 : response[1]);
    }

    /**
     * Read a specific register.
     *
     * @param start The register to write to.
     * @param data The value to write to the register.
     *
     * @return Zero upon success or a non-zero error code.
     */
    int Bno055::read_register(Bno055::Register start, uint8_t &data)
    {
        /*
         * Ensure that the correct page is set for the specified register.
         */
        uint16_t register_address = static_cast<uint16_t>(start);
        if ((register_address & 0x100>>2) != _page)
        {
            AbortIf(set_page(register_address & 0x100 >> 2));
        }

        /*
         * Allocate memory for a response, request a read, and read the
         * response.
         */
        vector<uint8_t> reply(3);
        vector<uint8_t> request = {Bno055::request_header, 0x01,
                static_cast<uint8_t>(start), 1};
        AbortIfNot(_port.Write(request.data(),
                request.size()) == request.size(), -1);
        AbortIfNot(_port.Read(reply.data(), 3) >= 2, -1);
        AbortIfNot(reply[0] == Bno055::read_success_header,
                static_cast<int>(reply[1]));
        AbortIfNot(reply[1] == 1, -1);

        data = reply[2];

        return 0;
    }

    /**
     * Read an arbitrary number of registers.
     *
     * @param start The starting location to begin reading registers from.
     * @param data A vector to store the read register information into.
     * @param len The number of registers to read.
     *
     * @return Zero upon success or a non-zero Bno-supplied error code.
     */
    int Bno055::read_register(Bno055::Register start, vector<uint8_t> &data,
            uint8_t len)
    {
        /*
         * Ensure that the correct page is set for the specified register.
         */
        uint16_t register_address = static_cast<uint16_t>(start);
        if ((register_address & 0x100>>2) != _page)
        {
            AbortIf(set_page(register_address & 0x100 >> 2));
        }

        /*
         * Allocate memory for a response, request a read, and read the
         * response.
         */
        uint8_t read_length;
        vector<uint8_t> reply(len + 2);
        vector<uint8_t> request = {Bno055::request_header, 0x01,
                static_cast<uint8_t>(start), len};
        AbortIfNot(_port.Write(request.data(), request.size()) ==
                request.size(), -1);
        /*
         * Ensure that atleast an error code can be read. Verify later that
         * entire length was read.
         */
        AbortIfNot((read_length = _port.Read(reply.data(), len + 2)) >= 2, -1);
        AbortIfNot(reply[0] == Bno055::read_success_header,
                static_cast<int>(reply[1]));
        AbortIfNot(read_length == len + 2, -1)
        AbortIfNot(reply[1] == len, -1);

        data.clear();
        data.insert(data.begin(), reply.begin()+2, reply.end());

        return 0;
    }

    /**
     * Sets the current register page selection on the Bno.
     *
     * @param id The page ID to change to. This value must be zero or 1.
     *
     * @return Zero upon success or a non-zero error code.
     */
    int Bno055::set_page(uint8_t id)
    {
        AbortIfNot(id == 1 || id == 0, -1);
        AbortIf(write_register(Bno055::Register::PAGE_ID, id));
        _page = id;
        return 0;
    }

    /**
     * Grab the system status and any potential error codes.
     *
     * @note The meaning of status and error codes can be found in [4.3.58] and
     *       [4.3.59].
     *
     * @param[out] status The current status of the status_register
     * @param[out] error The error code read by the sensor.
     *
     * @return Zero upon success or a non-zero error code.
     */
    int Bno055::getSystemStatus(uint8_t &status, uint8_t &error)
    {
        AbortIf(read_register(Bno055::Register::SYS_STATUS, status));
        if (status == 1)
        {
            AbortIf(read_register(Bno055::Register::SYS_ERROR, error));
        }

        return 0;
    }
}
