#include "Bno055.h"

namespace rs
{
    /**
     * Initialize the Bno to a default configuration state.
     *
     * @return Zero upon success and non-zero upon failure.
     */
    int Bno055::init()
    {
        AbortIf(set_page(0));
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
        AbortIf(write_register(Bno055::Register::PWR_MODE, mode));

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
        AbortIf(write_register(Bno055::Register::OPR_MODE, mode));
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
    int Bno055::SetOutputFormat(Bno055::Sensor sensor, Bno055::Format format)
    {
        uint8_t unit_select;
        AbortIf(read_register(Bno055::Register::UNIT_SEL, unit_select));
        switch (sensor)
        {
            case Bno055::Sensor::Accelerometer:
                AbortIfNot(format == Bno055::Format::MetersPerSecondSquared ||
                        format == Bno055::Format::MilliG, -1);
                unit_select &= ~(1<<1);
                uint_select |= static_cast<uint8_t>(format);
                break;
            case Bno055::Sensor::Gyroscope:
                AbortIfNot(format == Bno055::Format::DegreesPerSecond ||
                        format == Bno055::Format::RadiansPerSec, -1);
                unit_select &= ~(1<<2);
                uint_select |= static_cast<uint8_t>(format);
                break;
            case Bno055::Sensor::Fusion:
                AbortIfNot(format == Bno055::Format::EulerDegrees ||
                        format == Bno055::Format::EulerRadians, -1);
                uint_select &= ~(1<<3);
                uint_select |= static_cast<uint8_t>(format);
                break;
            case Bno055::Sensor::Thermometer:
                AbortIfNot(format == Bno055::Format::TempC ||
                        format == Bno055::Format::TempF, -1);
                uint_select &= ~(1<<5);
                uint_select |= static_cast<uint8_t>(format);
                break;
            default:
                return -1;
                break;
        }

        AbortIf(write_register(Bno055::Register::UNIT_SEL, unit_select));
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
        uint16_t register_address = reinterpret_Cast<uint16_t>(start);
        if ((register_address & 0x100>>2) != _current_page)
        {
            AbortIf(set_page(regster_address & 0x100 >> 2));
        }

        /*
         * Construct the message and transmit the write request.
         */
        vector<uint8_t> msg =
                {0xAA, 0x00, static_cast<uint8_t>(start), 1, data};
        AbortIfNot(port.write(msg.data(), msg.size()) == msg.size(), -1);

        /*
         * Verify that the write succeeded with the Bno's reply.
         */
        uint8_t response[2];
        AbortIfNot(port.read(response, 2) == 2, -1);

        AbortIfNot(response[0] == 0xEE, -1);

        return ((response[1] == 0x1)? 0 : response[1]);
    }

    /**
     * Write to specific registers within the BNO.
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
        uint16_t register_address = reinterpret_Cast<uint16_t>(start);
        if ((register_address & 0x100>>2) != _current_page)
        {
            AbortIf(set_page(regster_address & 0x100 >> 2));
        }

        /*
         * Verify that the write is of proper length, construct the message,
         * and transmit the write request.
         */
        uint8_t write_length = data.size();
        AbortIfNot(write_length < 128, -1);

        vector<uint8_t> msg =
                {0xAA, 0x00, static_cast<uint8_t>(start), write_length};
        msg.insert(msg.end(), data.begin(), data.end());
        AbortIfNot(port.write(msg.data(), msg.size()) == msg.size(), -1);

        /*
         * Verify that the write succeeded with the Bno's reply.
         */
        uint8_t response[2];
        AbortIfNot(port.read(response, 2) == 2, -1);

        AbortIfNot(response[0] == 0xEE, -1);

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
    int Bno055::read_register(Bno055::Register start, uint8_t data)
    {
        /*
         * Ensure that the correct page is set for the specified register.
         */
        uint16_t register_address = reinterpret_Cast<uint16_t>(start);
        if ((register_address & 0x100>>2) != _current_page)
        {
            AbortIf(set_page(regster_address & 0x100 >> 2));
        }

        /*
         * Allocate memory for a response, request a read, and read the
         * response.
         */
        vector<uint8_t> reply(len + 2);
        vector<uint8_t> request = {0xAA, 0x01, static_cast<uint8_t>(start), 1};
        AbortIfNot(port.write(request.data(), request.size() == request.size(),
                -1);
        AbortIfNot(port.read(reply.data(), len + 2) == len + 2, -1);
        AbortIfNot(reply[0] == 0xEE, reinterpret_cast<int>(reply[1]));
        AbortIfNot(reply[1] == len, -1);

        data.clear();
        data.insert(data.begin(), reply.start()+2, reply.end());

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
        uint16_t register_address = reinterpret_Cast<uint16_t>(start);
        if ((register_address & 0x100>>2) != _current_page)
        {
            AbortIf(set_page(regster_address & 0x100 >> 2));
        }

        /*
         * Allocate memory for a response, request a read, and read the
         * response.
         */
        vector<uint8_t> reply(len + 2);
        vector<uint8_t> request = {0xAA, 0x01, static_cast<uint8_t>(start),
                len};
        const size_t bytes_written = port.write(request.data(),
                request.size());
        AbortIfNot(bytes_written == request.size(), -1);
        AbortIfNot(port.read(reply.data(), len + 2) == len + 2, -1);
        AbortIfNot(reply[0] == 0xEE, reinterpret_cast<int>(reply[1]));
        AbortIfNot(reply[1] == len, -1);

        data.clear();
        data.insert(data.begin(), reply.start()+2, reply.end());

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
        _current_page = id;
        return 0;
    }

}
