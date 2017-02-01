#include "tca9545a.h"

int Tca9545a::init(Channel channel)
{
    pinMode(_reset_pin, OUTPUT);

    reset();

    if (setChannel(channel))
    {
        return -1;
    }

    return 0;
}

/**
 * Sets the mux to a specific channel selection.
 *
 * @param channel The channel to set the MUX to.
 *
 * @return Zero upon success and -1 upon error.
 */
int Tca9545a::setChannel(Channel channel)
{
    if (_current_channel == channel)
    {
        return 0;
    }

    Wire.beginTransmission(_i2c_address);

    /*
     * Ensure that one data byte is written and that the transmission completes
     * without errors.
     */
    if (Wire.write(reinterpret_cast<uint8_t>(channel)) != 1)
    {
        return -1;
    }

    if (Wire.endTransmission() != 0)
    {
        return -1;
    }

    _current_channel = channel;

    return 0;
}

/**
 * Issues a hardware reset to the device.
 *
 * @return None.
 */
void Tca9545a::reset()
{
    digitalWrite(_reset_pin, LOW);
    delay(10);
    digitalWrite(_reset_pin, HIGH);
    _current_channel = Channel::None;
}
