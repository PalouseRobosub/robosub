/**
 *TCA9545A I2C-Mux Driver.
 *
 * @author Ryan Summers
 * @date 1-31-2017
 */
#ifndef TCA9545A_H
#define TCA9545A_H

#include <Wire.h>

class Tca9545a
{
public:
    enum class Channel : uint8_t
    {
        None = 0,
        One = 0b1,
        Two = 0b10,
        Three = 0b100,
        Four = 0b1000
    };

    Tca9545a(const int reset_pin, bool a0_high, bool a1_high) :
        _reset_pin(reset_pin),
        _i2c_address(0b1110000 | ((a0_high)? 1 : 0) | ((a1_high)? 0b10 : 0))
    {
    }

    int init(Channel channel);

    int setChannel(Channel _channel);

    int reset();

private:
    Channel _current_channel;
    const int _reset_pin;
    const uint8_t _i2c_address;
};

#endif //TCA9545A_H
