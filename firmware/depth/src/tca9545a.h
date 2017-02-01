/**
 *TCA9545A I2C-Mux Driver.
 *
 * @author Ryan Summers
 * @date 1-31-2017
 */
#ifndef TCA9545A_H
#define TCA9545A_H
class Tca9545a
{

public:
    enum class Channel : uint8_t
    {
        None,
        One,
        Two,
        Three,
        Four
    };

    Tca9545a(const int reset_pin, bool a0_high, bool a1_high) :
        _reset_pin(reset_pin),
        _i2c_address(0b1110000 | ((a0_high)? 1 : 0) | ((a1_high)? 0b10 : 0))
    {
    }

    int init(Channel channel)

    int setChannel(Channel _channel);

    int reset();

private:
    Channel _current_channel;
    const int _reset_pin;
    const uint8_t _i2c_address;

    int read_control_register(uint8_t &control_register);
    int write_control_register(uint8_t control_register);
}

#endif //TCA9545A_H
