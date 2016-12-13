#ifndef __ROBOSUB_FTDI_SERIAL_H__
#define __ROBOSUB_FTDI_SERIAL_H__

#include <cstdint>
#include <ftd2xx.h>
#include <string>
#include <vector>

namespace rs
{
class FTDISerial
{
public:
    enum class bits_per_word : uint8_t
    {
        EIGHT = FT_BITS_8,
        SEVEN = FT_BITS_7
    };

    enum class stop_bits : uint8_t
    {
        ONE = FT_STOP_BITS_1,
        TWO = FT_STOP_BITS_2
    };

    enum class parity : uint8_t
    {
        NONE = FT_PARITY_NONE,
        ODD = FT_PARITY_ODD,
        EVEN = FT_PARITY_EVEN,
        MARK = FT_PARITY_MARK,
        SPACE = FT_PARITY_SPACE
    };

    enum class flow : uint16_t
    {
        NONE = FT_FLOW_NONE,
        RTS_CTS = FT_FLOW_RTS_CTS,
        DTR_DSR = FT_FLOW_DTR_DSR,
        XON_XOFF = FT_FLOW_XON_XOFF
    };

public:
    FTDISerial();
    ~FTDISerial();
    bool open(std::vector<uint8_t>& serial_number);
    bool close();
    bool read(std::vector<uint8_t>& msg, uint32_t numChars = 0);
    bool write(std::vector<uint8_t>& msg);
    void setBaudRate(uint32_t baud_rate);
    void setBitsPerWord(FTDISerial::bits_per_word bits_per_word);
    void setStopBits(FTDISerial::stop_bits stop_bits);
    void setParity(FTDISerial::parity parity);
    void setFlowControl(FTDISerial::flow flow, uint8_t x_on = '\0',
                        uint8_t x_off = '\0');

    static bool populateDeviceList();
    static const std::vector<std::string>& getDeviceList();

private:
    FT_HANDLE m_handle;
    FT_STATUS m_status;
    uint32_t m_baud_rate = 9600;
    uint8_t m_bits_per_word =
        static_cast<uint8_t>(bits_per_word::EIGHT);
    uint8_t m_stop_bits = static_cast<uint8_t>(stop_bits::ONE);
    uint8_t m_parity = static_cast<uint8_t>(parity::NONE);
    uint16_t m_flow_control =
        static_cast<uint16_t>(flow::NONE);
    uint8_t m_flow_x_on = '\0';
    uint8_t m_flow_x_off = '\0';
    bool m_open = false;
    static std::vector<std::string> m_dev_list;
};
}

#endif
