#ifndef __ROBOSUB_FTDI_SERIAL_H__
#define __ROBOSUB_FTDI_SERIAL_H__

#include <ftd2xx.h>
#include <string>
#include <vector>

namespace rs
{
    class FTDISerial
    {
    public:
        enum class bits_per_word : unsigned char
        {
            EIGHT = FT_BITS_8,
            SEVEN = FT_BITS_7
        };

        enum class stop_bits : unsigned char
        {
            ONE = FT_STOP_BITS_1,
            TWO = FT_STOP_BITS_2
        };

        enum class parity : unsigned char
        {
            NONE = FT_PARITY_NONE,
            ODD = FT_PARITY_ODD,
            EVEN = FT_PARITY_EVEN,
            MARK = FT_PARITY_MARK,
            SPACE = FT_PARITY_SPACE
        };

        enum class flow : unsigned short
        {
            NONE = FT_FLOW_NONE,
            RTS_CTS = FT_FLOW_RTS_CTS,
            DTR_DSR = FT_FLOW_DTR_DSR,
            XON_XOFF = FT_FLOW_XON_XOFF
        };

    public:
        FTDISerial();
        ~FTDISerial();
        bool open(const std::string& serial_number);
        bool close();
        bool read(std::string& msg, unsigned int numChars = 0);
        bool write(const std::string& msg);
        void setBaudRate(DWORD baud_rate);
        void setBitsPerWord(FTDISerial::bits_per_word bits_per_word);
        void setStopBits(FTDISerial::stop_bits stop_bits);
        void setParity(FTDISerial::parity parity);
        void setFlowControl(FTDISerial::flow flow, unsigned char x_on = '\0',
                            unsigned char x_off = '\0');

        static bool populateDeviceList();
        static const std::vector<std::string>& getDeviceList();

    private:
        FT_HANDLE m_handle;
        FT_STATUS m_status;
        DWORD m_baud_rate = 9600;
        unsigned char m_bits_per_word =
            static_cast<unsigned char>(bits_per_word::EIGHT);
        unsigned char m_stop_bits = static_cast<unsigned char>(stop_bits::ONE);
        unsigned char m_parity = static_cast<unsigned char>(parity::NONE);
        unsigned short m_flow_control =
            static_cast<unsigned short>(flow::NONE);
        unsigned char m_flow_x_on = '\0';
        unsigned char m_flow_x_off = '\0';
        bool m_open = false;
        static std::vector<std::string> m_dev_list;
    };
}

#endif
