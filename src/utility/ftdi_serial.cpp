#include "ftdi_serial.hpp"
#include <cstring>
#include <unistd.h>

namespace rs
{
    std::vector<std::string> FTDISerial::m_dev_list;

    FTDISerial::FTDISerial()
    {
    }

    FTDISerial::~FTDISerial()
    {
        this->close();
    }

    bool FTDISerial::open(const std::string& serial_number)
    {
        std::vector<char> sn(serial_number.c_str(),
                serial_number.c_str() + serial_number.length() + 1);

        m_status = FT_OpenEx(&sn[0], FT_OPEN_BY_SERIAL_NUMBER, &m_handle);
        if(m_status != FT_OK) {
            return false;
        }

        m_status |= FT_SetBitMode(m_handle, 0x00, 0);
        m_status |= FT_ResetDevice(m_handle);
        m_status |= FT_Purge(m_handle, FT_PURGE_RX | FT_PURGE_TX);
        m_status |= FT_SetDataCharacteristics(m_handle, m_bits_per_word,
                m_stop_bits, m_parity);
        m_status |= FT_SetFlowControl(m_handle, m_flow_control, 0, 0);
        m_status |= FT_SetLatencyTimer(m_handle, 2);

        if(m_status != FT_OK) {
            return false;
        }

        m_open = true;

        return true;
    }

    bool FTDISerial::close()
    {
        if(m_open) {
            m_status = FT_Close(m_handle);
            if(m_status != FT_OK) {
                return false;
            }

            m_open = false;
        }

        return true;
    }

    bool FTDISerial::read(std::string& msg, unsigned int numChars)
    {
        std::vector<char> buf;
        unsigned int bytesRead = 0;

        if(!numChars) {
            sleep(1);
            m_status = FT_GetQueueStatus(m_handle, &numChars);
            if(m_status != FT_OK) {
                return false;
            }
        }

        buf.reserve(numChars);

        m_status = FT_Read(m_handle, &buf[0], numChars, &bytesRead);
        if((bytesRead < numChars) || (m_status != FT_OK)) {
            return false;
        }
        
        msg = std::string{ &buf[0] };

        return true;
    }

    bool FTDISerial::write(const std::string& msg)
    {
        std::vector<char> msg_ptr(msg.c_str(),
                msg.c_str() + msg.length() + 1);
        unsigned int bytesWritten = 0;

        m_status = FT_Write(m_handle, &msg_ptr[0], msg.length(),
                &bytesWritten);
        if(m_status != FT_OK) {
            return false;
        }

        return true;
    }

    void FTDISerial::setBaudRate(unsigned int baud_rate)
    {
        m_baud_rate = baud_rate;
    }

    void FTDISerial::setBitsPerWord(FTDISerial::bits_per_word bits_per_word)
    {
        m_bits_per_word = static_cast<unsigned char>(bits_per_word);
    }

    void FTDISerial::setStopBits(FTDISerial::stop_bits stop_bits)
    {
        m_stop_bits = static_cast<unsigned char>(stop_bits);
    }

    void FTDISerial::setParity(FTDISerial::parity parity)
    {
        m_parity = static_cast<unsigned char>(parity);
    }

    void FTDISerial::setFlowControl(FTDISerial::flow flow, unsigned char x_on,
                                    unsigned char x_off)
    {
        m_flow_control = static_cast<unsigned short>(flow);
        m_flow_x_on = x_on;
        m_flow_x_off = x_off;
    }
    
    bool FTDISerial::populateDeviceList()
    {
        std::vector<std::vector<char>> dev_buf;
        std::vector<char*> dev_buf_ptr;
        int num_devs = 0;

        if(!FTDISerial::m_dev_list.empty()) {
            return false;
        }
        
        if(FT_ListDevices(&num_devs, NULL, FT_LIST_NUMBER_ONLY) != FT_OK) {
            return false;
        }

        for(int i = 0; i < num_devs; i++) {
            dev_buf.push_back(std::vector<char>(64));
            dev_buf_ptr.push_back(&dev_buf.back()[0]);
        }
        dev_buf_ptr.push_back(NULL);

        if(FT_ListDevices(&dev_buf_ptr[0], &num_devs,
                       FT_LIST_ALL | FT_OPEN_BY_SERIAL_NUMBER) != FT_OK) {
            return false;
        }

        for(int i = 0; i < num_devs; i++) {
            FTDISerial::m_dev_list.push_back(&dev_buf[i][0]);
        }

        return true;
    }

    const std::vector<std::string>& FTDISerial::getDeviceList()
    {
        return m_dev_list;
    }
}
