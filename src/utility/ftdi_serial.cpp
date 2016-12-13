#include "ftdi_serial.hpp"
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <vector>
#include <string>

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

    bool FTDISerial::open(std::vector<uint8_t>& serial_number)
    {
        serial_number.push_back(0);
        m_status = FT_OpenEx(&serial_number[0], FT_OPEN_BY_SERIAL_NUMBER,
                             &m_handle);
        if(m_status != FT_OK)
        {
            return false;
        }

        m_status |= FT_SetBitMode(m_handle, 0x00, 0);
        m_status |= FT_ResetDevice(m_handle);
        m_status |= FT_Purge(m_handle, FT_PURGE_RX | FT_PURGE_TX);
        m_status |= FT_SetDataCharacteristics(m_handle, m_bits_per_word,
                m_stop_bits, m_parity);
        m_status |= FT_SetFlowControl(m_handle, m_flow_control, 0, 0);
        m_status |= FT_SetLatencyTimer(m_handle, 2);

        if(m_status != FT_OK)
        {
            return false;
        }

        m_open = true;

        return true;
    }

    bool FTDISerial::close()
    {
        if(m_open)
        {
            m_status = FT_Close(m_handle);
            if(m_status != FT_OK)
            {
                return false;
            }

            m_open = false;
        }

        return true;
    }

    bool FTDISerial::read(std::vector<uint8_t>& msg, uint32_t num_bytes)
    {
        uint32_t bytes_read = 0;

        if(!num_bytes)
        {
            sleep(1);
            m_status = FT_GetQueueStatus(m_handle, &num_bytes);
            if(m_status != FT_OK)
            {
                return false;
            }
        }

        msg.resize(num_bytes);

        m_status = FT_Read(m_handle, &msg[0], num_bytes, &bytes_read);
        if((bytes_read < num_bytes) || (m_status != FT_OK))
        {
            return false;
        }

        return true;
    }

    bool FTDISerial::write(std::vector<uint8_t>& msg)
    {
        uint32_t bytes_written = 0;

        m_status = FT_Write(m_handle, &msg[0], msg.size(), &bytes_written);
        if((bytes_written < msg.size()) || (m_status != FT_OK)) {
            return false;
        }

        return true;
    }

    void FTDISerial::setBaudRate(uint32_t baud_rate)
    {
        m_baud_rate = baud_rate;
    }

    void FTDISerial::setBitsPerWord(FTDISerial::bits_per_word bits_per_word)
    {
        m_bits_per_word = static_cast<uint8_t>(bits_per_word);
    }

    void FTDISerial::setStopBits(FTDISerial::stop_bits stop_bits)
    {
        m_stop_bits = static_cast<uint8_t>(stop_bits);
    }

    void FTDISerial::setParity(FTDISerial::parity parity)
    {
        m_parity = static_cast<uint8_t>(parity);
    }

    void FTDISerial::setFlowControl(FTDISerial::flow flow, uint8_t x_on,
                                    uint8_t x_off)
    {
        m_flow_control = static_cast<uint16_t>(flow);
        m_flow_x_on = x_on;
        m_flow_x_off = x_off;
    }

    bool FTDISerial::populateDeviceList()
    {
        std::vector<std::vector<uint8_t>> dev_buf;
        std::vector<uint8_t*> dev_buf_ptr;
        uint32_t num_devs = 0;

        if(!FTDISerial::m_dev_list.empty() ||
                FT_ListDevices(&num_devs, NULL, FT_LIST_NUMBER_ONLY) != FT_OK)
        {
            return false;
        }

        for(uint32_t i = 0; i < num_devs; i++)
        {
            dev_buf.push_back(std::vector<uint8_t>(64));
            dev_buf_ptr.push_back(&dev_buf.back()[0]);
        }
        dev_buf_ptr.push_back(NULL);

        if(FT_ListDevices(&dev_buf_ptr[0], &num_devs,
                       FT_LIST_ALL | FT_OPEN_BY_SERIAL_NUMBER) != FT_OK)
        {
            return false;
        }

        for(uint32_t i = 0; i < num_devs; i++)
        {
            FTDISerial::m_dev_list.emplace_back(dev_buf[i].begin(),
                    dev_buf[i].end());
        }

        return true;
    }

    const std::vector<std::string>& FTDISerial::getDeviceList()
    {
        return m_dev_list;
    }
}
