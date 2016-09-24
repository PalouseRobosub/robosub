#include "utility/serial.hpp"
#include "ros/ros.h"
#include "ros/console.h"

namespace rs
{
    Serial::Serial()
    {
        m_port_fd = -1;
        m_port_name = "";
        m_is_open = false;
    }

    Serial::~Serial()
    {
        this->Close();
    }

    int Serial::Open(const char *port_name, int baud_rate)
    {
        this->m_port_fd = ::open(port_name, O_RDWR | O_NOCTTY | O_ASYNC | O_NDELAY);
        this->m_port_name = port_name;
        if (this->m_port_fd < 0)
        {
            ROS_FATAL("could not open port: %s", port_name);
            ros::shutdown();
        }


        this->Configure(baud_rate);

        m_is_open = true;

        return 0;
    }

    int Serial::Close()
    {
        if(m_is_open)
        {
            ::close(this->m_port_fd);
        }

        m_is_open = false;
        return 0;
    }

    int Serial::Write(uint8_t *buf, int num)
    {
        int i;
        int sent = 0;

        if(!m_is_open)
        {
            return -1;
        }

        while(sent < num)
        {
            i = ::write(this->m_port_fd, buf, num);
            if (i < 0)
            {
                ROS_ERROR("serial write error");
            }
            else
            {
                sent += i;
            }
        }

        return sent;
    }

    int Serial::Read(uint8_t *buf, int num)
    {

        int i = 0;
        int temp;

        if(!m_is_open)
        {
            return -1;
        }

        while(i < num)
        {
            temp = ::read(this->m_port_fd, buf+i, num-i);
            if(temp == -1)
            {
                ROS_ERROR("serial read error");
            }
            else
                i+=temp;
            usleep((num-i)<<3);
        }

        return i;
    }

    int Serial::QueryBuffer()
    {
        if(!m_is_open)
        {
            return -1;
        }

        int bytes_available = 0;
        ioctl(this->m_port_fd, FIONREAD, &bytes_available);
        return bytes_available;
    }

    void Serial::Configure(int baud_rate)
    {
        struct termios options = {0};
        int fd = this->m_port_fd;

        ROS_INFO("configuring serial port");

        if (tcgetattr(this->m_port_fd, &options) != 0)
        {
            ROS_ERROR("Failed to get serial attributes.");
        }

        options.c_iflag = 0; //no input handling
        options.c_oflag = 0; //no output mapping
        options.c_cflag = (options.c_cflag & ~CSIZE) |
                          CS8; //utilize 8 bit works and clear the current word setting to be overwritten (masked)
        options.c_cflag |= CLOCAL |
                           CREAD; //disable modem controls and enable the receiver
        //options.c_cflag &= ~CRTSCTS;

        cfmakeraw(&options);

        cfsetispeed(&options, baud_rate);
        cfsetospeed(&options, baud_rate);


        //Finally figured this out. With VMIN = 255 and VTIME != 0, there is a timeout (VTIME *.1 seconds) between each byte received on the UART
        //If the timeout occurs, read() will return
        //Read will also return if 255 bytes are read
        //Read will aditionally return if read()'s calling readAmt is read.

        //This is the most likely series of events that result in read() returning when the amount of bytes specified are read.

        options.c_cc[VMIN] = 255;
        options.c_cc[VTIME] = 5; //.5 second time out in between bytes

        if (tcsetattr(fd, TCSANOW, &options) != 0)
        {
            ROS_ERROR("Failed to set the serial port options.");
        }
    }
};
