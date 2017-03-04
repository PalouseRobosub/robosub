#include "utility/serial.hpp"
#include "ros/ros.h"
#include "ros/console.h"

namespace rs
{
    /**
     * Constructor.
     */
    Serial::Serial()
    {
        m_port_fd = -1;
        m_port_name = "";
        m_is_open = false;
    }

    /**
     * Deconstructor.
     */
    Serial::~Serial()
    {
        this->Close();
    }

    /**
     * Open a serial port.
     *
     * @param port_name The name of the port to open.
     * @param baud_rate The serial baud rate to open the port with.
     *
     * @return Zero upon success or a non-zero error code.
     */
    int Serial::Open(const char *port_name, int baud_rate)
    {
        //arbitrary small delay to allow ports to be created during tests
        //this is a hack, better solution would be to poll/check for a
        //certain amount of time then fail out
        ros::WallDuration(.1).sleep();

        this->m_port_fd = ::open(port_name, O_RDWR | O_NOCTTY | O_ASYNC |
                                 O_NDELAY);
        this->m_port_name = port_name;
        if (this->m_port_fd < 0)
        {
            ROS_FATAL("could not open port: %s", port_name);
            exit(1);
        }


        this->Configure(baud_rate);

        m_is_open = true;

        return 0;
    }

    /**
     * Closes a serial port.
     *
     * @return Zero upon success or a non-zero error code.
     */
    int Serial::Close()
    {
        if(m_is_open)
        {
            ::close(this->m_port_fd);
        }

        m_is_open = false;
        return 0;
    }

    /**
     * Flush the input and output buffers.
     *
     * @return Zero.
     */
    int Serial::Flush()
    {
        tcflush(this->m_port_fd, TCIOFLUSH);

        return 0;
    }

    /**
     * Write data to the serial port.
     *
     * @param buf A pointer to the data to write.
     * @param num The number of bytes to write.
     *
     * @return The number of bytes returned, or -1 if error.
     */
    int Serial::Write(uint8_t *buf, int num)
    {
        int i;
        int sent = 0;

        if(!m_is_open)
        {
            return -1;
        }

        while(sent < num && ros::ok())
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

    /**
     * Read data from the serial port.
     *
     * @param buf The location to store data.
     * @param num The number of bytes to read.
     *
     * @return The number of bytes read or -1 upon error.
     */
    int Serial::Read(uint8_t *buf, int num)
    {
        int i = 0;
        int temp;

        if(!m_is_open)
        {
            return -1;
        }

        while(i < num && ros::ok())
        {
            /*
             * Wait for atleast one byte to be available before reading so that
             * timeout is also valid if no bytes are received on the port.
             */
            ros::WallTime now = ros::WallTime::now();
            ros::WallTime exit_time = now + ros::WallDuration(0.5);
            while (QueryBuffer() == 0 && now < exit_time)
            {
                ros::WallDuration(0.02).sleep();
                now = ros::WallTime::now();
            }

            if (now > exit_time)
            {
                ROS_INFO("No serial bytes were available");
                return 0;
            }

            temp = ::read(this->m_port_fd, buf+i, num-i);
            if(temp == -1)
            {
                ROS_ERROR("Serial read error.");
            }
            else
                i+=temp;
            usleep((num-i) << 3);
        }

        return i;
    }

    /**
     * Get the number of available bytes.
     *
     * @return The number of bytes available on the serial port.
     */
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

    /**
     * Configure the opened file to function as a serial port.
     *
     * @note This function sets a default timeout of half a second for
     *       receiving any data on the port. This timeout is updated after each
     *       successful byte read.
     *
     * @param baud_rate The baud rate to configure the serial port to.
     */
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
        //utilize 8 bit works and clear the current word setting to be
        //overwritten (masked)
        options.c_cflag = (options.c_cflag & ~CSIZE) | CS8;
        //disable modem controls and enable the receiver
        options.c_cflag |= CLOCAL | CREAD;
        //options.c_cflag &= ~CRTSCTS;

        cfmakeraw(&options);

        cfsetispeed(&options, baud_rate);
        cfsetospeed(&options, baud_rate);


        //Finally figured this out. With VMIN = 255 and VTIME != 0, there is a
        //timeout (VTIME *.1 seconds) between each byte received on the UART If
        //the timeout occurs, read() will return Read will also return if 255
        //bytes are read Read will aditionally return if read()'s calling
        //readAmt is read.

        //This is the most likely series of events that result in read()
        //returning when the amount of bytes specified are read.

        options.c_cc[VMIN] = 255;
        options.c_cc[VTIME] = 5; //.5 second time out in between bytes

        if (tcsetattr(fd, TCSANOW, &options) != 0)
        {
            ROS_ERROR("Failed to set the serial port options.");
        }
    }
};
