#include "utility/serial.hpp"

namespace rs
{
	Serial::Serial(const char *port_name, int baud_rate)
	{
		this->port_fd = open(port_name, O_RDWR | O_NOCTTY | O_ASYNC | O_NDELAY);
		if (this->port_fd < 0)
		{
			//EXIT("could not open port: " + string(port_name));
		}

		this->configure(baud_rate);
	}

	Serial::~Serial()
	{
		close(this->port_fd);
	}


	int Serial::swrite(uint8_t *buf, int num)
	{
		int i;
		int sent = 0;

		while(sent < num)
		{
			i = write(this->port_fd, buf, num);
			if (i < 0)
			{
				//ERROR("serial write error");
			}
			else
			{
				sent += i;
			}
		}

		return sent;
	}

	int Serial::sread(uint8_t *buf, int num)
	{

		int i = 0;
		int temp;

		while(i < num)
		{
			temp = read(this->port_fd, buf+i, num-i);
			if(temp == -1)
			{
			//		temp = errno;
			//		printf("%d ", errno);
		
			}
			else
				i+=temp;
			usleep((num-i)<<3);
		}

		return i;
	/*
	   int i;

			i = read(this->port_fd, buf, num);
			if(i != num)
			{
				ERROR("Serial Read Error");
			}

		return i;
		*/
	}

    int Serial::queryBuffer()
    {
        int bytes_available = 0;
        ioctl(this->port_fd, FIONREAD, &bytes_available);
        return bytes_available;
    }

	void Serial::configure(int baud_rate)
	{
		struct termios options = {0};
		int fd = this->port_fd;

		//INFO("configuring port");

		if (tcgetattr(this->port_fd, &options) != 0)
		{
			//ERROR("Failed to get attributes.");
		}

		options.c_iflag = 0; //no input handling
		options.c_oflag = 0; //no output mapping
		options.c_cflag = (options.c_cflag & ~CSIZE) | CS8; //utilize 8 bit works and clear the current word setting to be overwritten (masked)
		options.c_cflag |= CLOCAL | CREAD; //disable modem controls and enable the receiver
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
			//ERROR("Failed to set the serial port options.");
		}

		//INFO("done!");
	}
};
