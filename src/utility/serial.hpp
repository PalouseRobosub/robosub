#ifndef __SERIAL_H__
#define __SERIAL_H__

typedef char u8;

#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <string>

namespace rs
{
    class Serial
    {
    public:
        Serial();
        ~Serial();
        int Open(const char *port_name, int baud_rate);
        int Close();
        int Write(uint8_t *buf, int num);
        int Read(uint8_t *buf, int num);
        int QueryBuffer();

    private:
        //private methods
        void Configure(int baud_rate);

        //private members
        int m_port_fd;
        std::string m_port_name;
        bool m_is_open;
    };
};

#endif
