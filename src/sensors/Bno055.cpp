#include "Bno055.h"

namespace rs
{
    int Bno055::init()
    {
        write_register(Bno055::Register::PAGE_ID, 0);
        _current_page = 0;
        write_register(Bno055::Register::PWR_MODE, static_cast<uint8_t>(Bno055::PowerMode::Normal));
        write_register(Bno055::Register::OPR_MODE, static_cast<uint8_t>(Bno055::OperationMode::Config));
        
    }

    int Bno055::setPowerMode(PowerMode mode)
    {

    }

    int Bno055::write_register(Bno055::Register start, vector<uint8_t> data)
    {
        /*
         * Ensure that the correct page is set for the specified register.
         */
        uint16_t register_address = reinterpret_Cast<uint16_t>(start);
        if (start & 0x100 != _current_page)
        {
            if (write_register(Bno::Register::PAGE_ID, (start & 0x100)? 1 : 0))
            {
                return -1;
            }
        }

        uint8_t write_length = data.size();
        if (write_length >= 128) return -1;

        vector<uint8_t> msg = {0xAA, 0x00};
        msg.push_back(static_cast<uint8_t>(start));
        msg.push_back(write_length);
        msg.insert(msg.end(), data.begin(), data.end());

        return (port.write(msg.data(), msg.size()) != msg.size());
    }

    int Bno055::read_register(Bno055::Register start, vector<uint8_t> &data,
            uint8_t len)
    {
        /*
         * Allocate memory for a response, request a read, and read the
         * response.
         */
        vector<uint8_t> reply(len + 2);
        vector<uint8_t> request = {0xAA, 0x01, static_cast<uint8_t>(start),
                len};
        const size_t bytes_written = port.write(request.data(),
                request.size());
        if (bytes_written != request.size())
        {
            return -1;
        }
        const size_t bytes_read = port.read(reply.data(), len + 2);

        /*
         * Atleast 2 bytes should always be read, even in case of errors [4.7].
         */
        if (bytes_read < 2)
        {
            return -1;
        }

        /*
         * The proper reply header is 0xEE from [4.7]. 0xBB is sent on error.
         */
        if (reply[0] != 0xEE)
        {
            return reinterpret_cast<int>(reply[1]);
        }

        /*
         * Exit if an unexpected number of bytes are read.
         */
        if (bytes_read != len + 2 || reply[1] != len)
        {
            return -1;
        }

        data.clear();
        data.insert(data.begin(), reply.start()+2, reply.end());
        return 0;
    }
}
