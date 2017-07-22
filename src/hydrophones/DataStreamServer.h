/**
 * @author Ryan Summers
 * @date 7-7-2017
 *
 * @brief Server for receiving UDP packets from a raspberry pi.
 */
#ifndef DATA_STREAM_SERVER
#define DATA_STREAM_SERVER

#include <cstdint>

class DataStreamServer
{
public:
    DataStreamServer();

    int32_t init(const uint16_t port_number);

    int32_t send_start_trigger(const uint32_t sample_count);

    int32_t get_packet(char *buf, const uint16_t max_len);

private:
    /**
     * Specifies the socket file descriptor.
     */
    uint32_t socket_fd;
};
#endif
