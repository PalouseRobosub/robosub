#include "DataStreamServer.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <cstring>
#include "AnalogPacket.h"

/**
 * Constructor.
 */
DataStreamServer::DataStreamServer() :
    socket_fd(0)
{
}

/**
 * Initializes the data stream server.
 *
 * @param port_number The UDP port number to listen on.
 *
 * @return Zero upon success.
 */
int32_t DataStreamServer::init(const uint16_t dest_port)
{
    /*
     * Build the socket.
     */
    socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd < 0)
    {
        return -1;
    }

    /*
     * Configure the socket to bind to an internet address even
     * in use. This allows for easy rebinding if the program is
     * killed.
     */
    int optval = 1;
    if (setsockopt(socket_fd,
                   SOL_SOCKET,
                   SO_REUSEADDR,
                   reinterpret_cast<const void *>(&optval),
                   sizeof(int)) < 0)
    {
        return -1;
    }

    /*
     * Set a recv timeout so we don't block forever.
     */
    struct timeval tv;
    tv.tv_sec = 5;
    tv.tv_usec = 0;
    if (setsockopt(socket_fd,
                   SOL_SOCKET,
                   SO_RCVTIMEO,
                   reinterpret_cast<char *>(&tv),
                   sizeof(tv)) < 0)
    {
        return -1;
    }

    /*
     * Connect to the external device.
     */
    struct sockaddr_in connection_address = {0};
    connection_address.sin_family = AF_INET;
    connection_address.sin_port = htons(dest_port);
    connection_address.sin_addr.s_addr = inet_addr("172.168.0.253");

    if (connect(socket_fd, reinterpret_cast<struct sockaddr *>(&connection_address), sizeof(connection_address)))
    {
        return -1;
    }

    return 0;
}

/**
 * Sends a trigger to initiate analog sampling.
 *
 * @param sample_count The number of samples to take.
 *
 * @return Zero upon success.
 */
int32_t DataStreamServer::send_start_trigger(const uint32_t sample_count)
{
    char trigger_buffer[12] = "trigger";

    /*
     * We can only request a multiple of samples_per_packet.
     */
    if (sample_count % AnalogPacket::samples_per_packet)
    {
        return -1;
    }

    uint32_t samples_net = htonl(sample_count);
    memcpy(&trigger_buffer[8], &samples_net, 4);

    if (send(socket_fd, trigger_buffer, 12, 0) != 12)
    {
        return -1;
    }

    return 0;
}

/**
 * Gets a packet from the connected port.
 *
 * @param buf The location to store the read data.
 * @param len The number of bytes to read from the connection.
 *
 * @return Zero upon success.
 */
int32_t DataStreamServer::get_packet(char *buf, const uint16_t len)
{
    uint32_t bytes_received = 0;
    while (bytes_received < len)
    {
        int32_t ret = recv(socket_fd, &buf[bytes_received], len - bytes_received, 0);

        if (ret < 0)
        {
            return -1;
        }

        bytes_received += ret;
    }

    return 0;
}
