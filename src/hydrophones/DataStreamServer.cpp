#include "DataStreamServer.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>

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
 * @return Success or fail.
 */
result_t DataStreamServer::init(const uint16_t port_number)
{
    /*
     * Build the socket.
     */
    socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd < 0)
    {
        return fail;
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
        return fail;
    }

    /*
     * Construct the socket internet address to accept from any address.
     */
    struct sockaddr_in serveraddr = {0};
    serveraddr.sin_family = AF_INET;
    serveraddr.sin_addr.s_addr = INADDR_ANY;
    serveraddr.sin_port = htons(port_number);

    /*
     * Bind the socket to the port.
     */
    if (bind(socket_fd, reinterpret_cast<struct sockaddr *>(&serveraddr),
                sizeof(serveraddr)) < 0)
    {
        return fail;
    }

    return success;
}

/**
 * Gets an AnalogPacket from the UDP port.
 *
 * @param buf The location to store the AnalogPacket.
 * @param max_len The maximum number of data bytes to store in buf.
 *
 * @RETURN The number of bytes read from the UDP port.
 */
int32_t DataStreamServer::get_packet(char *buf, const uint16_t max_len)
{
    return recvfrom(socket_fd, buf, max_len, 0, nullptr, nullptr);
}
