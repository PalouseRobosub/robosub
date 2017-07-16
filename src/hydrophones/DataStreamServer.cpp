#include "DataStreamServer.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <string>
#include <cstring>

using std::string;

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
result_t DataStreamServer::init(const uint16_t dest_port)
{
    /*
     * Build the socket.
     */
    socket_fd = socket(AF_INET, SOCK_STREAM, 0);
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

    ///*
    // * Construct the socket internet address to accept from any address.
    // */
    //struct sockaddr_in serveraddr = {0};
    //serveraddr.sin_family = AF_INET;
    //serveraddr.sin_addr.s_addr = htonl(0xACA80019);
    //serveraddr.sin_port = htons(port_number);

    ///*
    // * Bind the socket to the port.
    // */
    //if (bind(socket_fd, reinterpret_cast<struct sockaddr *>(&serveraddr),
    //            sizeof(serveraddr)) < 0)
    //{
    //    return fail;
    //}

    /*
     * Connect to the external device.
     */
    struct sockaddr_in connection_address = {0};
    connection_address.sin_family = AF_INET;
    connection_address.sin_port = htons(dest_port);
    connection_address.sin_addr.s_addr = inet_addr("172.168.0.253");

    if (connect(socket_fd, reinterpret_cast<struct sockaddr *>(&connection_address), sizeof(connection_address)))
    {
        return fail;
    }

    return success;
}

int32_t DataStreamServer::send_start_trigger()
{
    const string start_string = "trigger";

    if (send(socket_fd, start_string.c_str(), start_string.length(), 0) !=
            static_cast<ssize_t>(strlen(start_string.c_str())))
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
