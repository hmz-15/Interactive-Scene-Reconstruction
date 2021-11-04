#include "pg_map_ros/tcp_client.h"

using namespace std;

namespace pgm{


TcpClient::TcpClient(const string &host, const int port)
{
    host_ = host;
    port_ = port;
}


/**
 * Send string data
 * 
 * @param data a string of data to be sent
 * @return error code, return 0 if succeed.
 */
int TcpClient::sendData(const string &data)
{
    int sock = 0;
    struct sockaddr_in serv_addr;

    if((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) 
    { 
        printf("[TcpClient::send]: Socket creation error\n"); 
        return (int)ErrorType::SocketCreationFail; 
    }

    serv_addr.sin_family = AF_INET; 
    serv_addr.sin_port = htons(port_);

    if(inet_pton(AF_INET, host_.c_str(), &serv_addr.sin_addr)<=0) 
    { 
        printf("[TcpClient::send]: Invalid address/ Address not supported\n"); 
        return (int)ErrorType::SocketInvalidIP; 
    } 
    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) 
    { 
        printf("[TcpClient::send]: Connection Failed \n"); 
        return (int)ErrorType::SocketCreationFail; 
    }

    int32_t dsize = htonl(data.length());
    char *dsize_bin = (char *)&dsize;

    // send size of the package first (32 bytes)
    if(sendBin_(sock, dsize_bin, sizeof(dsize)) == -1)
    {
        printf("[TcpClient::send]: Fail to send package\n"); 
        return (int)ErrorType::SocketSendFail;
    }

    // send the package
    if(sendBin_(sock, data.c_str(), data.length()) == -1)
    {
        printf("[TcpClient::send]: Fail to send package\n"); 
        return (int)ErrorType::SocketSendFail;
    }
    
    close(sock);

    return 0;
}


/**
 * Send binary package
 * 
 * @param sock an established socket
 * @param pkg a pointer of the stringified data
 * @param dsize size of the package
 * @return number of bytes sent, return -1 if error occured
 */
int TcpClient::sendBin_(const int sock, const char *pkg, int dsize)
{
    int byte_sent = 0;

    while(byte_sent < dsize)
    {
        int tmp = send(sock, pkg + byte_sent, dsize, 0);
        
        if(tmp == -1)
            return -1;
        
        byte_sent += tmp;
        dsize -= tmp;
    }

    return byte_sent;
}

} // end of namespace pgm