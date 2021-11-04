#ifndef _PGM_TCP_CLIENT_H
#define _PGM_TCP_CLIENT_H

#include <stdio.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <unistd.h> 
#include <string.h>

#include <string>

#include "pgm_error.h"


namespace pgm{

class TcpClient
{
public:
    TcpClient(const std::string &, const int);

public:
    /**
     * Send string data
     * 
     * @param data a string of data to be sent
     * @return error code, return 0 if succeed.
     */
    int sendData(const std::string &);

private:
    /**
     * Send binary package
     * 
     * @param sock an established socket
     * @param pkg a pointer of the stringified data
     * @param dsize size of the package
     * @return number of bytes sent, return -1 if error occured
     */
    int sendBin_(const int, const char *, int);

private:
    std::string host_;
    int port_;
};

} // end of namespace pgm

#endif