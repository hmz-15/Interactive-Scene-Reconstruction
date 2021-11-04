#ifndef _PGM_ERROR_H
#define _PGM_ERROR_H

#include <iostream>
#include <string>

namespace pgm{
    
enum class ErrorType
{
    // save 0 for no error
    NoError = 0,
    BadOperation,
    NullError,
    NonExistNode,
    DuplicatedNodeID,
    SocketCreationFail,
    SocketInvalidIP,
    SocketSendFail,
    SocketConnectionFail,
    FailOpenFile
};

std::ostream &operator<<(std::ostream&, const ErrorType);


/**
 * Look up the information of the given error code
 * 
 * @param err_code the error code
 * @return a string of the information that explains the error code
 */
std::string errorInfo(int);


/**
 * Look up the information of the given error code
 * 
 * @param err the error type
 * @return a string of the information that explains the error code
 */
std::string errorInfo(const ErrorType);


/**
 * Prompt the error information and terminate the program
 * 
 * @param err the error type
 */
void error(ErrorType);


/**
 * Prompt the error information and terminate the program
 * 
 * @param err the error type
 * @param msg a message string to be prompted
 */
void error(ErrorType, const std::string &);

} // end of namespace pgm

#endif