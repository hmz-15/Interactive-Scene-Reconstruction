#include "pg_map_ros/pgm_error.h"

using namespace std;

namespace pgm
{

ostream & operator<<(ostream &os, const ErrorType err)
{
    switch (err)
    {
    case ErrorType::BadOperation:
        os << "BadOperation";
        break;

    case ErrorType::NonExistNode:
        os << "NonExistNode";
        break;

    case ErrorType::NullError:
        os << "NullError";
        break;

    case ErrorType::DuplicatedNodeID:
        os << "DuplicatedNodeID";
        break;

    case ErrorType::NoError:
        os << "NoError";
        break;

    case ErrorType::SocketConnectionFail:
        os << "SocketConnectionFail";
        break;

    case ErrorType::SocketInvalidIP:
        os << "SocketInvalidIP";
        break;

    case ErrorType::SocketSendFail:
        os << "SocketSendFail";
        break;

    case ErrorType::SocketCreationFail:
        os << "SocketCreationFail";
        break;

    case ErrorType::FailOpenFile:
        os << "FaileOpenFile";
        break;
    
    default:
        break;
    }

    return os;
}


/**
 * Look up the information of the given error code
 * 
 * @param err_code the error code
 * @return a string of the information that explains the error code
 */
string errorInfo(int err_code)
{
    ErrorType err_t = (ErrorType)err_code;
    return errorInfo(err_t);
}


/**
 * Look up the information of the given error code
 * 
 * @param err the error type
 * @return a string of the information that explains the error code
 */
string errorInfo(const ErrorType err)
{
    string error_info;

    switch (err)
    {
    case ErrorType::NoError:
        error_info = "[NoError]: no error triggered";
        break;

    case ErrorType::BadOperation:
        error_info = "[BadOperation]: not supported operation";
        break;

    case ErrorType::NullError:
        error_info = "[NullError]: receive null pointer or null container";
        break;
    
    case ErrorType::NonExistNode:
        error_info = "[NonExistNode]: cannot find node in parse graph";
        break;

    case ErrorType::DuplicatedNodeID:
        error_info = "[DuplicatedNodeID]: duplicated node ID in parse graph";
        break;

    case ErrorType::SocketCreationFail:
        error_info = "[SocketCreationFail]: socket creation error";
        break;

    case ErrorType::SocketInvalidIP:
        error_info = "[SocketInvalidIP]: Invalid address, or address not supported";
        break;

    case ErrorType::SocketSendFail:
        error_info = "[SocketSendFail]: fail to send data via socket";
        break;

    case ErrorType::SocketConnectionFail:
        error_info = "[SocketConnectionFail]: socket connection failed";
        break;

    case ErrorType::FailOpenFile:
        error_info = "[FailOpenFile]: Fail to open the file";
        break;

    default:
        error_info = "No such error code defined";      
        break;
    }

    return error_info;
}


/**
 * Prompt the error information and terminate the program
 * 
 * @param err the error type
 */
void error(ErrorType err)
{
    cerr << errorInfo(err) << endl;
    exit( (int)err );
}


/**
 * Prompt the error information and terminate the program
 * 
 * @param err the error type
 * @param msg a message string to be prompted
 */
void error(ErrorType err, const string &msg)
{
    cerr << errorInfo(err) << endl;
    cerr << msg << endl;
    
    exit( (int)err );
}

} // end of namespace pgm
