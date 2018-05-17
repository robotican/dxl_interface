//
// Created by sub on 17/05/18.
//

#ifndef DXL_INTERFACE_EXCEPTIONS_H
#define DXL_INTERFACE_EXCEPTIONS_H

#include <stdexcept>

namespace dxl
{
    namespace exceptions
    {

        class CommunicationException : public std::runtime_error
        {
        public:
            CommunicationException(std::string err_msg) : std::runtime_error(err_msg) {}
        };

        class ReadException : public CommunicationException
        {
        public:
            ReadException(std::string err_msg) : CommunicationException(err_msg) {}
        };

        class WriteException : public CommunicationException
        {
        public:
            WriteException(std::string err_msg) : CommunicationException(err_msg) {}
        };

        class ReadVelException : public ReadException
        {
        public:
            ReadVelException(std::string err_msg) : ReadException(err_msg) {}
        };

        class ReadPosException : public ReadException
        {
        public:
            ReadPosException(std::string err_msg) : ReadException(err_msg) {}
        };

        class ReadLoadException : public ReadException
        {
        public:
            ReadLoadException(std::string err_msg) : ReadException(err_msg) {}
        };

        class ReadErrorException : public ReadException
        {
        public:
            ReadErrorException(std::string err_msg) : ReadException(err_msg) {}
        };


        class WriteVelException : public WriteException
        {
        public:
            WriteVelException(std::string err_msg) : WriteException(err_msg) {}
        };

        class WritePosException : public WriteException
        {
        public:
            WritePosException(std::string err_msg) : WriteException(err_msg) {}
        };
    }

}

#endif //DXL_INTERFACE_EXCEPTIONS_H
