#ifndef SERIAL_HPP_
#define SERIAL_HPP_

#include <vector>
#include <string>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <sys/time.h>
#include <sys/ioctl.h>

namespace ombridge_driver
{
    class SerialHandler
    {
        public:
        SerialHandler();

        bool OpenPort(std::string port_path);
        void ClosePort();
        bool WritePort(std::string tx);

        private:
        int fd_;
        std::string port_path_;
    };
}

#endif