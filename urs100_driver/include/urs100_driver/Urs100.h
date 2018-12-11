#ifndef URS100_DRIVER_H
#define URS100_DRIVER_H

#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <sstream>
#include "std_msgs/String.h"

#include "serial/serial.h"
#include "ros/ros.h"
#include "ros/console.h"

namespace urs100_driver {

    class Urs100 {
    public:
        Urs100(std::string port, int baud, bool is_blocking);

        ~Urs100();

    public:
        double getPosition();

        std::string readState();

        void setPosition(double pos);

        void goHome();

        void stopMotion();

        static void enumerate_ports();

    private:
        void sendCommand(std::string command, std::string parameter);

    private:
        serial::Serial *m_serial_handle;
        std::string m_starter;
        std::string m_terminator;
        bool m_is_blocking;
        double m_previous;
    };
}

#endif