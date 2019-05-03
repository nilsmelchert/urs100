#include "urs100_driver/Urs100.h"

#define _USE_MATH_DEFINES
#define MAX_REC_DEPTH 15

#include <math.h>

namespace urs100_driver {
    Urs100::Urs100(std::string port, int baud, bool is_blocking) {

        m_is_blocking = is_blocking;
        m_starter = "01";
        m_terminator = "\r\n";
        m_previous = 0.0;
        m_rec_depth = 0;

        serial::Serial *serial_connection = new serial::Serial(port, baud, serial::Timeout::simpleTimeout(1000));;
        m_serial_handle = serial_connection;

        if (m_serial_handle->isOpen()) {
            std::cout << "Serial connection established" << std::endl;
        }

        // Reference stage if it is not referenced
        if (this->readState() == "0A") {
            this->goHome();
        }
    }


    Urs100::~Urs100() {
        std::cout << "Free the serial bus ressources..." << std::endl;
        delete m_serial_handle;
    }

    void Urs100::sendCommand(std::string command, std::string parameter = "") {
        std::string pos_str = m_starter + command + parameter + m_terminator;
        m_serial_handle->write(pos_str);
    }

    void Urs100::setVelocity(double vel) {

        this->sendCommand("VA", std::to_string(vel));
    }

    double Urs100::getVelocity() {
        std::string cmd_str = m_starter + "VA?" + m_terminator;
        m_serial_handle->write(cmd_str);
        std::string result = m_serial_handle->readline();
        double vel = std::stod(result.substr(4, result.find_first_of(m_terminator) - 4));
        return vel;
    }

    void Urs100::setPosition(double pos) {

        double pos_deg = pos / M_PI * 180.0;
        if (!this->m_is_blocking && (m_previous != pos)) {
            this->stopMotion();
        }
        this->sendCommand("PA", std::to_string(pos_deg));

        if (this->m_is_blocking) {
            while (this->readState() != "33") {
            }
        }
        m_previous = pos;
    }

    void Urs100::goHome() {
        this->sendCommand("PA", "");
        std::string cmd_str = m_starter + "OR" + m_terminator;
        m_serial_handle->write(cmd_str);

        std::string current_state = this->readState();
        // Blocking routine
        while (current_state != "32" && current_state != "33") {
            current_state = this->readState();
        }
        // Move to another position and to zero again to get the right state to work with!
        this->setPosition(0.1);
        this->setPosition(0.0);
        usleep(100000);
    }

    void Urs100::stopMotion() {
        this->sendCommand("ST", "");
    };

    double Urs100::getPosition() {

        std::string cmd_str = m_starter + "TP" + m_terminator;
        m_serial_handle->write(cmd_str);
        std::string result = m_serial_handle->readline();

        try {
            double pos = std::stod(result.substr(4, result.find_first_of(m_terminator) - 4)) / 180.0 * M_PI;
            m_rec_depth = 0;
            return pos;
        }
        catch (const std::exception &e) {
            if (m_rec_depth > MAX_REC_DEPTH) {
                std::cout << "ERROR: Max recursion depth of " << MAX_REC_DEPTH <<  " was reached" << std::endl;
                return -1;
            }
            m_rec_depth++;
            return this->getPosition();
        }
    }

    std::string Urs100::readState() {
        std::string cmd_str = m_starter + "TS" + m_terminator;
        m_serial_handle->write(cmd_str);
        std::string result = m_serial_handle->readline();
        return result.substr(8, result.find_first_of(m_terminator) - 8);
    }


    void Urs100::enumerate_ports() {
        std::vector<serial::PortInfo> devices_found = serial::list_ports();
        std::vector<serial::PortInfo>::iterator iter = devices_found.begin();

        while (iter != devices_found.end()) {
            serial::PortInfo device = *iter++;

            printf("(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
                   device.hardware_id.c_str());
        }
    }
}

