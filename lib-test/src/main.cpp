#define ASIO_STANDALONE
#define _WEBSOCKETPP_CPP11_TYPE_TRAITS_
#define _WEBSOCKETPP_CPP11_CHRONO_
#define _WEBSOCKETPP_NOEXCEPT_

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>   
#include "server.h"
#include "../lib/seriallib.h"


#include <iostream>
#include <string>
#include <chrono>
#include <memory>

#if defined(_WIN32) || defined(_WIN64)
#include <windows.h>
#define ROBOT_SLEEP(milliseconds) Sleep(milliseconds)
#define ROBOT_PORT "\\\\.\\COM5"
#endif

#if defined(__linux__) || defined(__APPLE__)
#include <unistd.h>
#define ROBOT_SLEEP(seconds) sleep(seconds)
#define ROBOT_PORT "/dev/ttyACM0"
#endif

//websocketpp::connection_hdl hdlReference;
//websocketpp::server<websocketpp::config::asio> server;

namespace Utils {

    template <typename SerialType>
    class Main {
    public:
        Main() : dev(new SerialType()) {
            this->ready = false;
        };

        ~Main() {
            if (this->dev != nullptr) {
                this->dev->closeDevice();
                delete this->dev;
            }
        }
        void setup(int bandw) {
            try {
                if (this->ready)
                    throw std::logic_error("Program already ready");
                else {
                    if (signed char error = this->dev->openDevice(ROBOT_PORT, bandw) != 1)
                        throw std::runtime_error("Failed to open serial port");
                    else {
                        this->ready = true;
                        std::cout << "Connection successful to port " << ROBOT_PORT << std::endl;
                    }
                }
            }
            catch (const std::exception& err) {
                std::cerr << err.what() << std::endl;
            }
        }

        template <typename TYPE_OF_MESSAGE>
        void sendMessage(TYPE_OF_MESSAGE arg) {
            try {
                if (!this->ready)
                    throw std::logic_error("Device is not ready");

                this->dev->writeString(std::is_same_v<TYPE_OF_MESSAGE, std::string> ? arg.c_str() : std::to_string(arg).c_str());
            }
            catch (const std::exception& err) {
                std::cerr << "Error: " << err.what() << std::endl;
            }
        }

        void sendMessage() {
            try {
                if (!this->ready)
                    throw std::logic_error("Device is not ready");

                if (this->shouldMove)
                    this->dev->writeString("move");

                else
                    this->dev->writeString("stop");

            }
            catch (const std::exception& err) {
                std::cerr << "Error: " << err.what() << std::endl;
            }
        }

        void readMessage() {
            char buffer[10] = {};
            this->dev->readString(buffer, '\n', sizeof(buffer) - 1, 2000);
            std::cout << buffer;
        }

    private:
        SerialType* dev{};
        bool ready;
    public:
        bool shouldMove = false;
    };

    template <typename DeviceType>
    void modifyMovement(bool status, Utils::Main<DeviceType>* coreDevice) {
        coreDevice->shouldMove = status;
    }
}

auto dev = std::make_shared<Utils::Main<serialib>>();

void on_message(websocketpp::server<websocketpp::config::asio>* s,
    websocketpp::connection_hdl hdl,
    websocketpp::server<websocketpp::config::asio>::message_ptr msg) {
    std::string payload = msg->get_payload();

    std::cout << "Received message: " << payload << std::endl;

    if (payload == "Detected human") {
        Utils::modifyMovement(false, dev.get());

        dev->sendMessage();
        dev->readMessage();

        try {
            s->send(hdl, "Detected human!", websocketpp::frame::opcode::text);
        }
        catch (const websocketpp::exception& e) {
            std::cout << "Error sending message: " << e.what() << std::endl;
        }
    }
}

int main() {
    dev->setup(115200);
    try {
        websocketpp::server<websocketpp::config::asio> server;
        std::cout << "Opnening 1" << std::endl;

        server.set_access_channels(websocketpp::log::alevel::all ^ websocketpp::log::alevel::frame_header);

        std::cout << "Opnening 2" << std::endl;
        server.clear_access_channels(websocketpp::log::alevel::frame_header);
        std::cout << "Opnening 3" << std::endl;
        server.init_asio();
        std::cout << "Opnening 4" << std::endl;
        server.set_message_handler(bind(&on_message, &server, ::_1, ::_2));
        std::cout << "Opnening 5" << std::endl;

        server.listen(SERVER_PORT);
        std::cout << "Opnening 6" << std::endl;
        server.start_accept();
        std::cout << "Opnening 7" << std::endl;
        server.run();
        std::cout << "Server listening on port " << SERVER_PORT;
    }
    catch (websocketpp::exception const& e) {
        std::cout << e.what() << std::endl;
    }
    catch (...) {
        std::cout << "other exception" << std::endl;
    }


}