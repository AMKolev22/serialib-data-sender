#include "../lib/seriallib.h"
#include <iostream>
#include <string>
#include <cstring>
#include <type_traits>

#if defined(_WIN32) || defined(_WIN64)
#include <windows.h>
#define ROBOT_SLEEP(milliseconds) Sleep(milliseconds)
#define ROBOT_PORT "\\\\.\\COM4"
#endif

#if defined(__linux__) || defined(__APPLE__)
#include <unistd.h>
#define ROBOT_SLEEP(seconds) sleep(seconds)
#define ROBOT_PORT "/dev/ttyACM0"
#endif

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

int main() {
    auto* dev = new Utils::Main<serialib>();
    dev->setup(115200);
    while (true) {
        dev->sendMessage();
        dev->readMessage();
        Utils::modifyMovement(!dev->shouldMove, dev);
        ROBOT_SLEEP(100);
    }
    delete dev;
}
