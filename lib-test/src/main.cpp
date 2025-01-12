#include "../lib/seriallib.h"
#include <unistd.h>
#include <iostream>
#include <string>
#include <cstring>
#include <type_traits>

#if defined(_WIN32) || defined(_WIN64)
    #include <windows.h>
    #define ROBOT_SLEEP(milliseconds) Sleep(milliseconds)
    #define ROBOT_PORT "\\\\.\\COM3"
#endif

#if defined(__linux__) || defined(__APPLE__)
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

                if constexpr (std::is_same_v<TYPE_OF_MESSAGE, std::string>) {
                    std::cout << arg << std::endl;
                    dev->writeString(arg.c_str());
                }
                else
                    throw std::invalid_argument("Only strings are supported");
                }
            catch (const std::exception& err) {
                    std::cerr << "Error: " << err.what() << std::endl;
                }
        }

        void readMessage() {
            char buffer[10] = {};
            this->dev->readString(buffer, '\n', sizeof(buffer) - 1, 2000);
            std::cout << buffer << std::endl;
        }

    private:
        SerialType* dev{};
        bool ready;
    };
}

int main() {
    auto* dev = new Utils::Main<serialib>();
    dev->setup(115200);
    while (true) {
        dev->sendMessage<std::string>("move");
        dev->readMessage();
        ROBOT_SLEEP(50);
    }
    delete dev;
}
