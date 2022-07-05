#pragma once

#include <string>

extern "C" {
    #include <termios.h>
}

namespace serial
{
    
struct Serial
{
    explicit Serial(std::string device, uint64_t baud_rate);
    ~Serial() noexcept;

    void configTTY();
    void loop();

private:
    std::string device_;
    int device_port_;

    struct termios tty;
};

} // namespace serial
