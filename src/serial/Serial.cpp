#include <serial/Serial.hpp>

#include <cstdio>
#include <cstring>
#include <stdexcept>

#include <cerrno>
extern "C" {
    #include <fcntl.h>
    #include <unistd.h>
}


namespace serial
{

Serial::Serial(std::string device, uint64_t baud_rate)
  : device_(device)
  , device_port_( open(device.c_str(), O_RDWR) )
{
    std::printf("INFO: Opening serial %s\n", device_.c_str());
    if (device_port_ < 0) {
        std::printf("ERROR: Id %i from open: %s\n", errno, std::strerror(errno));
        throw std::runtime_error("ERROR: Id from open.");
    }

    configTTY();
}

Serial::~Serial() noexcept
{
    std::printf("INFO: Closing serial %s\n", device_.c_str());
    close(device_port_);
}

void Serial::configTTY()
{
    if (tcgetattr(device_port_, &tty) != 0) {
        std::printf("ERROR: Id %i from tcgetattr: %s\n", errno, std::strerror(errno));
        throw std::runtime_error("ERROR: Id from config device. Can not get attr");
    }

    std::printf("INFO: configuring serial %s\n", device_.c_str());
    tty.c_cflag &= ~PARENB;   // Parity bit set
    tty.c_cflag &= ~CSTOPB;  // One stop bit
    tty.c_cflag &= ~CSIZE;   // 8 bits per byte
      tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS flow control
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 20;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 115200
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    // Save tty settings, also checking for error
    if (tcsetattr(device_port_, TCSANOW, &tty) != 0) {
        std::printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        throw std::runtime_error("ERROR: Id from config device. Can not set attr");
    }
}

void Serial::loop()
{
    char read_buf [256];
    std::memset(&read_buf, '\0', sizeof(read_buf));
    int num_bytes = read(device_port_, &read_buf, sizeof(read_buf));
    if (num_bytes<0)
    {
        std::printf("ERROR: Not received data\n");
        std::fflush(stdout);
        return;
    }
    std::printf("Read %i bytes. Received message: '%s'\n", num_bytes, read_buf);
    std::fflush(stdout);
}

} // namespace serial
