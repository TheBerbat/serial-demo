#include <serial/Serial.hpp>

#include <signal.h>

volatile sig_atomic_t stop;

void inthand(int signum) {
    stop = 1;
}


int main(int argc, char **argv)
{
    signal(SIGINT, inthand);
    serial::Serial a{"/dev/ttyACM0", 115200U};

    while (!stop)
    {
        a.loop();
    }
    return 0;
}