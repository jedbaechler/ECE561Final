#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <vector>
#include <sstream>
#include "GPSDriver.h"

GPSDriver::GPSDriver(void){

}

std::vector<std::string> GPSDriver::splitByDelimiter(const char* buffer, char delimiter = ',') {
    std::vector<std::string> tokens;
    std::stringstream ss(buffer);
    std::string token;

    while (std::getline(ss, token, delimiter)) {
        tokens.push_back(token);
    }

    return tokens;
}

bool GPSDriver::write_config(int serial_fd, const char * message, int length){
    if (write(serial_fd, message, length) > 0){
        return 1;
    };
    return 0;

}

int GPSDriver::configGPS(const char *serial_port){
    int baud_rate = B9600; // Baud rate for communication
    int serial_fd;

    // Open serial port
    serial_fd = open(serial_port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd == -1) {
        std::cerr << "Failed to open serial port" << std::endl;
        return 1;
    }

    // Configure serial port
    struct termios options;
    tcgetattr(serial_fd, &options);
    cfsetispeed(&options, baud_rate);
    cfsetospeed(&options, baud_rate);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Disable canonical mode and local echo
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
    options.c_oflag &= ~OPOST; // Disable output processing
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10; // 1 second timeout
    tcsetattr(serial_fd, TCSANOW, &options);

    const char baud_config[28] = {0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0x96,0x00,0x00,0x07,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x93,0x90};
    const char rate_2Hz[14] = {0xB5,0x62,0x06,0x08,0x06,0x00,0xF4,0x01,0x01,0x00,0x01,0x00,0x0B,0x77};
    const char rate_1Hz[14] = {0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39};
    printf("Config Verification %d\n",write_config(serial_fd, rate_2Hz, 14));
    printf("Config Verification %d\n",write_config(serial_fd, baud_config, 28));
    close(serial_fd);
    return 0;
}

int GPSDriver::initGPS(const char *serial_port){
    int serial_fd;
    serial_fd = open(serial_port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd == -1) {
        std::cerr << "Failed to open serial port" << std::endl;
        return 1;
    }
    struct termios options_1;
    int baud_rate = B38400;
    tcgetattr(serial_fd, &options_1);
    cfsetispeed(&options_1, baud_rate);
    cfsetospeed(&options_1, baud_rate);
    options_1.c_cflag |= (CLOCAL | CREAD);
    options_1.c_cflag &= ~PARENB;
    options_1.c_cflag &= ~CSTOPB;
    options_1.c_cflag &= ~CSIZE;
    options_1.c_cflag |= CS8;
    options_1.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Disable canonical mode and local echo
    options_1.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
    options_1.c_oflag &= ~OPOST; // Disable output processing
    options_1.c_cc[VMIN] = 0;
    options_1.c_cc[VTIME] = 10; // 1 second timeout
    tcsetattr(serial_fd, TCSANOW, &options_1);
    return serial_fd;
}

float GPSDriver::getGPS(int serial_fd) {

    try {

        char buffer[256];
        int buffer_index = 0;

        while (true) {
            // Read a character from the serial port
            char uart_char;
            if (read(serial_fd, &uart_char, 1) > 0) {
                // Store the character in the buffer
                buffer[buffer_index++] = uart_char;

                if (uart_char == '\n') {
                    buffer[buffer_index] = '\0'; // Null-terminate the string

                    if (strncmp(buffer, "$GNVTG", 6) == 0) {

//                        std::cout << "Recieved GNVTG Line" << std::endl;
                        std::vector<std::string> message = splitByDelimiter(buffer);
//                        std::cout << message[7] << " km/h" <<std::endl;
//                        std::cout << std::stof(message[7])*0.621371 << " Miles per Hour" << std::endl;
                        return std::stof(message[7])*0.621371;
                    }

                    // Reset buffer index
                    buffer_index = 0;
                }
            }
        }
    } catch (...) {
        // Close serial port on exception
        close(serial_fd);
        std::cerr << "Serial connection closed." << std::endl;
    }

    return 0;
}
