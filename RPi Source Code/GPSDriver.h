#ifndef GPSDRIVER_H
#define GPSDRIVER_H

#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <vector>
#include <sstream>

class GPSDriver {
    public:
    GPSDriver(void);
    std::vector<std::string> splitByDelimiter(const char* buffer, char delimiter);

    bool write_config(int serial_fd, const char * message, int length);

    int configGPS(const char *serial_port);

    int initGPS(const char *serial_port);

    float getGPS(int serial_fd);
};

#endif // GPSDRIVER_H
