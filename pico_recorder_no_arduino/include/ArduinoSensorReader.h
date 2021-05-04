// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <iostream>
#include <sstream>
#include <bitset>
#include <mutex>

#ifndef ARDUINOSENSORREADER_H
#define ARDUINOSENSORREADER_H

struct Sensordata
{
    float rpy[3] = {0};
    float sensor_readings[3] = {0};
};


class ArduinoSensorReader
{
    public:
        ArduinoSensorReader(std::string portname);
        virtual ~ArduinoSensorReader();

        Sensordata GetSensorData();
        bool HasData();
        void update();

    protected:

    private:
    int serial_port;
    Sensordata robotData;
    bool newdata;
    std::mutex mu;
    unsigned char read_buf [1];
    unsigned char message_buffer[256];
    float temp_buffer[6];
    int buffer_index;
    bool done_reading;
};

#endif // ARDUINOSENSORREADER_H
