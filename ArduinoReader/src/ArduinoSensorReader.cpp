#include "ArduinoSensorReader.h"

using namespace std;

ArduinoSensorReader::ArduinoSensorReader() : newdata(false)
{
    serial_port = open("/dev/ttyACM3", O_RDWR);
    if (serial_port < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
    }

    struct termios tty;
    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }

    struct termios {
        tcflag_t c_iflag;		/* input mode flags */
        tcflag_t c_oflag;		/* output mode flags */
        tcflag_t c_cflag;		/* control mode flags */
        tcflag_t c_lflag;		/* local mode flags */
        cc_t c_line;			/* line discipline */
        cc_t c_cc[NCCS];		/* control characters */
    };

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
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
    tty.c_cc[VTIME] = 0;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 1;

    cfsetspeed(&tty, B115200);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }

    tcflush(serial_port,TCIOFLUSH);

    // Allocate memory for read buffer, set size according to your needs
    unsigned char read_buf [1];
    unsigned char message_buffer[256];
    float temp_buffer[6];
    int buffer_index = -1;
    bool done_reading = false;

    while (true) {
        int n = read(serial_port, &read_buf, sizeof(read_buf));
        if (n > 0) {
            if (read_buf[0] == 0xff && buffer_index == -1) {
                buffer_index = 0;
            }

            if (buffer_index > -1) {
                message_buffer[buffer_index] = read_buf[0];
                buffer_index++;

                if (buffer_index > 24) {
                    done_reading = true;
                }
            }

            if (done_reading) {
                for (int i = 0; i < 6; i++) {
                    float * my_float = (float*)&message_buffer[1 + i*4];
                    std::cout << *my_float << std::endl;
                    temp_buffer[i] = *my_float;
                }
                robotData.sensor_readings[0] = temp_buffer[0];
                robotData.sensor_readings[1] = temp_buffer[1];
                robotData.sensor_readings[2] = temp_buffer[2];
                robotData.rpy[0] = temp_buffer[3];
                robotData.rpy[1] = temp_buffer[4];
                robotData.rpy[2] = temp_buffer[5];

                done_reading = false;
                buffer_index = -1;
                mu.lock();
                newdata = true;
                mu.unlock();
            }
        }
    }
}

ArduinoSensorReader::~ArduinoSensorReader()
{
    close(serial_port);
}

Sensordata ArduinoSensorReader::GetSensorData()
{
    mu.lock();
    newdata = false;
    mu.unlock();
    return robotData;
}

bool ArduinoSensorReader::HasData()
{
    return newdata;
}