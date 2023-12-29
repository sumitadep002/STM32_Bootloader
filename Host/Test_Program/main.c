#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

int main() {
    // Open the serial port (ttyUSB0)
    int serialPort = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);

    if (serialPort == -1) {
        perror("Error opening serial port");
        return 1;
    }

    // Configure the serial port
    struct termios serialOptions;
    tcgetattr(serialPort, &serialOptions);
    
    // Set baud rate to 115200
    cfsetispeed(&serialOptions, B115200);
    cfsetospeed(&serialOptions, B115200);

    // Set other serial port settings (8N1)
    serialOptions.c_cflag &= ~PARENB;x
    serialOptions.c_cflag &= ~CSTOPB;
    serialOptions.c_cflag &= ~CSIZE;
    serialOptions.c_cflag |= CS8;

    tcsetattr(serialPort, TCSANOW, &serialOptions);

    // Write data to the serial port
    char message[] = "Hello, ttyUSB0!\n";
    write(serialPort, message, strlen(message));

    // Read data from the serial port
    char buffer[256];
    ssize_t bytesRead = read(serialPort, buffer, sizeof(buffer));

    if (bytesRead > 0) {
        buffer[bytesRead] = '\0';
        printf("Received data: %s", buffer);
    } else {
        perror("Error reading from serial port");
    }

    // Close the serial port
    close(serialPort);

    return 0;
}

