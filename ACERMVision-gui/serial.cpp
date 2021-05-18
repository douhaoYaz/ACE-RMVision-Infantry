#include "serial.h"
#include "utils.h"

#if OS == 0
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#endif

#include <cmath>
#include <cstdio>
#include <iomanip>
#include <cstring>
#include <cerrno>
#include <string>
#include <iterator>

bool SerialPort::openPort(const char* path_dev) {
    fd = open(path_dev, O_RDWR | O_NOCTTY | O_NDELAY);      //根据路径打开文件，并根据参数决定打开方式，若成功则返回文件描述符，若失败则返回-1
    if (fd != -1) {
        fcntl(fd, F_SETFL, 0);                              //设置文件描述符状态标志，0对应的是O_APPEND（不确定），强制每次写(write)操作都添加在文件大的末尾

#ifdef TEXT_PORT_OPEN
        StatusLog::log("已打开串口!");
#endif //TEXT_PORT_OPEN
        return true;
    }
    else {
#ifdef TEXT_PORT_OPEN
        StatusLog::log(std::string("打开串口错误，无法打开") + path_dev);
#endif //TEXT_PORT_OPEN

        return false;
    }
}


int SerialPort::configurePort() {
    struct termios port_settings;               // structure to store the port settings in

    cfsetispeed(&port_settings, B115200);       // set baud rates
    cfsetospeed(&port_settings, B115200);

    port_settings.c_cflag = (port_settings.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    port_settings.c_iflag &= ~IGNBRK;         // disable break processing
    port_settings.c_lflag = 0;                // no signaling chars, no echo,
    // no canonical processing
    port_settings.c_oflag = 0;                // no remapping, no delays
    port_settings.c_cc[VMIN]  = 0;            // read doesn't block
    port_settings.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    port_settings.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    port_settings.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
    // enable reading
    port_settings.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    port_settings.c_cflag |= 0;
    port_settings.c_cflag &= ~CSTOPB;
    port_settings.c_cflag &= ~CRTSCTS;
    port_settings.c_iflag = ICANON;
    port_settings.c_cc[VMIN] = 10;           // read doesn't block
    port_settings.c_cc[VTIME] = 5;          // 0.5 seconds read timeout

    tcsetattr(fd, TCSANOW, &port_settings);             // apply the settings to the port

    return fd;
}

void SerialPort::restartPort() {
    close(fd);
    openPort(path_dev);
    configurePort();
}

bool SerialPort::sendXYZ(double x, double y, double z) {



    Parameter& param = Parameter::getParameter();
    uchar bytes_send[] = { 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE };
//    if (x > 0 || y > 0 || z > 0) {
    if (!(x||y||z)) {
        if (8 == write(fd, bytes_send, 8))
            return true;
        return false;
    }

    x *= param.other.factor_x;
    y *= param.other.factor_y;
    z *= param.other.factor_z;

    short* data = (short*)(bytes_send+1);
    data[0] = x;
    data[1] = y;
    data[2] = z;

//    std::copy(data+1, data+6, std::ostream_iterator<std::ostream>(std::cout, " "));
//    std::cout << std::endl;

    if (8 == write(fd, bytes_send, 8))
        return true;
    return false;
}

//TODO
bool SerialPort::receiveData(Parameter& param) {
    tcflush(fd, TCIFLUSH);   // Discards old data in the rx buffer
    unsigned char read_buffer[5];
    unsigned char head = 0xFF;
    unsigned char end = 0xFE;
    int  read_status = 0;

    read_status = read(fd, &read_buffer, 5);

    if(read_status == -1 || read_status == 0) {
        //重启串口
        restartPort();
        return false;
    }
    if(read_buffer[0] == head && read_buffer[4] == end) {
        param.other.color_enemy = bool(read_buffer[1]);
        param.other.mode_detect = bool(read_buffer[2]);
        param.other.type_buff = bool(read_buffer[3]);
        std::cout << "param.other.color_enemy:" << param.other.color_enemy << std::endl;
        std::cout << "param.other.mode_detect:" << param.other.mode_detect << std::endl;
        std::cout << "param.other.type_buff:" << param.other.type_buff << std::endl;

        return true;
    }
    return false;
}


bool SerialPort::sendData(char* data, int size) {

    unsigned char bytes_send[] = { 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE };
    if (size > 6 || size < 0) {
        if (8 == write(fd, bytes_send, 8))
            return true;
        return false;
    }

    bytes_send[1] = (unsigned char)size;
    for (int i = 0; i < size; i++)
        bytes_send[1+i] = data[i];

#ifdef TEXT_PORT_SEND
    std::cout << "port_send_data" << std::endl;
    for (int i = 0; i < 8; i++)
        printf(" %x", bytes_send[i]);
    std::cout << std::endl;
#endif  //TEXT_PORT_SEND

    if (8 != write(fd, bytes_send, 8))
        return false;
    return true;
}
