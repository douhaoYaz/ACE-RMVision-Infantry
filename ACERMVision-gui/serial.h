#ifndef SERIAL_H_INCLUDE
#define SERIAL_H_INCLUDE

#include <iostream>
#include <set>
#include "settings.h"

/**
 *@param 串口类
 */
class SerialPort {
public:
	int fd;					//串口描述配置文件
	std::string path_dev;   //串口路径
	bool is_open;			//是否已经打开

	/**
	 *brief 构造并打开串口
	 *@param path_device 串口路径
	 *@param parameter   调控参数
	 */
    SerialPort(const char* path_device):
            path_dev(path_device) {
        is_open = openPort(path_device);
    }
    SerialPort(std::string path_device):
            SerialPort(path_device.c_str()) { }

	/**
	 *@brief 打开串口，产生fd
     *@param  path_dev 串口路径
     *@return 打开成功:失败
	 */
	bool openPort(const char* path_dev);
    bool openPort(std::string path_dev) {
        return openPort(path_dev.c_str());
    }

    /**
	 *@brief 配置串口
	 */
    int configurePort();

	/**
	 *@brief 重启串口
	 */
    void restartPort();

	/**
	 *@brief 发送角度数据
	 *@param x, y, z 要发送的角度数据
	 *@return 发送成功:失败
	 */
    bool sendXYZ(double x, double y, double z);

	/**
	 *@brief 发送任意数据（最高6位）
	 *@param data 要发送的数组
	 *@param size 数组大小
	 *@return     发送成功:失败
	 */
    bool sendData(char* data, int size);

	/**
	 *@brief 获取数据
	 */
    bool receiveData(Parameter& param);
    
    

	
};

#endif //SERIAL_H_INCLUDE
