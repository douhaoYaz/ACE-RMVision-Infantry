#ifndef CAPTURE_VIDEO_H_INCLUDE
#define CAPTURE_VIDEO_H_INCLUDE

#include "settings.h"
#include <galaxy/GxIAPI.h>
#include <galaxy/DxImageProc.h>
#include <iostream>
#include <opencv2/opencv.hpp>

/**
 *@brief相机驱动类型
 *@CAPTURE_ +:
 *	@DRIVER_V4L2   0，未知状态
 *	@DRIVER_CV	   1，未激活
 *	@DRIVER_GALAXY 2，已激活
 */
enum CaptureDriverType {
    CAPTURE_DRIVER_V4L2   = 0,
    CAPTURE_DRIVER_CV     = 1,
    CAPTURE_DRIVER_GALAXY = 2
};

/**
 *@brief相机焦距类型
 *@CAPTURE_ +:
 *	@SHORT 0，未知状态
 *	@LONG  1，未激活
 */
enum CaptureType {
    CAPTURE_SHORT = 0,
    CAPTURE_LONG  = 1
};

// ----------- galaxy ---------------
class CameraDevice {
public:
    CameraDevice() { }
    ~CameraDevice();
    int init();
    void open();
	CameraDevice& operator>>(cv::Mat& img);
    uint64_t getFrameNumber();
private:
    GX_STATUS status;
    GX_DEV_HANDLE hDevice;
	GX_OPEN_PARAM stOpenParam;
    uint32_t nDeviceNum;
    GX_FRAME_DATA stFrameData;
    cv::Mat src;
    uint64_t nFrameNum;
};


// --------------------- v4l2 -------------------------------
class CaptureVideo {
public:
	CaptureVideo() { }
    CaptureVideo(const char* device, unsigned int in_size_buffer=1);
    void open(const char* device, unsigned int in_size_buffer=1);
    void open(const std::string& device, unsigned int in_size_buffer=1) {
        open(device.c_str(), in_size_buffer);
    }

    ~CaptureVideo();
    bool startStream();
    bool closeStream();
    char* videoName(const char* format=".avi");

    // setting
    bool setExposureTime(bool in_exp, int in_t);
    bool setVideoFormat(int in_width, int in_height, bool in_mjpg=1);
    bool setVideoFPS(int in_fps);
    bool setBufferSize(int in_buffer_size);
    bool setGamma(int in_gamma);

    // restarting
    bool changeVideoFormat(int in_width, int in_height, bool in_mjpg=1);
    void restartCapture();

    // getting
    bool getVideoSize(int& width, int& height);
    uint getFrameCount() {
        return current_frame;
    }
    void info();

    void check();

    CaptureVideo& operator>>(cv::Mat& image);

private:

	typedef struct {
		void* ptr;
        uint size;
	}MapBuffer;
    uint capture_width;
    uint capture_height;
    uint format;
	int fd;
    uint buffer_size;
    uint buffer_index;
    uint current_frame;
	MapBuffer* mb;
	const char* video_path;

    void cvtRaw2Mat(const void* data, cv::Mat& image);
    bool refreshVideoFormat();
    bool initMMap();
    int xioctl(int in_fd,int in_request, void *arg);

};

class Capture;
/**
 *@brief 相机数据
 *@用循环队列做缓存，FIFO，因此有BUFFER_SIZE帧的延迟
 */
class CaptureData {
public:
	
	typedef struct {
		cv::Mat img;	//图像
		int type;		//长焦or短焦
        int length_f;	//焦距
	}ImageData;

	/**
	 *@brief 创建相机数据
	 *@param size_buffer 缓存的大小
	 */
    CaptureData(int size_buffer): sz_buffer(size_buffer),
            idx_get(0), idx_read(0) {
        q_data = new ImageData[sz_buffer];
        sz_buffer = size_buffer;
	}

	~CaptureData() {
        if (q_data != nullptr)
            delete[] q_data;
	}

	/**
	 *@brief 获取相机传入的图像
	 *@return idx_get-1帧图像
	 */
	cv::Mat& currentGetImage() {
		return q_data[(idx_get - 1) % sz_buffer].img;
	}

	/**
	 *@brief 获取相机读取的图像
	 *@return idx_read-1帧图像
	 */
	cv::Mat& currentReadImage() {
		return q_data[(idx_read - 1) % sz_buffer].img;
	}

	/**
	 *@brief 传出图像
	 *@输出:
	 *	@img    要传出的cv::Mat
	 *	@return *this
	 */
	CaptureData& operator>>(cv::Mat& img) {
		q_data[idx_read % sz_buffer].img.copyTo(img);	//不要在此处自加，保证读完
        idx_read++;
		return *this;
	}

	/**
	 *@brief 判断队满
	 *@return 队满:队非满
	 */
	bool isFull() {
       return (idx_get - idx_read) >= sz_buffer;
	}

	/**
	 *@brief 判断队空
	 *@return 队空:队非空
	 */
	bool isEmpty() {
        return idx_get <= idx_read;
	}

	/**
	 *@brief 将get和read索引重置为0，以同步读写
	 */
	void reset() {
		idx_get = idx_read = 0;
	}


    void getFromCapture(Capture& cap);

private:
    ImageData* q_data;		//缓存队列
    int sz_buffer;			//缓存大小
    volatile int idx_get;	//读取索引
    volatile int idx_read;	//获取索引
};

//优先级队列处理图像，改进循环队列

class Capture {
public:
    std::string path_device;
    int length_f;
    cv::Size sz;
    int type_driver;	//驱动类型
    int type_f;			//是否为长焦相机
    union _Capture {
        cv::VideoCapture* cap_cv;
        CaptureVideo* cap_v4l2;
        CameraDevice* cap_galaxy;
        _Capture(): cap_cv(nullptr) { }
        ~_Capture() { }
    }cap;

    Capture(cv::Size size, int length_focus, int type_focus=CAPTURE_SHORT,
            int type_device=CAPTURE_DRIVER_CV);

    Capture(const std::string& path_video, cv::Size size);

    ~Capture() {
        if (type_driver == CAPTURE_DRIVER_V4L2)
            delete cap.cap_v4l2;
        else if (type_driver == CAPTURE_DRIVER_CV)
            delete cap.cap_cv;
        else if (type_driver == CAPTURE_DRIVER_GALAXY)
            delete cap.cap_galaxy;
    }

    Capture& operator>>(cv::Mat& img);

    Capture& operator>>(CaptureData& data);

private:
    static int id;
};

class CameraChooser {
public:
    float th_short;
    float th_long;
    float cnt_lost;
    float th_lost;
    float r;

    CameraChooser(Setting& setting): th_short(setting.th_short),
        th_long(setting.th_long), r(setting.rate_switch_cap), dist_last(0) { }



	/**
		*@brief 切换相机策略
		*@param dist_short 短距离阈值(mm)
		*@param dist_long  长距离阈值(mm)
		*@param mode_last  上一次摄像头模式
		*@return 返回本次摄像头类型 0-短焦 1-6mm长焦  2-8mm长焦
	*/
	/**
	 *@breif 根据数据更改data以选择相机输入
	 */
	bool chooseCamera();

    void resetDistance() {
        dist_last = 0;
    }

private:
    int dist_last;
};

#endif //ifndef CAPTURE_VIDEO_H_INCLUDE
