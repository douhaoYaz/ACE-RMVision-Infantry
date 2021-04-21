#ifndef THREAD_TASK_H_INCLUDE
#define THREAD_TASK_H_INCLUDE

#include "capture_video.h"
#include "armor_detect.h"
#include "solve_angle.h"
#include "serial.h"
#include "settings.h"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <unistd.h>
#include <chrono>

#ifdef GUI
#include "GUI/main_widget.h"
#include "GUI/start_widget.h"
#include "GUI/component.h"
#include <QApplication>
#include <condition_variable>
#endif

/**
 *@brief 兵种类型
 *@CAR_+:
 *  @INFANTRY 0，步兵
 *  @GUARD    1，哨兵
 */
enum CarType {
    CAR_INFANTRY = 0,
    CAR_GUARD    = 1,
    CAR_HERO     = 2
};


class ThreadTask
#ifdef GUI
        : public QApplication
#endif
{

#ifdef GUI
    Q_OBJECT

public:
    MainWindow* win;
    StartWidget* widget_start;
    MainWidget* widget_main;
    std::mutex mtx_init;
    std::condition_variable cond_var;
    bool is_ready;

public slots:
    void onStart(int mode, QVariant param1, QVariant param2, int type);
#endif

public:
	int type;               //车辆类型
	SerialPort port;
    Setting setting;
    CaptureData data_cap;
    int code_status;

public:
    ThreadTask(int& argc, char** argv);
    ~ThreadTask();
    void imageProduce();
    void imageProcess();
    void control();
    void getSTM32Order();

    void show();

    int exec();

private:
    const char* getTypePath(int type);
};


#endif //THREAD_TASK_H_INCLUDE
