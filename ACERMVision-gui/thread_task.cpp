#include "thread_task.h"
#include "serial.h"
#include "kalman.hpp"
#include "buff_detect.h"
#include "ui_tool.h"
#include <atomic>
#include <iostream>
#include <fstream>
#include <regex>
#include <condition_variable>


#ifdef GUI
#include <QThread>
#endif

#define UNTIL(BOOLEAN) while ((BOOLEAN)==false)

//设置结束线程标志位，负责结束所有线程。
static std::atomic_bool flag_end_thread(false);

ThreadTask::ThreadTask(int& argc, char** argv):
#ifdef GUI
        QApplication(argc, argv), is_ready(false),
#endif  //GUI
        type(0), port("/dev/ttyUSB0"), data_cap(2) {            //初始化车辆类型type，根据路径打开串口文件初始化串口,缓存队列大小设置为2

    port.configurePort();

#ifdef GUI

    QFile qss("../GUI/stylesheet/main.qss");
    qss.open(QFile::ReadOnly);
    this->setStyleSheet(qss.readAll());
    qss.close();

    win = new MainWindow();
    widget_start = new StartWidget(win);
    win->setWidget(widget_start);
    connect(widget_start, SIGNAL(openByMode(int, QVariant, QVariant, int)),
            this, SLOT(onStart(int, QVariant, QVariant, int)));
    win->show();

#else
    std::string path_video;
    std::string path_cap_short  = "../parameter/capture/camera_4mm_1.xml";
    std::string path_cap_galaxy = "../parameter/capture/camera_galaxy_1.xml";
    if (argc >= 2)
        type = argv[1][0]-'0';
    if (argc == 3)
        path_video = std::string(argv[2]);
    path_video = "../video/armor.avi";
    switch (type) {
    default: case CAR_INFANTRY:
        std::cout<<"Now it is INFANTRY mode!"<<std::endl;
        break;
    case CAR_GUARD:
        std::cout<<"Now it is GUARD mode!"<<std::endl;
        break;
    }
    setting.setInfantryParameter(path_cap_short);
//    setting.setInfantryParameter(path_cap_short,path_cap_galaxy);
//      setting.setInfantryParameter(path_cap_galaxy);                    //目前设定为，初始化即为步兵模式，并用工业相机

    if (path_video != "") {
        setting.is_video = false;
//        setting.path_video = path_video;
//        setting.setVideoParameter(path_video);
        setting.is_short_only = false;
        setting.is_prime_only = true;
    }
#endif
}

ThreadTask::~ThreadTask() {

}

// 图像生成线程
void ThreadTask::imageProduce() {

    Data& data = Data::getData();
    Parameter& param = Parameter::getParameter();
#ifdef GUI
    {
        std::unique_lock<std::mutex> ul(mtx_init);
        cond_var.wait(ul, [this]()->bool {
                return is_ready || flag_end_thread;
            }
        );
    }
#endif

    if (flag_end_thread)
        return;

    std::shared_ptr<Capture> cap_short;		//短焦/单摄             //新建智能指针对象cap_short
    if (setting.is_video)	//视频模式
        cap_short.reset(new Capture(setting.path_video, setting.sz_short));     //取消智能指针cap_short与相关指针的关联，并指向新建的Capture类对象
    else {
        cap_short.reset(new Capture(setting.sz_short, setting.length_f_short,
                CAPTURE_SHORT, setting.type_driver_short));
    }

    std::shared_ptr<Capture> cap_long;                             //新建智能指针对象cap_short
	if (setting.is_short_only == false)
        cap_long.reset(new Capture(setting.sz_long, setting.length_f_long,
                CAPTURE_LONG, setting.type_driver_long));

    std::shared_ptr<Capture> cap_prime = cap_short;

    CameraChooser chooser_cap(setting);                            //切换相机策略，长->短或短->长

    while(1) {

        if (param.other.mode_detect == DETECT_ARMOR) {
            if (setting.is_switch_cap) {
                if (setting.is_short_only == false && chooser_cap.chooseCamera()) {
                    if (data.mode_cap == CAPTURE_SHORT)
                        cap_prime = cap_short;
                    else
                        cap_prime = cap_long;
                    data_cap.reset();
                }
            }
            else if (cap_prime == cap_long) {
                cap_prime = cap_short;
                data_cap.reset();
            }
        }

        else if (param.other.mode_detect == DETECT_BUFF) {
            if (setting.is_short_only == false && cap_prime != cap_long) {
                cap_prime = cap_long;
                data_cap.reset();
            }
        }

        while (data_cap.isFull())
            if (flag_end_thread)
                return;
        //图像获取
		*cap_prime >> data_cap;

        if (flag_end_thread)
            return;
    }
}

// 图像处理线程
void ThreadTask::imageProcess() {
    Data& data = Data::getData();
    Parameter& param = Parameter::getParameter();
#ifdef GUI
    {
        std::unique_lock<std::mutex> ul(mtx_init);
        cond_var.wait(ul, [this]()->bool {
                return is_ready || flag_end_thread;
            }
        );
    }
#endif

    if (flag_end_thread)
        return;

#ifdef GUI
    UITool::log2("angle_x", 0.);
    UITool::log2("angle_y", 0.);
    UITool::log2("fps", 0.);
#endif

	cv::Mat image;
	float angle_x = 0.0;
	float angle_y = 0.0;
	float v_x = 0.0;
        float v_y = 0.0;
        float a_x = 0.0;
        float a_y = 0.0;
	double t;
	Timer timer;

//#ifdef USE_KALMANFILTER
//	AngleKalmanFilter kf(0, 0);
//#endif
	cv::VideoWriter writer;
	bool is_last_video_open = false; 
    std::shared_ptr<ArmorDetector> detector_armor(new ArmorDetector(image, setting));
    std::shared_ptr<BuffDetector> detector_buff;
	if (setting.is_buff)
        detector_buff.reset(new BuffDetector(image, setting));
    std::shared_ptr<Detector> detector = detector_armor;

    //短焦角度解算
    std::shared_ptr<AngleSolver> solver_short;
    if (setting.is_video == false)
        solver_short.reset(new AngleSolver(setting.mat_camera_short, setting.mat_coeffs_distortion_short,
                setting.length_f_short, setting));
    //长焦角度解算
    std::shared_ptr<AngleSolver> solver_long;
	if (setting.is_short_only == false)
    solver_long.reset(new AngleSolver(setting.mat_camera_long, setting.mat_coeffs_distortion_long,
            setting.length_f_long, setting));
    std::shared_ptr<AngleSolver> solver = solver_short;

    std::shared_ptr<Capture> cap_assisant;
    if (setting.is_prime_only == false)
        cap_assisant.reset(new Capture(setting.sz_short, setting.length_f_short,
                CAPTURE_SHORT, CAPTURE_DRIVER_CV));

    while (1) {
        while (data_cap.isEmpty())
            if (flag_end_thread)
                return;

		data_cap >> image;

        if (data.flag_update_param == true) {
            Parameter::update();
            data.flag_update_param = false;
        }

		if (param.other.flag_save == true && is_last_video_open == false) {
            std::time_t now = std::time(nullptr);
            std::stringstream ss;
            ss << std::put_time(std::localtime(&now), "../video/%Y-%m-%d_%H:%M:%S.avi");
			std::string str = ss.str();
            writer.open(str, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 300, cv::Size(image.cols, image.rows));
            if (writer.isOpened())
#ifdef GUI
                UITool::log("VideoWriter is opened!");
#else
                std::cout<<"VideoWriter is opened!"<<std::endl;
#endif
            else {
#ifdef GUI
                UITool::log("VideoWriter open failed!");
#else
                std::cout<<"VideoWriter open failed!"<<std::endl;
#endif
                param.other.flag_save = false;
            }
		}
		if (param.other.flag_save == true) {
			writer << image;
			is_last_video_open = true;
		}

        if (param.other.mode_detect == DETECT_ARMOR && detector != detector_armor) {
            detector = detector_armor;
            data.dist = 0;
            data.cnt_lost = 0;
        }
        else if (param.other.mode_detect == DETECT_BUFF && detector != detector_buff) {
            detector = detector_buff;
            data.dist = 0;
            data.cnt_lost = 0;
        }

        timer.start();
        bool flag_find = false;
        if (image.empty() == false)
            flag_find = detector->run();

        if (setting.is_video == false) {
            if(flag_find) {
                data.cnt_lost = 0;
                if (data.mode_cap == CAPTURE_SHORT)
                    solver = solver_short;
                else if (data.mode_cap == CAPTURE_LONG) {
                    solver = solver_long;
                    if(param.other.mode_detect == DETECT_BUFF)
                        solver->predictOffset3D();
                }
                solver->getAngle(angle_x, angle_y);
#ifdef USE_KALMANFILTER
                kf.angles_kalman = kf.run(angle_x + vec_gimbal_yaw,angle_y);
                angle_x = kf.angles_kalman.at<float>(0) - vec_gimbal_yaw;
#endif  //USE_KALMANFILTER

            }
            else {
                if (param.other.mode_detect == DETECT_ARMOR) {
                    bool flag_assisant = false;
                    if (setting.is_prime_only == false && data.is_ban_assistant==false) {
                        cv::Mat img_assisant;
                        *cap_assisant >> img_assisant;
                        flag_assisant = ArmorDetector::findArmor(img_assisant, param);
                    }
                    if (flag_assisant == false) {
                        data.cnt_lost++;
                        angle_x = 0;
                        angle_y = 0;
                        data.dist = 0;
                    }
                    else {
                        data.is_ban_assistant = true;
                        //XXX
                    }
                }
                else if (param.other.mode_detect == DETECT_BUFF && data.quadrant_buff && data.cnt_lost > 3) {
                    if (data.quadrant_buff && data.cnt_lost > 3) {
                        data.cnt_lost = 0;
                        solver->barrelReturnCenter(angle_x, angle_y);
                    }
                    else {			//这里是没有检测到就给角度为0
                        angle_x = 0;
                        angle_y = 0;
                        data.dist = 0;
                    }
                }
            }

		// std::cout<<"angle_x(yaw):"<<angle_x<<"         ";
           // std::cout<<"angle_y(pitch):"<<angle_y<<std::endl;
	    //t = timer.end();
            if (last_W_x != 0 || last_W_y != 0) //最强的滤波器
            {
                v_x = (angle_x - last_angle_x) / t;
                a_x = (v_x - last_W_x) / t;
                v_y = (angle_y - last_angle_y) / t;
                a_y = (v_y - last_W_y) / t;
                last_W_x = v_x;
                last_W_y = v_y;
                angle_x = solver->predict_x.run_position(angle_x,v_x, a_x);
                angle_y = solver->predict_y.run_position(angle_y,v_y, a_y);
            }
            else if (last_angle_x == 0.0 && last_angle_y == 0.0) //当没有历史角度时使用，只使用角度滤波器
            {
                angle_x = solver->predict_x.run_position(angle_x);
                angle_y = solver->predict_y.run_position(angle_y);
            }
            else    //存在历史角度，使用的有角加速度和角度的滤波器
            {
                v_x = (angle_x - last_angle_x) / t;
                v_y = (angle_y - last_angle_y) / t;
                last_W_x = v_x;
                last_W_y = v_y;
                angle_x = solver->predict_x.run_position(angle_x, v_x);
                angle_y = solver->predict_y.run_position(angle_y, v_y);
            }
    
            angle_x = Tool::limitAngle(angle_x, 6);
            angle_y = Tool::limitAngle(angle_y, 4);
	    last_angle_x = angle_x;
            last_angle_y = angle_y;
            port.sendXYZ(angle_x, angle_y, 0);
#ifdef GUI
            UITool::log2("angle_x", angle_x);
            UITool::log2("angle_y", angle_y);
#else
            //std::cout<<"angle_x(x) : "<<angle_x << "\t" <<"angle_y(y) : "<<angle_y<<std::endl;
#endif
        }

        //显示图像
        show();
        //t = timer.end();
#ifdef GUI
        UITool::log2("fps", 1000./t);
#else
//        std::cout<<"fps:"<<1000./t<<std::endl;
#endif
		if (param.other.flag_save == false && is_last_video_open == true) {
			writer.release();
			is_last_video_open = false;
#ifdef GUI
            UITool::log("Video has been saved!", FONT_NONE);
#else
            std::cout<<"Video has been saved!"<<std::endl;
#endif
		}

        if (flag_end_thread)
            return;
    }

}

void ThreadTask::control() {
    Parameter& param_tmp = Parameter::getTempParameter();
    auto onChange = [](int, void*) {
        Data::getData().flag_update_param = true;
    };

    namedWindow("control");
    cv::createTrackbar("armor_gray_th", "control", &param_tmp.armor.th_gray, 255, onChange);
    cv::createTrackbar("armor_color_th", "control", &param_tmp.armor.th_color, 255, onChange);


    cv::createTrackbar("armor_aim_offset_x", "control", &param_tmp.armor.pt_offset_cap.x, 200, onChange);
    cv::createTrackbar("armor_aim_offset_y", "control", &param_tmp.armor.pt_offset_cap.y, 200, onChange);

    cv::createTrackbar("buff_gray_th", "control", &param_tmp.buff.th_gray, 255, onChange);
    cv::createTrackbar("buff_color_th", "control", &param_tmp.buff.th_color, 255, onChange);
    cv::createTrackbar("buff_offset_cap_x", "control", &param_tmp.buff.pt_offset_cap.x, 200, onChange);
    cv::createTrackbar("buff_offset_cap_y", "control", &param_tmp.buff.pt_offset_cap.y, 200, onChange);
    cv::createTrackbar("buff_world_offset_x", "control", &param_tmp.buff.x_pt_offset_world, 1000, onChange);
    cv::createTrackbar("buff_offset_follow_x", "control", &param_tmp.buff.pt_offset_follow_delay.x, 200, onChange);
    cv::createTrackbar("buff_offset_follow_y", "control", &param_tmp.buff.pt_offset_follow_delay.y, 200, onChange);
    cv::createTrackbar("gimbal_offset_gravity", "control", &param_tmp.angle.gravity_offset, 10, onChange);
#ifdef CUI


    while (1) {
        std::string str;
        WindowPainter::painter >> str;
        std::regex reg("-[a-zA-Z]+(=(\\S)+)?");

        std::smatch sm;
        while (std::regex_search(str, sm, reg)) {
            std::string str_param = sm.str();

            std::string str_key;
            std::string str_value;

            auto pos = str_param.find("=", 0);
            if (pos != std::string::npos) {
                str_key = str_param.substr(0, pos);
                str_value = str_param.substr(pos+1, str_param.size());
            }
            else
                str_key = str_param;

            if (str_key == "-quit" || str_key == "-exit") {
                flag_end_thread = true;
                break;
            }
            else if (str_key == "-save")
                param_tmp.other.flag_save = !Parameter::getParameter().other.flag_save;
            else if (str_key == "mode") {
                if (str_value == "armor" && param_tmp.other.mode_detect != DETECT_ARMOR) {
#ifdef GUI
                    Tool::log("Change to ARMOR mode!");
#else
                    std::cout<<"Change to AutoAim mode!"<<std::endl;
#endif
                    param_tmp.other.mode_detect = DETECT_ARMOR;
                }
                else if (str_value == "buff" && param_tmp.other.mode_detect != DETECT_BUFF) {
#ifdef GUI
                    Tool::log("Change to BUFF mode!");
#else
                    std::cout<<"Change to AttackBuff mode!"<<std::endl;
#endif
                    param_tmp.other.mode_detect = DETECT_ARMOR;
                }
            }

            str = sm.suffix();

        }

        if (flag_end_thread)
            break;

        Data::getData().flag_update_param = true;
    }
#endif
}

void ThreadTask::getSTM32Order() {

    while (1) {
        if (flag_end_thread)
            return;
        port.receiveData(Parameter::getParameter());

        if (flag_end_thread)
            return;
    }
}

void ThreadTask::show() {
#ifdef GUI
    widget_main->display();
#else   //GUI
    cv::imshow("show",  Data::getData().img_show);
    cv::waitKey(1);
#endif  //GUI
}

int ThreadTask::exec() {


    std::thread thread_process(&ThreadTask::imageProcess, this);
    std::thread thread_produce(&ThreadTask::imageProduce, this);
    std::thread thread_receive(&ThreadTask::getSTM32Order, this);


#ifdef GUI
    int code_status = QApplication::exec();
    flag_end_thread = true;
    is_ready = true;
    cond_var.notify_all();

#else   //GUI
    control();
#endif  //GUI

    thread_process.join();
    thread_produce.join();
    thread_receive.detach();
    return code_status;
}

#ifdef GUI
void ThreadTask::onStart(int mode, QVariant param1, QVariant param2, int type) {

    std::string path_param;
    if (mode == 0) {
        switch (type) {
        case CAR_INFANTRY: default:
            if (param2.toString().size() == 0)
                setting.setInfantryParameter("../parameter/capture/"+param1.toString().toStdString());
            else
                setting.setInfantryParameter("../parameter/capture/"+param1.toString().toStdString(),
                        "../parameter/capture/"+param2.toString().toStdString());
            break;
       case CAR_GUARD:
            if (param2.toString().size() == 0)
                setting.setGuardParameter("../parameter/capture/"+param1.toString().toStdString());
            else
                setting.setGuardParameter("../parameter/capture/"+param1.toString().toStdString(),
                        "../parameter/capture/"+param2.toString().toStdString());
            break;            
       }
    }
    else if (mode == 1)
        setting.setVideoParameter("../video/"+param1.toString().toStdString());


    widget_start->close();
    widget_main = new MainWidget(win);
    win->setWidget(widget_main);
    widget_main->show();
    win->show();

    is_ready = true;
    cond_var.notify_all();

    UITool::log("Thread task start!", FONT_NONE);
    if (mode == 0) {
        switch (type) {
        case CAR_INFANTRY: default:
            UITool::log("Now it is INFANTRY mode!", FONT_NONE);
            break;
       case CAR_GUARD:
            UITool::log("Now it is GUARD mode!", FONT_NONE);
            break;
       case CAR_HERO:
            UITool::log("Now it is HERO mode!", FONT_NONE);
            break;
       }
    }
    else if (mode == 1)
        UITool::log("Now it is VIDEO mode!", FONT_NONE);
}
#endif  //GUI
