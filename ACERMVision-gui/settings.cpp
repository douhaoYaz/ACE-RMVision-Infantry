 #include "settings.h"

void Setting::save() {

    std::time_t now = std::time(nullptr);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now), "%Y-%m-%d_%H:%M:%S.xml") ;
    std::string str = ss.str();
    cv::FileStorage fs(str, cv::FileStorage::WRITE);        //写文件

	fs << "speed_bullet" << speed_bullet;
	fs << "h_barrel2ptz" << h_barrel2ptz;
	fs << "h_car" << h_car;
	fs << "pt3_ptz2camera" << pt3_ptz2camera;

    fs << "path_param_short" << path_param_short;
    fs << "is_short_only"<< is_short_only;
	if (!is_short_only) {
        fs << "path_param_long"<< path_param_long;

		fs << "is_buff"<< is_buff;
		if (is_buff) {
			fs << "h_buff"<< h_buff;
			fs << "r_buff"<< r_buff;
			fs << "rate_buff"<< rate_buff;
			fs << "dist_buff"<< dist_buff;
		}

		fs << "is_switch_cap"<< is_switch_cap;
		if (is_switch_cap) {
			fs << "th_lost_target"<< th_lost_target;
			fs << "rate_switch_cap"<< rate_switch_cap;
			fs << "th_short"<< th_short;
			fs << "th_long"<< th_long;
		}
	}

    fs << "is_prime_only" << is_prime_only;
}

void Setting::setInfantryParameter(const std::string& path_prime) {

    is_video = false;           //视频模式
    is_prime_only = true;       //只用主相机
    is_short_only = true;       //只有短焦相机
    is_switch_cap = false;      //关闭切换长短焦相机模式
    path_param_short = path_prime;  //短焦相机相关参数路径

    is_buff = true;             //开启大符检测
    h_buff = 350;               //大风车（底部装甲）高度(mm)
    r_buff = 700;               //大风车扇叶半径(mm)
    rate_buff = 0.1;            //用于判断旋转方向的刷新频率
    dist_buff = 5200;           //大风车距离(mm)

    cv::FileStorage fs("../parameter/infantry.xml", cv::FileStorage::READ);

    fs["speed_bullet"] >> speed_bullet;
    fs["h_barrel2ptz"] >> h_barrel2ptz;
    fs["h_car"] >> h_car;
    fs["pt3_ptz2camera"] >> pt3_ptz2camera;
    fs["path_param_short"] >> path_param_short;

    fs.open(path_prime, cv::FileStorage::READ);     //读取短焦相机相关参数路径
    fs["length_f"] >> length_f_short;
    fs["sz"] >> sz_short;
    fs["type_driver"] >> type_driver_short;
    fs["pt3_offset"] >> pt3_offset_short;
    fs["mat_camera"] >> mat_camera_short;
    fs["mat_coeffs_distortion"] >> mat_coeffs_distortion_short;
    fs.release();
    set();
}

void Setting::setInfantryParameter(const std::string& path_short,
        const std::string& path_long, bool flag_switch_cap) {

    is_video = false;                   //非视频模式
    is_prime_only = true;               //只用主相机
    is_short_only = false;              //不只有短焦相机
    is_switch_cap = flag_switch_cap;    //是否切换长短焦相机模式
    path_param_short = path_short;      //短焦相机相关参数路径
    path_param_long = path_long;        //长焦相机相关参数路径

    cv::FileStorage fs("../parameter/infantry.xml", cv::FileStorage::READ);

    fs["speed_bullet"] >> speed_bullet;
    fs["h_barrel2ptz"] >> h_barrel2ptz;
    fs["h_car"] >> h_car;
    fs["pt3_ptz2camera"] >> pt3_ptz2camera;
    fs["path_param_short"] >> path_param_short;

    is_buff = true;                     //开启大符检测
    fs["h_buff"] >> h_buff;
    fs["r_buff"] >> r_buff;
    fs["rate_buff"] >> rate_buff;
    fs["dist_buff"] >> dist_buff;

    fs.open(path_short, cv::FileStorage::READ);
    fs["length_f"] >> length_f_short;
    fs["sz"] >> sz_short;
    fs["type_driver"] >> type_driver_short;
    fs["pt3_offset"] >> pt3_offset_short;
    fs["mat_camera"] >> mat_camera_short;
    fs["mat_coeffs_distortion"] >> mat_coeffs_distortion_short;

    fs.open(path_long, cv::FileStorage::READ);
    fs["length_f"] >> length_f_long;
    fs["sz"] >> sz_long;
    fs["type_driver"] >> type_driver_long;
    fs["pt3_offset"] >> pt3_offset_long;
    fs["mat_camera"] >> mat_camera_long;
    fs["mat_coeffs_distortion"] >> mat_coeffs_distortion_long;
    fs.release();

    th_lost_target = 200;                  //丢失目标帧阈值为200帧，到达后短->长
    rate_switch_cap = 0.5;                 //刷新频率
    th_short = 1300;                       //长->短距离阈值(mm)
    th_long = 1600;                        //短->长距离阈值(mm)

    set();
}

void Setting::setGuardParameter(const std::string& path_prime) {

    is_video = false;
    is_prime_only = true;
    is_short_only = true;
    is_switch_cap = false;
    path_param_short = path_prime;

    cv::FileStorage fs("../parameter/guard.xml", cv::FileStorage::READ);

    fs["speed_bullet"] >> speed_bullet;
    fs["h_barrel2ptz"] >> h_barrel2ptz;
    fs["h_car"] >> h_car;
    fs["pt3_ptz2camera"] >> pt3_ptz2camera;
    fs["path_param_short"] >> path_param_short;

    fs.open(path_prime, cv::FileStorage::READ);

    fs["length_f"] >> length_f_short;
    fs["sz"] >> sz_short;
    fs["type_driver"] >> type_driver_short;
    fs["pt3_offset"] >> pt3_offset_short;
    fs["mat_camera"] >> mat_camera_short;
    fs["mat_coeffs_distortion"] >> mat_coeffs_distortion_short;
    fs.release();

    set();
}

void Setting::setGuardParameter(const std::string &path_prime, const std::string& path_assistant) {

    is_video = false;
    is_prime_only = false;
    is_short_only = true;
    is_switch_cap = false;
    path_param_short = path_prime;
    this->path_assistant = path_assistant;

    cv::FileStorage fs("../parameter/guard.xml", cv::FileStorage::READ);

    fs["speed_bullet"] >> speed_bullet;
    fs["h_barrel2ptz"] >> h_barrel2ptz;
    fs["h_car"] >> h_car;
    fs["pt3_ptz2camera"] >> pt3_ptz2camera;
    fs["path_param_short"] >> path_param_short;

    fs.open(path_prime, cv::FileStorage::READ);
    fs["length_f"] >> length_f_short;
    fs["sz"] >> sz_short;
    fs["type_driver"] >> type_driver_short;
    fs["pt3_offset"] >> pt3_offset_short;
    fs["mat_camera"] >> mat_camera_short;
    fs["mat_coeffs_distortion"] >> mat_coeffs_distortion_short;
    fs.release();

    set();
}

void Setting::setVideoParameter(const std::string& path_video) {

    is_video = false;
    is_prime_only = true;
    is_short_only = true;
    is_switch_cap = false;
    is_video = true;
    this->path_video = path_video;

    is_buff = true;
    h_buff = 350;
    r_buff = 700;
    rate_buff = 0.1;
    dist_buff = 5200;

    set();
}
