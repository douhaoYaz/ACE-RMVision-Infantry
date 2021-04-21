 #include "settings.h"

void Setting::save() {

    std::time_t now = std::time(nullptr);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now), "%Y-%m-%d_%H:%M:%S.xml") ;
    std::string str = ss.str();
    cv::FileStorage fs(str, cv::FileStorage::WRITE);

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

    is_video = false;
    is_prime_only = true;
    is_short_only = true;
    is_switch_cap = false;
    path_param_short = path_prime;

    is_buff = true;
    h_buff = 350;
    r_buff = 700;
    rate_buff = 0.1;
    dist_buff = 5200;

    cv::FileStorage fs("../parameter/infantry.xml", cv::FileStorage::READ);

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

void Setting::setInfantryParameter(const std::string& path_short,
        const std::string& path_long, bool flag_switch_cap) {

    is_video = false;
    is_prime_only = true;
    is_short_only = false;
    is_switch_cap = flag_switch_cap;
    path_param_short = path_short;
    path_param_long = path_long;

    cv::FileStorage fs("../parameter/infantry.xml", cv::FileStorage::READ);

    fs["speed_bullet"] >> speed_bullet;
    fs["h_barrel2ptz"] >> h_barrel2ptz;
    fs["h_car"] >> h_car;
    fs["pt3_ptz2camera"] >> pt3_ptz2camera;
    fs["path_param_short"] >> path_param_short;

    is_buff = true;
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

    th_lost_target = 200;
    rate_switch_cap = 0.5;
    th_short = 1300;
    th_long = 1600;

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
