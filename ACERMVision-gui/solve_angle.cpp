#include "solve_angle.h"
#include "utils.h"
#include "ui_tool.h"

void AngleSolver::getAngle(float& angle_x, float& angle_y) {
    Parameter& param = Parameter::getParameter();
    generate3DPoints();
    if (param.other.mode_detect == DETECT_ARMOR)
        getAimAngle(angle_x, angle_y);
    else if (param.other.mode_detect == DETECT_BUFF)
        getBuffAngle(angle_x, angle_y);
}

void AngleSolver::getAimAngle(float& angle_x, float& angle_y) {

    Data& data = Data::getData();
	cv::Mat tvec;   //平移向量，3*1
	cv::Mat rvec;   //平移向量，3*1，(pitch, yaw, roll)
    cv::solvePnP(pt3s_obj, data.pts_2d, mat_camera, mat_coeffs_distortion, rvec, tvec);

    double rm[3][3];
    cv::Mat mat_rot(3, 3, CV_64FC1, rm);
    cv::Rodrigues(rvec, mat_rot);

    //这里的角度本来是用于r_data进行坐标系调整
    //float theta_y = Tool::radian2Angle(atan2(static_cast<float>(rm[1][0]), static_cast<float>(rm[0][0])));//x
	//theta_y = atan2(-rm[2][0], sqrt(rm[2][0] * rm[2][0] + rm[2][2] * rm[2][2])) * 57.2958;//y
	//theta_y = atan2(rm[2][1], rm[2][2]) * 57.2958;//z

    // 相机坐标系=>云台坐标系
    //相机+枪管到云台的pitch角(很小)
    double theta = -atan(static_cast<double>(pt3_ptz2camera.y + h_barrel2ptz))
			/ static_cast<double>(dist_overlap);

    std::cout << "dist_overlap:\t" << dist_overlap << std::endl;

    double r_data[] = {
        1, 0,           0,
        0, cos(theta),  sin(theta),
        0, -sin(theta), cos(theta)
    };
    double t_data[] = {
        static_cast<double>(pt3_ptz2camera.x),
        static_cast<double>(pt3_ptz2camera.y),
        static_cast<double>(pt3_ptz2camera.z)
    };

    cv::Mat r_camera_ptz(3, 3, CV_64FC1, r_data);
    cv::Mat t_camera_ptz(3, 1, CV_64FC1, t_data);

    cv::Mat position_in_ptz = r_camera_ptz * tvec - t_camera_ptz;

    const double* _xyz = (const double*)position_in_ptz.data;

    //重力补偿pitch角度
    double t_down = 0.0;	//落地时间，s
    if(speed_bullet > 10e-3)
        t_down = _xyz[2] /1000.0 / speed_bullet;
    double offset_gravity = 0.5 * 9.8 * t_down * t_down * 1000;

//#ifdef AIM_SET_ZEROS_GRAVITY
    offset_gravity = 0;
//#endif
    double xyz[3] = { _xyz[0], _xyz[1] - offset_gravity, _xyz[2] };
    data.dist = static_cast<float>(xyz[2]);
#ifdef GUI
    UITool::log2("armor_dist", data.dist);
#else
    std::cout<<"armor_dist"<<data.dist<<std::endl;
#endif

    double alpha = 0.0;
    double thta = 0.0;
    h_barrel2ptz = 0;
    alpha = asin(static_cast<double>(h_barrel2ptz) / sqrt(xyz[1]*xyz[1] + xyz[2]*xyz[2]));
    if (xyz[1] < 0) {
        thta = atan(-xyz[1]/xyz[2]);
        angle_y = static_cast<float>(-(alpha+thta)); //camera coordinate
    }
    else if (xyz[1] < static_cast<double>(h_barrel2ptz)) {
        thta = atan(xyz[1]/xyz[2]);
        angle_y = static_cast<float>(-(alpha - thta));
    }
    else {
        thta = atan(xyz[1]/xyz[2]);
        angle_y = static_cast<float>((thta-alpha));   // camera coordinate
    }
    angle_x = static_cast<float>(atan2(xyz[0], xyz[2]));
    angle_x = Tool::radian2Angle(angle_x);
    angle_y = Tool::radian2Angle(angle_y);
    std::cout<<"angle_x(x) : "<<angle_x << "\t" <<"angle_y(y) : "<<angle_y<<std::endl;
}

void AngleSolver::getBuffAngle(float &angle_x, float &angle_y) {
    Data& data = Data::getData();
    //ACE大风车参数：
    //             大能量机关最底部装甲板(中心)与桥面高度 35cm
    //             能量机关到桥头水平距离  5.2m
    //             大风车半径 70cm
    //             大风车中心离地高度 2283mm

    //步兵车参数：
    //             步兵枪口距离桥面高度   41cm
    //             大风车半径 70cm
    //             相机到云台水平距离17cm


    float h_delta = h_buff - h_car;			//枪口距离大能量机关最底部装甲板高度

    float angle = data.angle_buff + data.angle_pre;	//能量机关装甲板实际预测角度=装甲板本身角度+预测角度(顺逆时针方向值不同)
    float h_buff = r_buff * sin(Tool::angle2Radian(angle)) + r_buff;   // 计算能量机关装甲板相对最底面装甲高度　0－1600(mm)

    float h_target = h_delta + h_buff;                    //枪口到装甲板垂直高度(mm)
    float dist = sqrt(pow(h_target, 2) + pow(dist_buff, 2));            //枪口到装甲板的实际空间上距离

	// 姿态结算（得到相机坐标系），（已补偿世界坐标，即运动预测完毕）
    cv::Mat rvec;
	cv::Mat tvec;   //平移向量，3*1
    cv::solvePnP(pt3s_obj, data.pts_2d, mat_camera, mat_coeffs_distortion, rvec, tvec);
    tvec.at<double>(2, 0) = dist;


    //相机+枪管到云台的pitch角(很小)
    double theta = -atan(static_cast<double>(pt3_ptz2camera.y + h_barrel2ptz)) / static_cast<double>(dist_overlap);

    //旋转向量
    double r_data[] = {
        1, 0,           0,
        0, cos(theta),  sin(theta),
        0, -sin(theta), cos(theta)
    };
    //平移向量
    double t_data[] = {
        static_cast<double>(pt3_ptz2camera.x),
        static_cast<double>(pt3_ptz2camera.y),
        static_cast<double>(pt3_ptz2camera.z)
    };

    cv::Mat r_camera2ptz(3, 3, CV_64FC1, r_data);
    cv::Mat t_camera2ptz(3, 1, CV_64FC1, t_data);
    cv::Mat position_in_ptz;

    // 坐标系转换 -摄像头坐标系到云台坐标系
    //position_in_ptz = r_camera_ptz * tvec - t_camera_ptz;
    position_in_ptz = tvec - t_camera2ptz;

    const double* _xyz = (const double *)position_in_ptz.data;

    // 计算角度（现在开始是计算云台到目标点距离）
    double xyz[3] = { _xyz[0], _xyz[1], dist };//x轴(yaw)，y轴(pitch)，z轴 --相机坐标系

	//计算x(yaw轴)
    angle_x = static_cast<float>(atan2(xyz[0],xyz[2]));
	angle_x = Tool::radian2Angle(static_cast<float>(angle_x));	//yaw轴需要旋转的角度

	//计算y(pitch轴)
    angle_y = static_cast<float>(atan2(xyz[1], xyz[2]));

#ifdef USE_GIMBAL_OFFSET
    float gimbal_y = dist * sin(gravity_offset*3.14/180);
    angle_y = -getOffsetGravity(dist/1000, (gimbal_y - xyz[1] )/1000, ballet_speed);
    angle_y += gimbal_pitch*CV_PI/180;
#endif
#ifdef USE_GRAVITY_OFFSET
    float angle_theta = -static_cast<float>(atan2(xyz[1],dist)); // 云台与目标点的相对角度（假设此时云台的pitch轴角度为0）
    float angle_beta = static_cast<float>(atan2(h_target,dist)) - angle_theta; // 云台与地面的相对角度(实际需要的pitch轴角度)
    angle_y = -getOffsetGravity(dist/1000, (h_target)/1000);
    angle_y += angle_beta;
#endif
    angle_y = Tool::radian2Angle(static_cast<float>(angle_y));
}


//重力云台pitch轴补偿(getBuffPitch)
float AngleSolver::getOffsetGravity(float dist, float tvec_y) {
    // 申明临时y轴(pitch轴)方向长度,子弹实际落点，实际落点与击打点三个变量不断更新（mm）
    float y_temp, y_actual, dy;

    // 重力补偿枪口抬升角度
    float angle = 0.0;
    float gravity = 9.7887f; //shenzhen 9.7887
    y_temp = tvec_y;
    // 迭代求抬升高度
    for (int i = 0; i < 10; i++) {
        // 计算枪口抬升角度
        angle = (float)atan2(y_temp, dist);
        // 计算实际落点
        float t;
        t = dist / (speed_bullet * cos(angle));
        y_actual = speed_bullet * sin(angle) * t - gravity * t * t / 2;
        dy = tvec_y - y_actual;
        y_temp = y_temp + dy;
        // 当枪口抬升角度与实际落点误差较小时退出
        if (fabsf(dy) < 0.01)
            break;
        
    }
    return angle;
}

void AngleSolver::generate3DPoints() {
    Data& data = Data::getData();
    pt3s_obj.clear();
    float width = 0.0;
    float height = 0.0;
	cv::Point2f pt_offset;
    switch (data.type_target) {
    case TARGET_SMALL_ARMOR:
        width = 140;      // mm the armor two led width
        height = 60;      // mm the armor led height
        pt_offset = data.pt_offset_world_armor;
        break;
    case TARGET_BIG_ARMOR:     // big_armor
        width = 230;//230;//140;      // mm the armor two led width
        height = 60;      // mm the armor led height
        pt_offset = data.pt_offset_world_armor;
        break;

    case TARGET_BUFF_ARMOR:    // buff_armor
        width = 230;//230;//140;      // mm the armor two led width
        height = 130;      // mm the armor led height
        pt_offset = data.pt_offset_world_buff;
        break;
    }

    float x = width / 2;
    float y = height / 2;
    float z = 0;
    //待解算的世界3D坐标（+补偿）
    pt3s_obj.push_back(cv::Point3f(-x, -y, z)+cv::Point3f(pt_offset.x, pt_offset.y, 0));
    pt3s_obj.push_back(cv::Point3f(x, -y, z)+cv::Point3f(pt_offset.x, pt_offset.y, 0));
    pt3s_obj.push_back(cv::Point3f(x, y, z)+cv::Point3f(pt_offset.x, pt_offset.y, 0));
    pt3s_obj.push_back(cv::Point3f(-x, y, z)+cv::Point3f(pt_offset.x, pt_offset.y, 0));
}

//预测实际击打点世界3D坐标补偿(以装甲板中心为世界坐标原点)
void AngleSolver::predictOffset3D() {
    Data& data = Data::getData();
    Parameter& param = Parameter::getParameter();
    float x_world_offset,y_world_offset;

    if(param.other.type_buff == SMALL_RUNE) {
        //pt_offset_world(750,500)
        x_world_offset = param.buff.x_pt_offset_world - 500;
        // y = r - sqrt(x^2)
        y_world_offset = r_buff - pow((r_buff*r_buff - pow(x_world_offset, 2)), 0.5);
    }else if (param.other.type_buff == BIG_RUNE) {
        getRotationSpeed();

        float time_step = 0.5;
        data.angle_pre = data.real_spd * time_step;

        x_world_offset = r_buff * sin(data.angle_pre);
        y_world_offset = r_buff * (1-cos(data.angle_pre));
    }
    if (param.other.direction == BUFF_CLOCKWISE) {
        data.pt_offset_world_buff = cv::Point2f(-(x_world_offset+param.buff.pt_offset_follow_delay.x),
                -(y_world_offset+param.buff.pt_offset_follow_delay.y));
        data.angle_pre = atan(x_world_offset / (r_buff - y_world_offset));
    }
    else if (param.other.direction == BUFF_ANTICLOCKWISE) {
        data.pt_offset_world_buff = cv::Point2f(x_world_offset+param.buff.pt_offset_follow_delay.x,
                -(y_world_offset+param.buff.pt_offset_follow_delay.y));
        data.angle_pre = -atan(x_world_offset / (r_buff - y_world_offset));
    }
    else if (param.other.direction == BUFF_NONE) {
        data.pt_offset_world_buff = cv::Point2f(0, 0);
        data.angle_pre = 0;
    }
}

void AngleSolver::getRotationSpeed() {
    Data& data = Data::getData();
    //新增加一环世界坐标补偿，通过时间差拟合得到偏移量(50次)
    if(data.sample_count < 15){
        //利用极短时间内两个装甲板的旋转角度差得到位移，进而得到此时的时间t方便后面预测
        //收集点集
        if(data.sample_count > 0){
            data.buff_timer.end();
            float delta_t = data.buff_timer.calculateAverageTime();
            data.buff_timer.start();

            float theta_rotate = data.angle_buff - data.angle_buff_last;
            float spd = theta_rotate * 2 * PI /360;
            data.orig_t = (asin((spd-1.305) / 0.785) / 1.884) - delta_t;

            data.temp_x.push_back(data.sample_count);
            data.temp_y.push_back(data.orig_t);
        }

        data.sample_count++;

        data.buff_timer.start();
    }else if(data.sample_count >= 15)
    {
        //大能量机关转速按照三角函数呈周期变化。速度目标函数为：spd = 0.785 ∗ sin (1.884 ∗ t) + 1.305，其中spd 的单位为 rad/s，t 的单位为 s
        //由于速度启动时间t0未知，故spd = 0.785 ∗ sin (1.884 ∗ (t0 + Δt)) + 1.305
        //拟合得到一个较准确的时间来代入公式直接补偿偏移

        //由于asin函数计算的弧度区间为 [-pi/2,+pi/2]其对应初始时间orig_t,故需要额外判断是否位于[-pi,-pi/2]或[+pi/2,+pi]其对应初始时间为T/2-orig_t
        //根据采集到数据进行最小二乘拟合出斜率，若斜率k>0,则打击大符程序启动执行初始时间为orig_t，若斜率k<0,则初始时间为T/2-orig_t

        cv::Point2f point_coefficient = Tool::LineFitting(data.temp_x, data.temp_y, data.sample_count-1);

        if(point_coefficient.x < 0){
            data.orig_t = (PI/1.884) - data.orig_t;
        }

        data.buff_timer.end();
        float real_t = data.orig_t + data.buff_timer.calculateAverageTime();

        //计算当前的实时转速并适当增加时间步长
        data.real_spd = (0.785*sin(1.884*real_t)) + 1.305;
    }
}

void AngleSolver::barrelReturnCenter(float& angle_x, float& angle_y) {
    Data& data = Data::getData();
	//param.buff_Origin 停留在上一次检测值
	float th_x_reset_angle = 16;
	float th_y_reset_angle = 16;
	float angle_x_reset = 32;
	float angle_y_reset = 16;
	float dist_x_reset = 0;
	float dist_y_reset = 0;
	//复位可以,角度极性正确
	//丢失目标时归中复位待找到未激活扇叶(以最后一次检测到的装甲位置)

	int quadrant = data.quadrant_buff;
	data.quadrant_buff = -1;

	switch (quadrant) {
	case BUFF_ORIGIN:
		if (flag_reset) {
			float th_angle = 0.2;
			float factor = 0.7;
			angle_x = Tool::smoothAngleChange(angle_x, factor, th_angle);
			angle_y = Tool::smoothAngleChange(angle_y, factor, th_angle);

			if (angle_x == 0 && angle_y == 0)
				flag_reset = false;
		}
		else {
			angle_x = 0;
			angle_y = 0;
		}
		break;

		//云台抖动较大(有点震荡)待优化,暂时效果不如给直接数,比例复位的测试好像有点问题待测
	case BUFF_QUADRANT1:
		flag_reset = true;

		//dist_x_reset =r_buff*cos(180-param.angle_buff);
		//angle_x_reset = th_x_reset_angle * (dist_x_reset / r_buff);
		//dist_y_reset =r_buff*sin(180-param.angle_buff);
		//angle_y_reset = th_y_reset_angle * (dist_y_reset / r_buff);
		angle_x = -angle_x_reset;
		angle_y = angle_y_reset;

		break;

	case BUFF_QUADRANT2:
		flag_reset = true;

		//dist_x_reset =r_buff*cos(param.angle_buff);
		//angle_x_reset = th_x_reset_angle * (dist_x_reset / r_buff);
		//dist_y_reset =r_buff*sin(param.angle_buff);
		//angle_y_reset = th_y_reset_angle * (dist_y_reset / r_buff);

		angle_x = angle_x_reset;
		angle_y = angle_y_reset;
		break;

	case BUFF_QUADRANT3:
		flag_reset = true;

		//dist_x_reset =r_buff*cos(360-param.angle_buff);
		//angle_x_reset = th_x_reset_angle * (dist_x_reset / r_buff);
		//dist_y_reset =r_buff*sin(360-param.angle_buff);
		//angle_y_reset = th_y_reset_angle * (dist_y_reset / r_buff);

		angle_x = angle_x_reset;
		angle_y = -angle_y_reset;
		break;
	case BUFF_QUADRANT4:
		flag_reset = true;

		//dist_x_reset =r_buff*cos(param.angle_buff-180);
		//angle_x_reset = th_x_reset_angle * (dist_x_reset / r_buff);
		//dist_y_reset =r_buff*sin(param.angle_buff-180);
		//angle_y_reset = th_y_reset_angle * (dist_y_reset / r_buff);

		angle_x = -angle_x_reset;
		angle_y = -angle_y_reset;
		break;

	}
}



