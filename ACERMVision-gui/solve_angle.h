#ifndef SOLVE_ANGLE_H_INCLUDE
#define SOLVE_ANGLE_H_INCLUDE

#include "settings.h"
#include "utils.h"
#include <opencv2/opencv.hpp>

#define PI 3.1415926

class AngleSolver {
public:

	std::vector<cv::Point3f> pt3s_obj;	//pnp解算的目标点

	cv::Mat mat_ptobj;	
	cv::Mat& mat_camera;				//相机内参矩阵
	cv::Mat& mat_coeffs_distortion;		//相机畸变系数
	int length_f;						//相机焦距
		
    float speed_bullet;             //子弹速度
    float dist_buff;				//能量机关到桥头水平距离(mm)
    float h_buff;					//大能量机关最底部装甲板(中心)与桥面高度(mm)
    float h_car;					//步兵枪口距离桥面高度(mm)
    float r_buff;					//扇叶半径(mm)
    float h_barrel2ptz;             //枪管与云台的相对高度(mm)
    cv::Point3f pt3_ptz2camera;     //云台到相机的补偿(mm)
								//x+ camera is left on ptz, x- camera is right on ptz
								//y+ camera is up on ptz, y- camera is down on ptz
								//z+ camera is back on ptz, z- camera is front on ptz

    int quadrant;       //大风车所在像项
    bool flag_reset;    //枪管归中复位标志
    
    float dist_overlap; //距离补偿（？）
    Predict predict_x;
    Predict predict_y;

	AngleSolver(cv::Mat& matrix_camera, cv::Mat& matrix_coeffs_distortion,
            int length_focus, Setting& setting): mat_camera(matrix_camera),
            mat_coeffs_distortion(matrix_coeffs_distortion), length_f(length_focus),
            speed_bullet(setting.speed_bullet), dist_buff(setting.dist_buff),
            h_buff(setting.h_buff), h_car(setting.h_car), r_buff(setting.r_buff),
            h_barrel2ptz(setting.h_barrel2ptz) {

        pt3_ptz2camera = static_cast<cv::Point3f>(setting.pt3_ptz2camera);
        cv::Mat(pt3s_obj).convertTo(mat_ptobj, CV_32F);
	}


	/**
	 *@brief 根据当前模式获取角度
	 *@输出：
	 *	@param angle_x 云台yaw轴角度
	 *	@param angle_y 云台pitch轴角度
	 */
    void getAngle(float& angle_x, float& angle_y);

	/**
	 *@brief 装甲识别的角度解算
	 *@输出：
	 *	@param angle_x 云台yaw轴角度
	 *	@param angle_y 云台pitch轴角度
	 */
	void getAimAngle(float& angle_x, float& angle_y);

	/**
     *@brief 打符模式的角度解算
	 *@输出：
	 *	@param angle_x 云台yaw轴角度
	 *	@param angle_y 云台pitch轴角度
	 */
	void getBuffAngle(float& angle_x, float& angle_y);

	/**
	 *@brief 能量机关云台pitch补偿
	 */
	float getOffsetGravity(float dist, float tvec_y);

	/**
	 *@brief 生成装甲板世界坐标
	 */
    void generate3DPoints();

    /**
     *@brief 预测装甲板世界坐标补偿用于后面角度解算器生成世界坐标
     */
    void predictOffset3D();

    /**
     *@brief 大符模式的风车实时转速
     */
    void getRotationSpeed();

    /**
     *@brief 打符枪管复位归中
     */
	void barrelReturnCenter(float& angle_x, float& angle_y);


};

#endif // SOLVE_ANGLE_H_INCLUDE
