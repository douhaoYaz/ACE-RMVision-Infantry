#ifndef BUFF_DETECT_H_INCLUDE
#define BUFF_DETECT_H_INCLUDE

#include "detector.h"

using namespace cv;
using namespace std;

/**
 *@brief 能量机关扇叶状态
 *@BUFF_ +:
 *	@SMALL	0，未知状态
 *	@BIG	1，未激活
 *	@NULL	2，已激活
 */
enum BuffFlabellumType {
    BUFF_UNKNOW = 0,
    BUFF_INACTION = 1,
    BUFF_ACTION = 2
};


//           ↑
//          90
//           |
//           |
// 0         |
// ----------|----------180→
// 360       |
//           |
//           |
//          270

/**
 *@brief 能量机关扇叶类
 *@在逻辑识别部分需要修改原有旋转矩形属性
 *@在计算0-360角度上需要用到旋转矩形原始参数
 */
class BuffFlabellum {
public:

	cv::RotatedRect rrect_small;		//能量机关扇叶内轮廓(即装甲板或其他内部轮廓)
	cv::RotatedRect rrect_big;          //能量机关扇叶外轮廓

	std::vector<cv::Point2f> pts_2d;	//pnp角度解算的四个装甲点

	float angle;						//装甲板的角度
	float angle_diff;					//角度差
	int type;							//扇叶类型，激活or未激活

	BuffFlabellum() { }

	/**
	 *@brief 用轮廓矩形构造扇叶
	 *@param small 小矩形，即装甲板位置
	 *@param big   大矩形，即外部轮廓部分
	 */
	BuffFlabellum(cv::RotatedRect small, cv::RotatedRect big):
			rrect_small(small), rrect_big(big), type(BUFF_UNKNOW) {

        angle_diff = fabsf(rrect_big.angle - rrect_small.angle);
	}

    /**
     *@brief 绘制最终装甲板的中心点
     *@param image 要处理的原图
     */
    void drawTarget(cv::Mat &img);

    /**
     *@brief 更新能量机关装甲板的绝对位置
     */
    void update2DPoints();

    /**
     *@brief ，根据两个ROI区域亮度值判断能量机关扇叶的状态（激活or未激活）
     *@param image 要处理的原图
     */
    void getStatus(cv::Mat& img);
};

class BuffDetector: public Detector {
public:

    //判断方向用到的一些参数
    float angle_buff;		//获取到的目标旋转角度，用于获取旋转方向计算时用到
    float angle_diff;		//两帧图像中目标的角度差，用于判断旋转方向
    float d_angle;			//用于判断旋转方向的变量
    float rate;				//r为刷新频率，介于0.0 - 1.0之间
    float angle_last;       //上一帧目标的旋转角度
    uint cnt_find;			//检测到的帧数
    int direction;			//检测到的风车方向
    float r_buff;           //大风车扇叶半径

	/**
     *@brief 读取初始参数
     *@param image     要处理的原图
     *@param setting   读入的固定设置
	 *@param data_tmp  临时数据
     *@param parameter 可调参数
	 */
    BuffDetector(cv::Mat& image, Setting& setting):
            Detector(image, setting), rate(setting.rate_buff),
            r_buff(setting.r_buff) { }


	/**
     *@brief 能量机关识别的总任务
     *@return 是否识别到击打区域
	 */
    bool run() override;


private:

    /**
     *@brief 找出图像中的能量机关扇叶，并进行分类
     *@param vec_target 扇叶候选目标集
     */
    void findFlabellums(std::vector<BuffFlabellum>& vec_target);

    /**
     *@brief 根据规则从扇叶集中选出最终要击打的能量机关装甲
	 *@输入：
     *	@param vec_target 候选扇叶集
     *@输出：
	 *	@target_final 目标装甲
     *	@return       选择情况
     */
    bool chooseTarget(std::vector<BuffFlabellum>& vec_target,BuffFlabellum& target_final);

    /**
     *@brief 将相对roi的矩形转换到原图上
     *@param target_final 最终装甲板
     */
    void getPoints(BuffFlabellum& target_final);

    /**
     *@brief 获取能量机关象限位置
     *@param angle 能量机关装甲板角度
	 *@return 得到的象限
     */
    int getQuadrant(float angle);

    /**
     *@brief 判断大风车旋转方向
     *@param angle 能量机关装甲板角度
	 *@return 旋转方向
     */
    int getDirection(float angle);
};


#endif //BUFF_DETECT_H_INCLUDE
