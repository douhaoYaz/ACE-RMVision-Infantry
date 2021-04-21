#ifndef DETECTOR_H_INCLUDE
#define DETECTOR_H_INCLUDE

#include "settings.h"

/**
 *@brief 检测器虚类
 */
class Detector {
public:
    cv::Mat& img;			//输入的原图

    /**
     *@brief 初始化
     *@param img_src   要读入的相机画面
     *@param setting   读入的固定参数
	 *@param data_tmp  读入的交互参数
     *@param parameter 读入的输入参数
     */
    Detector(cv::Mat& img_src, Setting& setting ): img(img_src) { }

    virtual ~Detector();

    /**
     *@brief 运行，纯虚模板
     *@return 是否识别到
     */
    virtual bool run()=0;

};

#endif // DECISIONMAKER_H_INCLUDE
