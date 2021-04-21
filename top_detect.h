#ifndef TOP_DETECT_INCLUDE
#define TOP_DETECT_INCLUDE

#include "utils.h"
#include "detector.h"
#include "armor_detect.h"

#include <opencv2/dnn.hpp>


/**
 * @brief 上次检测的装甲的历史数据
 */
class TopHistory {
public:
    int type_last;      //上次的装甲类型
    bool flag;          //数据是否有效
    int id_last;        //上次得到的装甲id
    cv::Rect rect_last;	//上次的装甲板矩形位置
    LEDPair virual_pair_last;//上次的虚拟灯条对，主要用于特殊情况下虚拟装甲板位置的保持
    int cnt_update;     //expend计数器
    int cnt_max;        //计数器最大值

    /**
     *@brief 唯一构造
     *@param data_tmp   交互数据
     */
    TopHistory(): type_last(-1), flag(false),
        id_last(0), cnt_max(10) { }

    /**
     *@brief 根据装甲更新
     *@param arm    要更新的装甲数据
     *@param pt_roi roi图像偏移，即roi左上角点在源图像位置
     */
    void update(Armor arm, cv::Point pt_roi);

    /**
     *@brief 没有检测到的情况，根据计数器决定是否重置
     *@param sz_img 图像尺寸
     */
    void update(cv::Size sz_img);
};

/**
 *@brief 装甲识别类
 *@通过拟合灯条来识别装甲
 */
class TopDetector: public Detector {
public:
    cv::Mat img_roi;            //图像roi区域
    cv::Point pt_roi;           //roi图像偏移，即roi左上角点在源图像的位置
    TopHistory history;       //上次装甲历史
    ArmorClassifier classifier;	//装甲分类器
    DestoryableWindow win_roi;	//ROI对应的窗口
    bool is_useroi;             //是否使用roi
    bool is_classifier;         //是否使用分类器
    int rule_choose;            //选择装甲的规则
    bool is_ignore2;			//是否忽略工程

#if TYPE_ARMOR_CLASSIFIER == 0
    /**
     *@brief 初始化
     *@param img_src   要读入的相机画面
     *@param setting   读入的固定参数
     *@param data_tmp  读入的交互参数
     */
    TopDetector(cv::Mat& img_src, Setting& setting, Data& data_tmp):
            Detector(img_src, setting, data_tmp),
            classifier(setting.path_classifier_arm_svm),
            history(data_tmp) { }

#elif TYPE_ARMOR_CLASSIFIER == 1
        /**
     *@brief 初始化
     *@param img_src   要读入的相机画面
     *@param setting   读入的固定参数
     *@param data_tmp  读入的交互参数
     */
    TopDetector(cv::Mat& img_src, Setting& setting) :
            Detector(img_src, setting),
            classifier(setting.path_classifier_arm_caffe_net,
            setting.path_classifier_arm_caffe_model) { }

#endif

    /**
     *@brief  装甲识别的主要函数
     *@return 是否识别到装甲
     */
    bool run() override;

private:
    /**
     *@brief 根据上次装甲设置ROI
     */
    void setROI();

    /**
     *@beief 找出图像中的灯条，并进行配对
     *@输入&输出：
     *  @img_gray   传入的灰度图，作为灯条判断的依据之一，输出时被二值化
     *	@led_sticks 灯条集，已配对
     */
    void findLamps(cv::Mat& img_gray, SticksSet& led_sticks);

    /**
     *@brief 根据灯条来拟合装甲(全匹配)
     *输入：
     *  @intensity_avg 装甲区域内的最小平均亮度
     *  @angle_err     两灯条的最大误差角度
     *输出：
     *	@arms 得到的装甲集
     */
    void fitArmor(SticksSet& led_sticks,std::vector<Armor>& arms, float angle_err=5.5f,float intensity_avg=100.f);

    /**
     *@brief 根据灯条分布情况拟合生成一块虚拟装甲
     *@输入：
     *	@arms 装甲集
     *@输出：
     *	@pair_target 目标装甲
     */
    void generateArmor(SticksSet& led_sticks,std::vector<Armor>& arms, LEDPair& pair_target);

    /**
     *@brief 从灯条对中获取点，用于角度解算
     *@param pair_target 目标装甲
     */
    void getPoints(LEDPair& pair_target);

    /**
     *@brief 得到roi点在原图的位置
     *@param pt roi上要转换的点
     *@return 原图上的点
     */
    template<class _Pt>
    _Pt convertSourcePoint(_Pt pt) {
        return pt + static_cast<_Pt>(pt_roi);
    }

    /**
     *@brief 将相对roi的矩形转换到原图上
     *@param rect roi上要转换的矩形
     *@return 原图上的矩形
     */
    template<class _Rect>
    _Rect convertSourceRectange(_Rect rect) {
        rect.x += pt_roi.x;
        rect.y += pt_roi.y;
        return rect;
    }

    /**
     *@brief 将相对roi的旋转矩形转换到原图上
     *@param rrect roi上要转换的旋转矩形
     *@return 原图上的旋转矩形
     */
    template<class _Rrect>
    _Rrect convertSourceRotatedRectange(_Rrect rrect) {
        rrect.center = convertSourcePoint(rrect.center);
        return rrect;
    }
};

#endif	//TOP_DETECT_H_INCLUDE
