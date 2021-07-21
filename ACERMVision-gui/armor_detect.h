#ifndef ARMOR_DETECT_INCLUDE
#define ARMOR_DETECT_INCLUDE

#include "utils.h"
#include "detector.h"

#include <opencv2/dnn.hpp>

/**
 *@brief 装甲类型
 *@ARMOR_ +:
 *	@SMALL	0，小装甲
 *	@BIG	1，大装甲
 *	@NULL	2，未知类型（大概率不是装甲）
 */
enum ArmorType {
    ARMOR_SMALL	= 0,
    ARMOR_BIG	= 1,
    ARMOR_NULL	= 2
};

/**
 *@brief 装甲选择规则
 *@Armor_ +:
 *	@LAST_ID_FIRST  0，优先选择上次检测到的ID，再根据距离选择
 *	@NEAREST_FIRST  1，在图像中距补偿中心最近的装甲优先
 *                    (最靠近炮台方向而不是靠近实际距离最近)
 */
enum ArmorChooseRule {
    ARMOR_LAST_ID_FIRST = 0,
    ARMOR_NEAREST_FIRST = 1
};

/**
 *@brief 灯条
 */
struct LEDStick {

    cv::RotatedRect rrect;	//灯条的矩形区域
    float length;           //灯条长度
    bool is_match;          //是否成功配对，用于识别陀螺模式

    LEDStick() { }

    /**
     *@brief 用拟合出来的旋转矩阵构造结构体
     *@param r 灯条轮廓拟合出来的旋转矩阵
     */
    LEDStick(const cv::RotatedRect& r) {
        rrect = r;
        length = rrect.boundingRect2f().size().height;
    }
};
typedef std::vector<LEDStick> SticksSet;    //灯条集

/**
 *@brief 装甲板id和size分类器
 *@有SVM（太原理工开源）和cv::dnn+caffe两种方法
 */
class ArmorClassifier {
public:

#if TYPE_ARMOR_CLASSIFIER == 0

    cv::HOGDescriptor* hog;             //hog特征
    cv::Ptr<cv::ml::SVM> classifier;	//装甲SVM分类器

    /**
     *@brief 初始化svm分类器
     *@param path_svm svm分类器路径(.xml)
     */
    ArmorClassifier(std::string path_svm) {
        hog = new cv::HOGDescriptor(cv::Size(64, 64), cv::Size(16, 16),
                cv::Size(8, 8), cv::Size(8, 8), 9);
        classifier = cv::Algorithm::load<cv::ml::SVM>(path_svm);
    }

#elif TYPE_ARMOR_CLASSIFIER == 1

    cv::dnn::Net net;	//caffe网络

    /**
     *@brief 初始化
     *@param path_caffe_net   caffe网络文件路径(.protext/.prototxt)
     *@param path_caffe_model caffe权重文件(.caffemodel)
     */
    ArmorClassifier(std::string path_dnn_net, std::string path_dnn_model) {
        resetBackground();
        net = cv::dnn::readNetFromCaffe(path_dnn_net, path_dnn_model);
    }
#elif TYPE_ARMOR_CLASSIFIER == 2
    cv::dnn::Net net;        //lenet网络
    /**
     *@brief 初始化
     *@param path_dnn_net   lenet模型路径(.protext/.prototxt)
     */
    ArmorClassifier(std::string path_dnn_net) {
        //重置背景
        resetBackground();
        //读取lenet模型
        net = cv::dnn::readNetFromONNX(path_dnn_net);
    }
#endif

public:

    cv::Mat img_bkg;	//号码分类器一览背景

    /**
     *@brief 默认构造，背景刷白
     */
    ArmorClassifier() {
        resetBackground();
    }

    /**
     *@brief 重置背景
     */
    void resetBackground() {
        img_bkg = cv::Mat::zeros(64*2, 64*8, CV_8UC1);
    }

    /**
     *@brief 分类
     *@输入：
     *	@param img     待分类图像
     *@输出：
     *	@param type_sz 装甲大小类型
     *	@param id      得到的id
     */
    void getResult(const cv::Mat& img, int& type_sz, int& id);

    void draw(const cv::Mat& img, int sz, int id) {
        int x = id * 64;
        int y = sz * 64;
        cv::Rect rect(x, y, 64, 64);

        img.copyTo(img, img_bkg);
    }
};

/**
 *@brief 灯条对类
 *@由两个灯条构造，只要用来匹配灯条
 */
class LEDPair {
public:
    cv::Rect rect;					//灯条对构成的矩形区域
    LEDStick led_sticks[2];         //灯条对，顺序为左、右
    float rate_len; 				//长度之比
    float angle_err;                //角度之差
    cv::Point2i pt_cen;				//灯条对构成矩形的中心
    int intensity_avg;				//矩形区域内的平均强度，用来衡量是否为误判
    bool is_empty;                  //是否为空

    /**
     *@brief 默认空构造
     */
    LEDPair(): is_empty(true) {}

    /**
     *@brief 使用两灯条来拟合装甲
     */
    LEDPair(const LEDStick& l1, const LEDStick& l2);

    /**
     *@brief 计算并获取平均强度
     *@param img 装甲所在背景
     *@return 平均强度
     */
    int getAverageIntensity(cv::Mat& img) const;

    /**
     *@brief 判断是否为合适的尺寸
     *@return 是否是合适的尺寸
     */
    bool isSuitableSize() const;
};

/**
 *@brief 装甲类
 *@主要为两个灯条之间的装甲区域
 */
class Armor: public LEDPair {
public:
    // 3           2
    //  II-------II
    //  II   5   II
    //  II-------II
    // 0           1
    cv::Size size_img;  //图像的尺寸
    cv::Point pts[4];   //装甲矩形的四个点
    int type_armor;     //装甲类型
    cv::Mat img;		//装甲区域对应部分二值
    int id;             //装甲号码牌，0为未知，正数为对应号码
    bool is_empty;      //是否为空

    /**
     *@brief 使用两灯条来拟合装甲
     *@param pair       要拟合的灯条对
     *@param size_image 图像的大小
     */
    Armor(const LEDPair& pair, const cv::Size& size_image);

    Armor(): is_empty(true) { }

    /**
     *@brief 获取roi数字图像
     *@param img 装甲板所处背景
     *@return 得出图像是否可用
     */
    bool getROIImage(const cv::Mat& img,cv::Mat& img_num);

    /**
     *@brief 根据尺寸获取装甲板类型
     */
    void getTypeBySize();

public:
    struct ArmorComparer {
        cv::Point pt_aim;

        ArmorComparer(cv::Point point_aim): pt_aim(point_aim) { }
        bool operator()(const Armor& arm1, const Armor& arm2) const {
            return Tool::calculateDistance(arm1.pt_cen, pt_aim) <
                    Tool::calculateDistance(arm2.pt_cen, pt_aim);
        }
    };

private:
    static std::vector<cv::Point2f> points; //透视变换后的四个点
};

/**
 * @brief 上次检测的装甲的历史数据
 */
class ArmorHistory {
public:
    int type_last;      //上次的装甲类型
    bool flag;          //数据是否有效
    int id_last;        //上次得到的装甲id
    cv::Rect rect_last;	//上次的装甲板矩形位置
    int cnt_update;     //expend计数器
    int cnt_max;        //计数器最大值

    /**
     *@brief 唯一构造
     *@param data_tmp   交互数据
     */
    ArmorHistory(): type_last(-1), flag(false),
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
class ArmorDetector: public Detector {
public:
    cv::Mat img_roi;            //图像roi区域
    cv::Point pt_roi;           //roi图像偏移，即roi左上角点在源图像的位置
    ArmorHistory history;       //上次装甲历史
    ArmorClassifier classifier;	//装甲分类器
    DestoryableWindow win_roi;	//ROI对应的窗口
    bool is_useroi = true;             //是否使用roi
    bool is_classifier = true;         //是否使用分类器
    int rule_choose;                  //选择装甲的规则
    bool is_ignore2 =true;		    	//是否忽略工程

#if TYPE_ARMOR_CLASSIFIER == 0
    /**
     *@brief 初始化
     *@param img_src   要读入的相机画面
     *@param setting   读入的固定参数
     *@param data_tmp  读入的交互参数
     */
    ArmorDetector(cv::Mat& img_src, Setting& setting, Data& data_tmp):
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
    ArmorDetector(cv::Mat& img_src, Setting& setting) :
            Detector(img_src, setting),
            classifier(setting.path_classifier_arm_caffe_net,
            setting.path_classifier_arm_caffe_model) { }
#elif TYPE_ARMOR_CLASSIFIER == 2
     /**
     *@brief 初始化
     *@param img_src   要读入的相机画面
     *@param setting   读入的固定参数
     */
    ArmorDetector(cv::Mat& img_src, Setting& setting) :
            Detector(img_src, setting),
            classifier(setting.path_classifier_arm_lenet_model) { }
#endif

    /**
     *@brief  装甲识别的主要函数
     *@return 是否识别到装甲
     */
    bool run() override;


    static bool findArmor(cv::Mat& img, Parameter& param,
            float angle_err=5.5f, float intensity_avg=100.f);
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
     *	@arm_target 目标装甲
     */
    void fitArmor(Armor& arm_target, float angle_err=5.5f, float intensity_avg=100.f);

    /**
     *@brief 从目标中获取点，用于角度解算
     *@param arm_target 目标装甲
     */
    void getPoints(Armor& arm_target);

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

#endif	//ARMOR_DETECT_H_INCLUDE
