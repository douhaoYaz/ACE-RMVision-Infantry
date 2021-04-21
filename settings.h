#ifndef SETTINGS_H_INCLUDE
#define SETTINGS_H_INCLUDE

#include "utils.h"
#include <opencv2/opencv.hpp>
#include <iostream>


/**
 *@brief 敌人颜色
 *@TARGET_ +:
 *  @RED  0，红色
 *  @BLUE 1，蓝色
 */
enum TargetColor {
	TARGET_RED  = 0,
	TARGET_BLUE = 1
};

/**
 *@brief 待检测目标类型
 *@TARGET_ +:
 *  @ARMOR 0，小装甲
 *  @BUFF  1，大装甲
 *  @DART  2，风车
 */
enum TargetMode {
	TARGET_SMALL_ARMOR = 0,
	TARGET_BIG_ARMOR   = 1,
    TARGET_BUFF_ARMOR  = 2,
};

/**
 *@brief 检测模式
 *@DETECT_ +:
 *	@ARMOR 0,装甲模式
 *	@BUFF  1,能量机关模式
 *  @TOP   2,陀螺模式
 */
enum DetectMode {
    DETECT_ARMOR = 0,
    DETECT_BUFF  = 1,
    DETECT_TOP = 2
};

/**
 *@brief 能量机关类型
 *@TARGET_ +:
 *  @SMALL_RUNE  0，小符
 *  @BIG_RUNE    1，大符
 */
enum BuffType {
    SMALL_RUNE  = 0,
    BIG_RUNE = 1
};

/**
 *@brief 大风车旋转方向
 *@BUFF_ +:
 *	@CLOCKWISE	    1，顺时针
 *	@ANTICLOCKWISE	-1，逆时针
 *	@NONE           0，未知状态
 */
enum BuffDirection {
    BUFF_CLOCKWISE	    = 1,
    BUFF_ANTICLOCKWISE	= -1,
    BUFF_NONE	        = 0
};

/**
 *@brief 能量机关扇叶所在象限
 *@BUFF_ +:
 *	@BUFF_ORIGIN	0，中心原点
 *	@BUFF_QUADRANT1	1，第一象限
 *	@BUFF_QUADRANT2	2，第二象限
 *	@BUFF_QUADRANT3	3，第三象限
 *	@BUFF_QUADRANT4	4，第四象限
 */
enum BuffQuadrant {
    BUFF_ORIGIN    = 0,
    BUFF_QUADRANT1 = 1,
    BUFF_QUADRANT2 = 2,
    BUFF_QUADRANT3 = 3,
    BUFF_QUADRANT4 = 4
};


/**
 *@brief 设置类，程序初始的配置，所有线程只读
 */
class Setting {
public:
	
	cv::Point3f pt3_ptz2camera;	//相机到云台补偿(mm)
	float speed_bullet;			//子弹速度(m/s)
	float h_barrel2ptz;			//枪管与云台的相对高度(mm)
	float h_car;				//车（枪管）距地面高度(mm)

    std::string path_classifier_arm_svm;			//svm分类器路径
    std::string path_classifier_arm_caffe_net;		//caffe网络
    std::string path_classifier_arm_caffe_model;	//caffe权重文件

    std::string path_param_short;			//短焦相机相关参数路径
	int length_f_short;						//焦距(mm)
	cv::Size sz_short;						//图像尺寸(px)
	int type_driver_short;					//驱动类型(v4l2/opencv/galaxy)
	cv::Point3f pt3_offset_short;			//图像补偿点
	cv::Mat mat_camera_short;				//内参矩阵
	cv::Mat mat_coeffs_distortion_short;	//畸变系数

    bool is_short_only;             //是否只有短焦相机
    std::string path_param_long;    //长焦相机相关参数路径
	int length_f_long;
	cv::Size sz_long;
	int type_driver_long;
	cv::Point3f pt3_offset_long;
	cv::Mat mat_camera_long;
	cv::Mat mat_coeffs_distortion_long;

	bool is_buff;		//是否开启大符检测
	int h_buff;			//大风车（底部装甲）高度(mm)
	float r_buff;		//大风车扇叶半径(mm)
    float rate_buff;    //用于判断旋转方向的刷新频率
	int dist_buff;		//大风车距离(mm)

	bool is_switch_cap;		//是否开启切换长短焦相机模式
	int th_lost_target;		//丢失目标帧阈值，到达后短->长
	float rate_switch_cap;	//刷新频率
	int th_short;			//长->短距离阈值(mm)
	int th_long;			//短->长距离阈值(mm)

	bool is_prime_only;				//是否只用主相机
	std::string path_assistant;	//辅助用相机相关参数路径
	//TODO

	bool is_video;			//是否为视频模式
	std::string path_video;	//视频路径

    Setting() {
        path_classifier_arm_svm = "../parameter/other/small_armor.xml";
        path_classifier_arm_caffe_net = "../parameter/other/lenet_predict.protext";
        path_classifier_arm_caffe_model = "../parameter/other/lenet.caffemodel";
	}

	/**
	 *@breif 在读取xml之后，临时调参写在这，可以不用改xml文件
	 **/
	void set() {

		/**
		 *@比赛大风车参数：
		 *@大能量机关最底部装甲板(中心)与桥面高度 暂未知
		 *@能量机关到桥头水平距离	7100mm
		 *@大风车半径				100cm
		 *@大风车中心离地高度		2283mm
		 */

		/**
		 *@ACE大风车参数：
		 *@大能量机关最底部装甲板(中心)与桥面高度 35cm
		 *@能量机关到桥头水平距离	4米多
		 *@大风车半径				70cm
		 *@大风车中心离地高度		2283mm
		 */
		/**
		 *@步兵车参数：
		 *@步兵枪口距离桥面高度		41cm
		 *@大风车半径				70cm
		 *@相机到云台水平距离		17cm
		 */

		dist_buff = 5200;
		h_buff = 350;
		h_car = 410;
		r_buff = 700;

        speed_bullet = 17;
        path_classifier_arm_svm = "../parameter/other/small_armor.xml";
        path_classifier_arm_caffe_net = "../parameter/other/lenet_predict.protext";
        path_classifier_arm_caffe_model = "../parameter/other/lenet.caffemodel";

	}

	/**
	 *@brief 将参数保存为xml文件
	 */
	void save();

    void setInfantryParameter(const std::string& path_prime);

    void setInfantryParameter(const std::string& path_short, const std::string& path_long, bool flag_switch_cap=false);

    void setGuardParameter(const std::string& path_prime);

    void setGuardParameter(const std::string& path_prime, const std::string& path_assistant);

    void setVideoParameter(const std::string& path_video);


};

/**
 *@brief 程序各部分的中间交互数据，控制线程只读，其他线程可读可写
 */
struct Data {  //为什么这里使用struct
public:
    int type_target=0;
    int cnt_lost=0;
    bool is_last_long=false;			//上一次是否为长焦
    int mode_cap=0;						//当前相机模式
	std::vector<cv::Point2f> pts_2d;	//检测出的目标点

    float angle_buff=0;					//能量机关装甲板角度
    float angle_pre=0;	                //能量机关预测角，根据旋转方向和暴力推算得出
    int quadrant_buff=0;			    //能量机关象限位置

    Timer buff_timer;
    float angle_buff_last=0;      //上一次的能量机关装甲板角度
    int sample_count=0;             //大符起始时间求取的采样数目标志位
    float orig_t=0;               //打击大符程序启动执行初始时间
    float real_spd=0;             //大符的实时转速
    //拟合斜率的vector
    std::vector<int> temp_x;
    std::vector<float> temp_y;

    float dist=0;                       //自瞄角度解算出的距离
    cv::Point2f pt_final_world_offset;  //最终暴力推算得到的世界坐标补偿,即弹丸运动的延时补偿
    cv::Point pt_offset_world_armor = cv::Point(0, 0);
    cv::Point pt_offset_world_buff = cv::Point(650, 500);     //能量机关初始世界坐标补偿
                                        //x轴(yaw轴)方向补偿值,y轴(pitch轴)由计算得出
    cv::Mat img_show;
    bool is_ban_assistant=false;

    bool flag_update_param=false;

    static Data& getData() { //获取当前getData
        static Data data;
        return data;
    }

private:
    Data() { }
};

/**
 *@brief 参数类，由控制线程输入，其他线程只读，控制线程可读可写
 */
class Parameter {
public:

    struct ArmorParameter {
		int th_color;				//自瞄检测算法的颜色阈值
		int th_gray;                //自瞄检测算法的灰度阈值
        cv::Point pt_offset_cap;    //自瞄短焦相机成像偏移坐标补偿，(100, 100)为图像中心，等尺度（即装甲板的位置补偿）
    }armor;

    struct BuffParameter {
        int th_color;                  //能量机关检测算法的颜色阈值
        int th_gray;                   //能量机关检测算法的灰度阈值
        cv::Point pt_offset_cap;       //能量机关长焦相机成像偏移坐标补偿
		cv::Point pt_offset_follow_delay;//云台识别跟随延时的补偿(由旋转方向最终确定补偿的极性)
        int x_pt_offset_world;         //打小符云台枪管弹丸执行延时的世界坐标补偿的x值
    }buff;

    struct AngleParameter {
		int gravity_offset = 1;     //云台pitch轴重力补偿角度
    }angle;

    struct OtherParameter {
        double factor_x;
        double factor_y;
        double factor_z;
		int color_enemy;	//敌方颜色，从串口接收或手动设置调试
        int mode_detect;	//目标类型的模式，从串口接收或手动设置调试
        int type_buff;      //能量机关的大小符类型
        int direction;		//能量机关旋转方向，1顺时针，-1逆时针
		bool flag_save;		//保存视频标志
    }other;

    static Parameter& getParameter() {
        static Parameter param;
        return param;
    }

    static Parameter& getTempParameter() {
        static Parameter param_tmp;
        return param_tmp;
    }

    static void update() {
        getParameter() = getTempParameter();
    }

private:
    /**
     *@brief 动态参数的默认设定
     */
    Parameter() {

//        size_t offset = reinterpret_cast<size_t>(&((ArmorParameter*)0)->th_color);
//        Reflector<ArmorParameter>::registerMember("th_color", Reflector<ArmorParameter>::INT, offset);
        registerReflector(ArmorParameter, th_color, Reflector<ArmorParameter>::INT);
        registerReflector(ArmorParameter, th_gray, Reflector<ArmorParameter>::INT);
        registerReflector(ArmorParameter, pt_offset_cap, Reflector<ArmorParameter>::OTHER);

        registerReflector(BuffParameter, th_color, Reflector<ArmorParameter>::INT);
        registerReflector(BuffParameter, th_gray, Reflector<ArmorParameter>::INT);
        registerReflector(BuffParameter, pt_offset_cap, Reflector<ArmorParameter>::OTHER);
        registerReflector(BuffParameter, pt_offset_follow_delay, Reflector<ArmorParameter>::OTHER);
        registerReflector(BuffParameter, x_pt_offset_world, Reflector<ArmorParameter>::INT);

        registerReflector(AngleParameter, gravity_offset, Reflector<ArmorParameter>::INT);

        registerReflector(OtherParameter, factor_x, Reflector<ArmorParameter>::DOUBLE);
        registerReflector(OtherParameter, factor_x, Reflector<ArmorParameter>::DOUBLE);
        registerReflector(OtherParameter, factor_z, Reflector<ArmorParameter>::DOUBLE);

        other.factor_x = 100;
        other.factor_y = 100;
        other.factor_z = 100;
        other.color_enemy = TARGET_RED;
        other.mode_detect = DETECT_ARMOR;
        other.type_buff = SMALL_RUNE;
        other.direction = BUFF_NONE;
        other.flag_save = false;

        armor.th_color = 16;
        armor.th_gray = 25;
        buff.th_color = 15;
        buff.th_gray = 78;

        //4mm 的装甲板补偿
        armor.pt_offset_cap = cv::Point(75, 120);
        //8mm 的装甲板补偿
        //armor.pt_offset_cap = cv::Point(0, 0);


        buff.pt_offset_cap = cv::Point(100, 100);//45,133

        buff.pt_offset_follow_delay = cv::Point2f(0, 0);//-20

    }


};

#endif	//SETTINGS_H_INCLUDE
