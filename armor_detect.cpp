#include "armor_detect.h"
#include <iterator>
/*
 *cols 列数（x）
 *rows 行数（y）
 */


//LEDPair灯条对类构造方法 （采用两个灯条类对象进行构造）
LEDPair::LEDPair(const LEDStick& l1, const LEDStick& l2) {
    //LEDStick led_sticks  灯条对，顺序为左、右
    //rrect 灯条 的旋转矩形
    //灯条匹配 通过灯条类的rrect矩形的左上角点的x值分为左灯条和右灯条
    if (l1.rrect.center.x < l2.rrect.center.x) //l1.rrect.center  旋转矩形的质点 Point2f
                                               //l2.rrect.center.x 旋转矩形的质点的x值
    {
        led_sticks[0] = l1;
        led_sticks[1] = l2;
    }
    else
    {
        led_sticks[0] = l2;
        led_sticks[1] = l1;
    }
    //rect 灯条对 构成的矩形区域
    //fabs( )主要是求精度要求更高的double ，float 型的绝对值，在<cmath>头文件里。
    //angle_err灯条对角度之差 rrect.angle 旋转角度，当角度为0、90、180、270等时，矩形就成了一个直立的矩形
    angle_err = fabs(l1.rrect.angle - l2.rrect.angle);
    //rate_len灯条对长度之比  l1.length 灯条长度
    rate_len = fabs(l1.length / l2.length);
    //rect 灯条对构成的矩形区域  rect.width 求矩形区域的长度
    rect.width = abs(static_cast<int>(l1.rrect.center.x - l2.rrect.center.x));
    //rect 灯条对构成的矩形区域  rect.height 求矩形区域的宽度  Size2f size矩形的边长
    rect.height = static_cast<int>((l1.rrect.size.height + l1.rrect.size.height) / 2); //把两个不等长的旋转矩形的高 转换成等高的Rect矩形区域
    //cv::Point2i pt_cen灯条对构成矩形的中心
    pt_cen.x = static_cast<int>((l1.rrect.center.x + l2.rrect.center.x) / 2);
    pt_cen.y = static_cast<int>((l1.rrect.center.y + l2.rrect.center.y) / 2);
    //把灯条本身的宽度也考虑进去了,把灯条本身的宽度也考虑进去了，所以这是暴力拟合
    // 整块装甲板，#表示灯条区域 11表示灯条对构成的矩形区域 , 现在拟合的是111的部分
    //       ##11111111##
    //       ##11111111##
    //       ##11111111##
    //       ##11111111##
    //  灯条区域(矩形区域)灯条区域
    rect.x = pt_cen.x - rect.width / 3;
    rect.y = pt_cen.y - rect.height / 3;
    rect.width = static_cast<int>(rect.width * 2.0/3);
    rect.height = static_cast<int>(rect.height * 2.0/3);
}

int LEDPair::getAverageIntensity(cv::Mat& img) const //计算并获取平均强度
{
    //判断边界条件
    if (rect.width < 1 || rect.height < 1 || rect.x < 0 || rect.y < 0)
        return 255;
    //计算图像ROI区域强度
    return Tool::calculateRoiAverageIntensity(img, rect);
}

bool LEDPair::isSuitableSize() const //判断灯条对是否为合适的尺寸
{
    //1、左灯条的高度高于右灯条 2、右灯条的高度高于左灯条 return false
    if(led_sticks[0].rrect.size.height*0.7f > led_sticks[1].rrect.size.height
        || led_sticks[0].rrect.size.height*1.3f < led_sticks[1].rrect.size.height)
        return false;
    //灯条对的平均高度 float
    float h_avg = (led_sticks[0].rrect.size.height + led_sticks[1].rrect.size.height)/2.0f;
    //灯条倾斜程度判断  Q：Q：阈值是否为最优解？
    if(fabs(led_sticks[0].rrect.center.y - led_sticks[1].rrect.center.y) < 0.8f* h_avg
        && h_avg*2.7f > rect.width && h_avg < 2.0f* rect.width)
        return true;
    //不然最后也会返回false 不然最后也会返回false灯条对不匹配
    return false;
}

//透视变换后的四个点
std::vector<cv::Point2f> Armor::points = {
	cv::Point(0, 64), cv::Point(64, 64),
	cv::Point(64, 0), cv::Point(0, 0)
};

/**LEDPair
 * cv::Rect rect;					//灯条对构成的矩形区域
LEDStick led_sticks[2];         //灯条对，顺序为左、右
float rate_len; 				//长度之比
float angle_err;                //角度之差
cv::Point2i pt_cen;				//灯条对构成矩形的中心
int intensity_avg;				//矩形区域内的平均强度，用来衡量是否为误判
bool is_empty;                  //是否为空
*/
/**
 *@brief 使用两灯条来拟合装甲
 *@param pair       要拟合的灯条对
 *@param size_image 图像的大小
 */
Armor::Armor(const LEDPair& pair, const cv::Size& size_image):
        LEDPair(pair), size_img(size_image), is_empty(false) {
    //区域宽
    rect.width = static_cast<int>(led_sticks[1].rrect.center.x -
            led_sticks[0].rrect.center.x);
//????????????? Question1
    double height_led = static_cast<double>((led_sticks[0].rrect.size.height +
            led_sticks[0].rrect.size.height) / 2);
    //cv::Point2i pt_cen; 灯条对构成矩形的中心
    pt_cen.x = static_cast<int>((led_sticks[0].rrect.center.x +
            led_sticks[1].rrect.center.x) / 2);
    pt_cen.y = static_cast<int>((led_sticks[0].rrect.center.y +
            led_sticks[1].rrect.center.y) / 2);

    //灯条高大约6cm，装甲高大约12cm，小装甲宽大约12cm，大装甲宽大约24cm
    //拟合区域 宽(长）度 x 高度  12x6
    //求矩形Rect的左上角点
    rect.height = static_cast<int>(height_led * 2);
    rect.x = pt_cen.x - rect.width / 2;
    rect.y = pt_cen.y - rect.height / 2;
    if (rect.y < 0)
    {
        rect.y = 0;
    }

    //如果装甲的高度+矩形区域的左上角y超过图像的高度
    if (rect.height + rect.y > size_img.height)  //int x; /* 方形的最左角的x-坐标 */ //int y; /* 方形的最上或者最下角的y-坐标 */
    {
        rect.height = size_img.height - rect.y - 1; //不断缩小矩形区域
    }

    //装甲矩形的四个点(透视变换前的四个点) 赋值
    pts[0] = cv::Point(rect.x, rect.y + rect.height);
    pts[1] = cv::Point(rect.x + rect.width, rect.y + rect.height);
    pts[2] = cv::Point(rect.x + rect.width, rect.y);
    pts[3] = cv::Point(rect.x, rect.y);
}

/**
 *@brief 获取roi图像
 *@param img 装甲板所处背景
 *@return 得出图像是否可用
 */
bool Armor::getROIImage(const cv::Mat& img_src) {

    //装甲矩形的四个点，放入透视变换
    std::vector<cv::Point2f> points(pts, pts + 4);
    //透视变换后的图像 64x64
    cv::Mat img_gray = cv::Mat::zeros(64, 64, CV_8UC3);
    //求取透视变换矩阵3x3
    cv::Mat mat_trans = cv::getPerspectiveTransform(points, Armor::points);
    //进行透视变换
    cv::warpPerspective(img_src, img_gray, mat_trans, img_gray.size());
    //将透视变换后的图像的RGB图像转换成灰度图像
	cv::cvtColor(img_gray, img_gray, cv::COLOR_BGR2GRAY);
    //在透视变化后的图像上绘制矩形框  64x64
    //填充两侧灯条，防止影响
	cv::rectangle(img_gray, cv::Point(0, 0), cv::Point(9, 63), BGR::all(0), -1);
	cv::rectangle(img_gray, cv::Point(54, 0), cv::Point(63, 63), BGR::all(0), -1);
    //直方图均匀(提高图像的质量，该算法对图像的亮度进行了归一化处理，提高了图像的对比度)
    cv::equalizeHist(img_gray, img_gray);


    cv::Mat img_bin;        //二值图像
    double th;              //大津法（OTSU）得出来的阈值
    int area_light;         //亮点面积
    int area_all;           //总面积
    double ratio;           //亮点面积比例
    //Threshold函数典型的应用是对灰度图像进行阈值操作得到二值图像。
    //大津法（OTSU）。 通过函数cv2.threshold会自动找到一个介于两波峰之间的阈值
    //最大类间方差法（OTSU）是找到自适应阈值的常用方法。原理参考了冈萨雷斯的《数字图像处理》。
    th = cv::threshold(img_gray, img_bin, 5, 255, cv::THRESH_OTSU);//关键字THRESH_OTSU

    //countNonZero()：返回灰度值不为0的像素数，可用来判断图像是否全黑。
    area_light = cv::countNonZero(img_bin); //亮点面积(不为0的像素点数目)
    area_all = 64*64;                       //总面积(64x64个像素点)
    ratio = 1.0 * area_light / area_all;    //亮点面积比例

    static int th_const = 100;              //固定阈值

	if ((ratio < 0.1 && th > th_const) || (ratio > 0.5 && th < th_const)) {
		//大津无效，改为固定阈值
        /* THRESH—BINARY
           如果像素值大于阈值(100)，像素值就会被设为255
           小于等于阈值，设定为0
        */
		cv::threshold(img_gray, img_bin, th_const, 255, cv::THRESH_BINARY);
        //
		area_light = cv::countNonZero(img_bin);
		ratio = 1.0 * area_light / area_all;
        //
        if (ratio < 0.1){
            th_const = th_const - 1 == 0 ? 1 : th_const - 1; //降低阈值，降低标准
        }
        else if (ratio > 0.5){
            th_const = th_const + 1 == 255 ? 254 : th_const + 1;//提高阈值，提高标准
        }
		else {
			img = img_bin;
			return true;
		}
	}
    else if (ratio > 0.1 && ratio < 0.5) { //阈值符合标准，可以输出二值化图像
		img = img_bin;
		return true;
	}
    //不符合标准，对透视变换后的图像二值化失败
	img = cv::Mat();
	return false;
}



/**
 *@brief 根据尺寸获取装甲板类型（大装甲、小装甲）
 */

void Armor::getTypeBySize() {
    //灯条对形成的矩形区域的高度
    double height_led = static_cast<double>((led_sticks[0].rrect.size.height +led_sticks[0].rrect.size.height) / 2);
    //矩形区域的高度与比值
    const double ratio_wh = rect.width / height_led;
    if (ratio_wh > 3. && ratio_wh < 5.)
        type_armor = TARGET_BIG_ARMOR;//大装甲板
    else if (ratio_wh > 1.2 && ratio_wh <= 3.)
        type_armor = TARGET_SMALL_ARMOR;//小装甲板
}





#if TYPE_ARMOR_CLASSIFIER == 0

void ArmorClassifier::getResult(const cv::Mat& img, int& id, int& type_sz) {

	std::vector<float> v_descriptors;			//存放结果
	hog->compute(img_num, v_descriptors, cv::Size(1, 1), cv::Size(0, 0));    //Hog特征计算

	cv::Mat v_predict = cv::Mat(1, v_descriptors.size(), CV_32FC1);	//预测向量
	int i = 0;	//对应类别索引
	for (auto iter = v_descriptors.begin(); iter != v_descriptors.end(); ++iter) {
		v_predict.at<float>(0, i) = *iter;//第i个样本的特征向量中的第n个元素
		i++;
	}
	v_predict.convertTo(v_predict, CV_32FC1);
	this->id = (unsigned int)classifier_arm.classifier->predict(v_predict);

    return;
}

#elif TYPE_ARMOR_CLASSIFIER == 1



/**
 *@brief 装甲板id和size分类器
 *@有SVM（太原理工开源）和cv::dnn+caffe两种方法
 */
/**
 *@brief 分类
 *@输入：
 *	@param img     待分类图像
 *@输出：
 *	@param type_sz 装甲大小类型
 *	@param id      得到的id
 */
void ArmorClassifier::getResult(const cv::Mat& img, int& type_sz, int& id) {
    /*
     * 在进行深度学习或者图片分类时，blobFromImage主要是用来对图片进行预处理。包含两个主要过程：
     1、整体像素值减去平均值（mean）
     2、通过缩放系数（scalefactor）对图片像素值进行缩放
   */
    /* blobFromImage(InputArray image, double scalefactor=1.0, const Size& size = Size(),const Scalar& mean = Scalar(),bool swapRB = false, bool crop = false,int ddepth = CV_32F)*/
    cv::Mat input_blob = cv::dnn::blobFromImage(img, 1, cv::Size(64, 64));

    //设置网络输入
    net.setInput(input_blob, "data");


    //前向运行计算输出层，输出层名称为outputName。 Mat forward(const String& outputName = String());
    cv::Mat pred = net.forward("prob");  //"prob"测试窗口名字 ,forward 得到的是n*1的矩阵
    //std::cout << pred << std::endl; 检测是否是n*1的矩阵
    //  cvMinMaxLoc()找出图片或一组数据中最大值及最小值的数据,以及最大值及最小值的位置
	cv::Point pt;
	cv::minMaxLoc(NULL, NULL, NULL, NULL, &pt);
    int res = pt.x; //位置的x是类别，最大值的x就是最有可能的类别
    type_sz = res < 8 ? TARGET_SMALL_ARMOR : TARGET_BIG_ARMOR;  // 0-7 小装甲板 8-15大装甲板 总共16个类别
    id = res % 8 + 1;//求id


    draw(img, type_sz, id);//draw函数还有待修改

}

#endif



/**
 *@brief  装甲识别的主要函数
 *@return 是否识别到装甲
 */
//bool run() override; //override,c++11特性 必须重写该基类的虚函数，否则编译会出错
bool ArmorDetector::run() {

    /*ArmorDetector  公有继承Detector 并且Detector（父类）里面有个成员变量  cv::Mat& img; //输入的原图
    */
    Data& data = Data::getData(); // Data 程序各部分的中间交互数据，控制线程只读，其他线程可读可写

    data.img_show = img.clone();  // cv::Mat& img; //输入的原图


    setROI(); //roi区域矩形

    Armor arm_target;    //目标装甲
    fitArmor(arm_target);  //对得到的灯条进行匹配

    if (arm_target.is_empty)//没识别到装甲
        return false;

    getPoints(arm_target);//从目标中获取点，用于角度解算
    return true;//识别到装甲

}






//应用场景:哨兵模式辅助相机使用
//找装甲
bool ArmorDetector::findArmor(cv::Mat& img_src, Parameter& parameter, float angle_err, float intensity_avg) {

    Data& data = Data::getData();

    cv::Mat img_color;	//颜色通道相减图像
    cv::Mat img_gray;   //灰度图像

    std::vector<cv::Mat> channels;	//bgr通道
    cv::split(img_src, channels);   //输入的原图进行通道分离

    if (parameter.other.color_enemy == TARGET_RED)//如果敌方的颜色为红色
        subtract(channels[2], channels[0], img_color);
    else                                          //如果敌方的颜色为蓝色
        subtract(channels[0], channels[2], img_color);

    cv::threshold(img_gray, img_gray, parameter.armor.th_gray, 255, cv::THRESH_BINARY);    //parameter.armor.th_gray自瞄检测算法的灰度阈值
    cv::threshold(img_color, img_color, parameter.armor.th_color, 255, cv::THRESH_BINARY); //parameter.armor.th_color自瞄检测算法的颜色阈值

    std::vector<std::vector<cv::Point>> contours_color;	//颜色通道相减图像（外部）轮廓
    std::vector<std::vector<cv::Point>> contours_gray;	//灰度图像（外部）轮廓
    cv::findContours(img_color, contours_color, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    cv::findContours(img_gray, contours_gray, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    SticksSet led_sticks;//typedef std::vector<LEDStick> SticksSet;    //灯条集

    size_t sz_gray = contours_gray.size();		//灰度轮廓数量
    for (size_t i = 0; i < sz_gray; i++) {
        double area = cv::contourArea(contours_gray[i]);	//灰度单条轮廓面积
        if (area < 20.0 || 1e5 < area)
            continue;//排除面积不合理
        for (size_t j = 0; j < contours_color.size(); j++) {
            if (cv::pointPolygonTest(contours_color[j], contours_gray[i][0], false) >= 0.0) {
                /*
                 * double pointPolygonTest(InputArray contour, Point2f pt, bool measureDist)
                 * 判断一个点是否在一个contour的内部还是外部。
                 * contour – 输入findContour提取到的边缘.
                 * pt – 需要检测的点.
                 * measureDist – 为真，则计算检测点到边缘的距离，为负值在外部，0在边上，正值在内部。为假，则返回-1（在contour外部）、0（在contour上）、1（在contour内部。
                 */

                /*
                 * double arcLength(InputArray curve, bool closed)
                 * InputArray curve：表示图像的轮廓
                 * bool closed：表示轮廓是否封闭的
                 */
                double length = cv::arcLength(cv::Mat(contours_gray[i]), true);	//灰度轮廓周长
                if (length < 30 || length > 400)//排除周长不合理
                    continue;
                cv::RotatedRect rrect = cv::fitEllipse(cv::Mat(contours_gray[i]));	//拟合的旋转矩阵
                if (rrect.angle > 90.0f)	//标准化
                    rrect.angle = rrect.angle - 180.0f;

                if (abs(rrect.angle) >= 30)		//筛选倾斜过度的灯条
                    continue;
                led_sticks.emplace_back(rrect);//不断在容器尾插入拟合的旋转矩阵


                cv::RotatedRect rrect_draw = rrect;
                Tool::drawRotatedRectangle(img_src, rrect_draw, BGR(255, 0, 255), 1);


                break;
            }
        }
    }

    cv::resize(img_color, img_color, cv::Size(img_color.cols/2, img_color.rows/2));
    cv::imshow("color", img_color);

    size_t sz_leds = led_sticks.size();	//灯条数量
    for (size_t i = 0; i < sz_leds; i++)
        for (size_t j = i + 1; j < sz_leds; j++) {
            LEDPair pair_led(led_sticks.at(i), led_sticks.at(j));	//临时灯条对
            if (pair_led.angle_err < angle_err
                    && pair_led.rate_len < 1.5f
                    && pair_led.isSuitableSize() == true
                    && pair_led.getAverageIntensity(img_gray) < intensity_avg)
                return true;
        }
    return false;
}



/**
 *@brief 根据上次装甲设置ROI
 * 
 */
//扩大ROI区域进行检测
void ArmorDetector::setROI() {

    Data& data = Data::getData();

    cv::Rect2f rect_roi;  //roi矩形框
    float ratio_w = 2.5;  //宽度比率
    float ratio_h = 1.5;  //高度比率

    if (is_useroi && history.flag && (is_ignore2 == false ||
            ((history.id_last == 2 && history.id_last != 2)))) { //(使用roi && 上次装甲的历史history的flag 数据有效 &&（不忽略工程 || （（history上次得到的装甲id==2(步兵）&& history上次得到的装甲id不等于2))))

        //数学解算部分
        rect_roi = history.rect_last; //cv::Rect rect_last;	//上次的装甲板矩形位置
        rect_roi.x -= rect_roi.width * ((ratio_w-1)/2);     //往左边预测 拉开 x-（x*0.75)
        rect_roi.width = static_cast<int>(rect_roi.width * ratio_w); //扩大ROI区域的宽度
        rect_roi.y -= rect_roi.height * ((ratio_h-1)/2);    //往上面拉开 y-（y*0.25）
        rect_roi.height *= ratio_h;                                  //扩大ROI区域的高度

        if (rect_roi.x < 0)
            rect_roi.x = 0;//重置
        if (rect_roi.y < 0)
            rect_roi.y = 0;//重置
        if (rect_roi.x + rect_roi.width >= img.cols)
            rect_roi.width = img.cols - rect_roi.x - 1;//超过的话就减一
        if (rect_roi.y + rect_roi.height >= img.rows)
            rect_roi.height = img.rows - rect_roi.y - 1;//超过的话就减一
        if (rect_roi.width <= 0 || rect_roi.height <= 0)
            rect_roi = cv::Rect(0, 0, img.cols, img.rows);//如果roi区域的宽度和高度无效，则设置整张输入的原图为roi区域
    }
    else
        rect_roi = cv::Rect(0, 0, img.cols, img.rows); //整张图为ROI
    img_roi = img(rect_roi);//输入的原图img取rect_roi矩阵的图像区域 变成 img_roi（roi区域的Mat类）


    if (is_useroi)
        cv::rectangle(data.img_show, rect_roi, BGR::all(255), 3); //用白色框框  框住ROI区域 //cv::rectangle(img_show, rect_roi, BGR::all(255), 3);

    // cv::Point pt_roi;  roi图像偏移，即roi左上角点在源图像的位置
    pt_roi = cv::Point2f(rect_roi.x, rect_roi.y);


    if (rect_roi.width != img.cols &&
            rect_roi.height != img.rows) { //当rect_roi的宽度和高度没有覆盖满 img（输入的原图），就将img_roi区域的图像输出出来
        /* showImage
         *@brief 显示图片，若flag为false会自动创建窗口
         *@param img 要显示的图片
         */
        win_roi.showImage(img_roi);// DestoryableWindow win_roi  ROI对应的窗口类
    }
    else
        win_roi.destory();         //消掉窗口



}


/*
 *@beief 找出图像中的灯条，并进行配对
 *@输入&输出：
 *  @img_gray   传入的灰度图，作为灯条判断的依据之一，输出时被二值化
 *	@led_sticks 灯条集，已配对
 */
void ArmorDetector::findLamps(cv::Mat& img_gray, SticksSet& led_sticks) {

    Data& data = Data::getData();//图像程序各部分的中间交互数据
    Parameter& param = Parameter::getParameter();//图像参数数据
    led_sticks.clear(); //vector容器自动清除容器内所有的数据
    cv::Mat img_color;	//颜色通道相减图像

    std::vector<cv::Mat> channels;	//bgr通道
    cv::split(img_roi, channels);   //分离通道

    //颜色通道相减（提取相应通道的图像）
    if (param.other.color_enemy == TARGET_RED) //若敌方是红色，则提取R通道里面的图像
        subtract(channels[2], channels[0], img_color);
    else                                       //否则提取B通道里的图像
        subtract(channels[0], channels[2], img_color);

    /*二值化
    1、th_gray自瞄检测算法的灰度阈值
    2、th_color自瞄检测算法的颜色阈值
    */
    cv::threshold(img_gray, img_gray, param.armor.th_gray, 255, cv::THRESH_BINARY);
    cv::threshold(img_color, img_color, param.armor.th_color, 255, cv::THRESH_BINARY);

    //寻找并绘制轮廓
    std::vector<std::vector<cv::Point>> contours_color;	//颜色通道相减图像（外部）轮廓
    std::vector<std::vector<cv::Point>> contours_gray;	//灰度图像（外部）轮廓
    cv::findContours(img_color, contours_color, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    cv::findContours(img_gray, contours_gray, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

//    //debug
//    cv::Mat temp = img;
//    cv::resize(temp, temp, cv::Size(img_color.cols/2, img_color.rows/2));
//    cv::drawContours(temp,contours_color,-1,(0,0,255),10);
//    imshow("test",temp);
    //
    size_t sz_gray = contours_gray.size();		//灰度轮廓数量
    for (size_t i = 0; i < sz_gray; i++) {
        double area = cv::contourArea(contours_gray[i]);	//灰度单条轮廓面积 cvContourArea 计算轮廓面积
        if (area < 20.0 || 1e5 < area)
            continue;//跳过
        for (size_t j = 0; j < contours_color.size(); j++) {
            if (cv::pointPolygonTest(contours_color[j], contours_gray[i][0], false) >= 0.0) { //pointPolygonTest 判断一个点是否在一个contour的内部还是外部。
                /* double pointPolygonTest(InputArray contour, Point2f pt, bool measureDist)
                 * contour – 输入findContour提取到的边缘.
                   pt – 需要检测的点.
                   measureDist – 为真，则计算检测点到边缘的距离，为负值在外部，0在边上，正值在内部。为假，则返回-1（在contour外部）、0（在contour上）*/

                //
                double length = cv::arcLength(cv::Mat(contours_gray[i]), true);	//灰度轮廓周长(长度)
                if (length < 30 || length > 400)//排除周长不合理
                    continue;
                cv::RotatedRect rrect = cv::fitEllipse(cv::Mat(contours_gray[i]));	//拟合的旋转矩阵
                if (rrect.angle > 90.0f)	//标准化
                    rrect.angle = rrect.angle - 180.0f;

                if (abs(rrect.angle) >= 30)		//筛选倾斜过度的灯条
                    continue;
                led_sticks.emplace_back(rrect); //在容器尾部添加一个元素，这个元素原地构造，不需要触发拷贝构造和转移构造。而且调用形式更加简洁，直接根据参数初始化临时对象的成员。


                cv::RotatedRect rrect_draw = rrect;

                /*
                 *@brief 得到roi点在原图的位置
                 *@param pt roi上要转换的点
                 *@return 原图上的点
                 * 因为rrect是roi区域上的点，坐标以roi区域左上角为原点，所以要进行转换（具体：加上这个的cv::Point pt_roi roi区域左上角的坐标位置）
                 */
                rrect_draw.center = convertSourcePoint(rrect_draw.center);

                //用线段line画出旋转矩形的样子
                Tool::drawRotatedRectangle(data.img_show, rrect_draw, BGR(255, 0, 255), 1);


                break;
            }
        }
    }
    cv::resize(img_color, img_color, cv::Size(img_color.cols/2, img_color.rows/2));
    cv::imshow("color", img_color);

}




/*
 *@brief 根据灯条来拟合装甲(全匹配)
 *输入：
 *  @intensity_avg 装甲区域内的最小平均亮度
 *  @angle_err     两灯条的最大误差角度
 *输出：
 *	@arm_target 目标装甲
 */
void ArmorDetector::fitArmor(Armor& arm_target, float angle_err, float intensity_avg) {
    Parameter& param = Parameter::getParameter(); //取参数
    Data& data = Data::getData();                 //（程序各部分的中间交互数据）取中间数据

    cv::Mat img_gray;	//灰度图像
    cv::cvtColor(img_roi, img_gray, cv::COLOR_BGR2GRAY);//将img_roi区域转换为灰度图像

    SticksSet led_sticks;//typedef std::vector<LEDStick> SticksSet;    //灯条集
    findLamps(img_gray, led_sticks);// 找出图像中的灯条，并进行配对


    /*cv::Mat& img 输入的原图
     *param.armor.pt_offset_cap.x 自瞄短焦相机成像偏移坐标补偿，(100, 100)为图像中心，等尺度（即装甲板的位置补偿）
     *补偿前 图像中心的坐标为（100，100）\ 补偿后的中心为经验值补偿中心(即：调出来的，一般是让红外对准装甲板中心) 无实际物理意义
     *cv::Point pt_roi; roi图像偏移，即roi左上角点在源图像的位置
     */
    cv::Point pt_aim = cv::Point(img.cols/2 + param.armor.pt_offset_cap.x - 100, \
            img.rows/2 + 100 - param.armor.pt_offset_cap.y) - pt_roi;    //补偿后的中心（经验补偿）


    typedef std::set<Armor, Armor::ArmorComparer> ArmorSet;//set<Elem,op> 使用op作为排序准则，以Elem作为元素类型的set
    Armor::ArmorComparer comparer_arm(pt_aim);//初始化一个实例
    ArmorSet arms(comparer_arm); // set c(rv) Move构造函数，rv是一个set右值引用，那么这里的构造函数是一个Move构造函数，建立一个新的set，取右值内容（C++11新特性）

    size_t sz_leds = led_sticks.size();	//灯条数量
    ArmorSet::iterator iter;//迭代器是一种检查容器内元素并遍历元素的数据类型。迭代器提供对一个容器中的对象的访问方法，并且定义了容器中对象的范围。
    for (size_t i = 0; i < sz_leds; i++)
        for (size_t j = i + 1; j < sz_leds; j++) {
            LEDPair pair_led(led_sticks.at(i), led_sticks.at(j));	//临时灯条对
            if (pair_led.angle_err > angle_err                      //临时灯条对的角度之差  大于  设定的好的角度之差阈值
                    || pair_led.rate_len > 1.5f                     //临时灯条对的长度之比  大于  设定的好的长度之比阈值
                    || pair_led.isSuitableSize() == false           //如果临时灯条对不为合适的尺寸
                    || pair_led.getAverageIntensity(img_gray) > intensity_avg)  //img_gray的图像区域强度  大于  装甲区域内的最小平均亮度
                continue;//跳过下一组灯条对

            /*@brief 使用两灯条来拟合装甲
             *@param pair       要拟合的灯条对
             *@param size_image 图像的大小
             */
            Armor arm(pair_led, img.size()); //临时装甲板

            //1、使用分类器获取装甲板类型 2、使用尺寸获取装甲板类型
            if (is_classifier) {//
                classifier.getResult(img, arm.type_armor, arm.id);//取装甲识别的数值


                cv::putText(data.img_show, std::to_string(arm.id), convertSourcePoint(arm.pt_cen),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, BGR::all(255), 2);//putText 在图像上绘制文字


                if (rule_choose == ARMOR_LAST_ID_FIRST && arm.id == history.id_last
                        && (is_ignore2 || arm.id != 2)) { //(选择装甲的规则(优先选择上次检测到的ID，再根据距离选择) && 临时装甲板的id和历史上一次的id相同 && ( 忽略工程 ||装甲的id不是2(2为工程)))
                    /*
                     *@brief 根据装甲更新
                     *@param arm    要更新的装甲数据
                     *@param pt_roi roi图像偏移，即roi左上角点在源图像位置
                     */
                    history.update(arm, pt_roi);// update 函数
                    //
                    arm_target = arm; // arm_target目标装甲板 = arm临时装甲板的值
                    //
                    goto END_FIT_ARMOR; //goto ++最强大的关键字之一，它可以使程序跳转到指定位置继续运行。
                }
            }
            else
                arm.getTypeBySize();//根据尺寸获取装甲板类型（大装甲、小装甲）

            arms.emplace(std::move(arm));//从实现上讲，std::move基本等同于一个类型转换：static_cast<T&&>(lvalue);

        }

    for (iter=arms.begin(); iter!=arms.end(); ++iter) {
        if (iter->id == 2 && is_ignore2 && arms.size()!=1) // 如果iter的id为2（工程） && 忽略了工程 && 容器arms的长度不为1
            continue;
        arm_target = *iter;// 目标装甲 = iter的arm
        history.update(arm_target, pt_roi);// 更新装甲信息
        break;
    }
    if (iter == arms.end())
        history.update(img.size());// 若没找到要更新的装甲信息


END_FIT_ARMOR:
    ;

    Tool::drawCross(data.img_show, convertSourcePoint(pt_aim), 5, BGR::all(255), 1);
    //5、在点处绘制十字标记（上面）
    /* static void drawCross(cv::Mat& img, cv::Point pt, int len_cross, const BGR color,int thickness=1, int type_line=cv::LINE_8, int shift=0)
     *@brief 在点处绘制十字标记
     *@param img       要绘制的原图
     *@param pt        十字原点
     *@param len_cross 十字一侧的线长
     *@param ...       其余与cv::line基本一致
     */


//#ifdef IMAGE_SHOW
    cv::resize(img_gray, img_gray, cv::Size(img_gray.cols/2, img_gray.rows/2));
    cv::imshow("gray", img_gray);
//#endif

//#ifdef IMAGE_ARMOR_GET_ID
//    cv::imshow("classifier", classifier.img_bkg);
//    classifier.resetBackground();
//#endif

}


/*void ArmorDetector::getPoints(Armor& arm_target)
 *@brief 从目标中获取点，用于角度解算
 *@param arm_target 目标装甲
 */
void ArmorDetector::getPoints(Armor& arm_target) {
    Data& data = Data::getData();
    Parameter& param = Parameter::getParameter();
    cv::Point2f pts_tmp[4];
    cv::Point2f pts_2d[4];
    cv::Point pt_offset = cv::Point(100, 100) - static_cast<cv::Point>(param.armor.pt_offset_cap); //自瞄短焦相机成像偏移坐标补偿

    /*  函数模版
     *  _Pt convertSourcePoint(_Pt pt)
    *@brief 得到roi点在原图的位置
    *@param pt roi上要转换的点
    *@return 原图上的点
    */
    //     3---0  3           2  3---0
    //     I   I   II-------II   I   I
    //     I   I   II   5   II   I   I
    //     I   I   II-------II   I   I
    //     2---1  0           1  2---1
    arm_target.led_sticks[0].rrect.points(pts_tmp);// 返回包含左边旋转矩形的4个顶点
    pts_2d[0] = convertSourcePoint(static_cast<cv::Point>(pts_tmp[1]) + pt_offset);
    pts_2d[3] = convertSourcePoint(static_cast<cv::Point>(pts_tmp[0]) + pt_offset);
    arm_target.led_sticks[1].rrect.points(pts_tmp);// 返回包含右边旋转矩形的4个顶点
    pts_2d[1] = convertSourcePoint(static_cast<cv::Point>(pts_tmp[2]) + pt_offset);
    pts_2d[2] = convertSourcePoint(static_cast<cv::Point>(pts_tmp[3]) + pt_offset);

    //瞄准点
    cv::Point2f pt_aim = cv::Point((pts_2d[0].x + pts_2d[2].x)/2,(pts_2d[0].y + pts_2d[2].y)/2); // 瞄准装甲中心
    cv::circle(data.img_show, pt_aim, 3, BGR(255, 255, 255), 2);                                 // 在瞄准中心画圆 圆的颜色为白色
    //装甲
    for(int i = 0; i < 4; i++)
        cv::line(data.img_show, pts_2d[i%4], pts_2d[(i+1)%4], BGR(128, 0, 128), 3);              // 装甲区域画线 线的颜色为紫色

    data.pts_2d.clear();//std::vector<cv::Point2f> pts_2d;	//检测出的目标点  容器清空
    for (int i = 0; i < 4; i++)
    {
        data.pts_2d.push_back(pts_2d[i]);//把新检测出的目标点放入datad交互数据的容器pts_2d中
        /*//debug222
        std::cout << pts_2d[i] <<"\t" */
    }

//    //debug222
//    std::cout << std::endl;
//    std::cout << std::endl;
//    std::cout << std::endl;
}


/*
 *@brief 根据装甲更新
 *@param arm    要更新的装甲数据
 *@param pt_roi roi图像偏移，即roi左上角点在源图像位置
 */
void ArmorHistory::update(Armor arm, cv::Point pt_roi) {
    //bool flag; 数据是否有效
    flag = true;
    //如果arm的id不为0 那就把arm的id给id_last
    if (arm.id != 0)
        id_last = arm.id;
    type_last = arm.type_armor;//arm的装甲类型 赋值给 上一次装甲类的 装甲类型
    cnt_update = 0;            //expend计数器为0
    rect_last = arm.rect;      //arm的灯条对构成的矩形区域 = 上一次装甲类的 上次的装甲板矩形位置
    rect_last.x += pt_roi.x;   //上次的装甲板矩形位置x pt_roi以原来的像素坐标系为参考 rect_last以roi左上角点为原点作为参考
    rect_last.y += pt_roi.y;   //上次的装甲板矩形位置y pt_roi以原来的像素坐标系为参考 rect_last以roi左上角点为原点作为参考
}


/*
 *@brief 没有检测到的情况，根据计数器决定是否重置
 *@param sz_img 图像尺寸
 */
void ArmorHistory::update(cv::Size sz) {
    if (++cnt_update >= cnt_max) {
        flag = false; //数据不有效
        id_last = 0;  //上次得到的装甲id置为0

        //
        rect_last.x -= sz.width/4;//上次的装甲板矩形位置x
        if (rect_last.x < 0)
            rect_last.x = 0;
        //
        rect_last.y += sz.height/4;//上次的装甲板矩形位置y
        if (rect_last.y < 0)
            rect_last.y = 0;
        //
        rect_last.width += sz.width/4;
        if (rect_last.x + rect_last.width >= sz.width)
            rect_last.width = sz.width - rect_last.x;
        //
        rect_last.height += sz.height/4;
        if (rect_last.y + rect_last.height >= sz.height)
            rect_last.height = sz.height - rect_last.y;
    }
}


