#include "buff_detect.h"
#include "utils.h"


/*
 *@brief 绘制最终装甲板的中心点(画圆)
 *@param image 要处理的原图
 */
/*
 *  @SMALL	0，未知状态
 *	@BIG	1，未激活
 *	@NULL	2，已激活
 * BUFF_UNKNOW = 0,
 * BUFF_INACTION = 1,
 * BUFF_ACTION = 2
 *
 */
void BuffFlabellum::drawTarget(cv::Mat &img) {
    if (type == BUFF_INACTION)    //能量机关未激活 中心点画绿
        cv::circle(img, rrect_small.center, 3, cv::Scalar(0, 0, 255), -1);
    else if (type == BUFF_ACTION) //能量机关已激活 中心点画白
        cv::circle(img, rrect_small.center, 3, cv::Scalar(255, 255, 255), -1);
    else                          //未知状态      中心点画白
        cv::circle(img, rrect_small.center, 3, cv::Scalar(255, 255, 255), 1);
}



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

//更新装甲板相机坐标系顺序
//始终保持装甲板的外圈为0,1
//方便后面计算roi确定扇叶类型
//      0________________1
//      |                |
//      |                |
//      |                |
//      |________________|
//      3       ||       2
//              ||
//              ||
//              ||
//              ||
//              ||
//              ||
//              ||
//              ||
/*
 *         旋转矩形返回的四个点
 *        p[1]*---------*p[2]
 *            -----------
 *            -----------height
 *            -----------
 *            -----------
 *        p[0]*-(width)-*p[3]
 *              angle=0'
 */
void BuffFlabellum::update2DPoints() {
    //std::vector<cv::Point2f> pts_2d;	//pnp角度解算的四个装甲点
    pts_2d.clear(); // vector清空

    cv::Point2f points[4]; // vector初始化

    //cv::RotatedRect rrect_small 能量机关扇叶内轮廓(即装甲板或其他内部轮廓)
    rrect_small.points(points); // 构造旋转矩阵新的四个点

    //取新的装甲中心
    cv::Point2f pt_cen_up = (points[0] + points[1]) / 2;   //扇叶上边缘的中心位置
    cv::Point2f pt_cen_down = (points[2] + points[3]) / 2; //扇叶下边缘的中心位置

    //cv::RotatedRect rrect_big 能量机关扇叶外轮廓
    //求扇叶中心和装甲板的两条长边的距离
    double dist_up = Tool::calculateDistance(pt_cen_up, rrect_big.center);     //扇叶中心到装甲板的上边缘长边的距离
    double dist_down = Tool::calculateDistance(pt_cen_down, rrect_big.center); //扇叶中心到装甲板的下边缘长边的距离

    //通过比较扇叶中心和装甲板的两条长边的距离来确定
    //装甲的点push_back的顺序根据该扇叶是否在y>0的象限
    //FIT_ELLIPASE 使用椭圆将矩形填充
    if (dist_up > dist_down) {           //该扇叶在y>0的象限
        angle = rrect_small.angle;
        pts_2d.push_back(points[0]);
        pts_2d.push_back(points[1]);
        pts_2d.push_back(points[2]);
        pts_2d.push_back(points[3]);
    }
    else {                               //该扇叶在y<0的象限
        // question1 等待解决
        angle = rrect_small.angle + 180; //float angle; 装甲板的角度
        pts_2d.push_back(points[2]);
        pts_2d.push_back(points[3]);
        pts_2d.push_back(points[0]);
        pts_2d.push_back(points[1]);
    }

    //FIT_MINAREA
    //float width = rrect_small.size.width;
    //float height = rrect_small.size.height;
    //cv::Point2f points[4];
    //rrect_small.points(points);
    //if (width >= height) {
    //    cv::Point2f pt_cen_up = (points[0] + points[3]) / 2;
    //    cv::Point2f pt_cen_down = (points[1] + points[2]) / 2;
    //    float dist_up = Tool::calculateDistance(pt_cen_up, rrect_big.center);
    //    float dist_down = Tool::calculateDistance(pt_cen_down, rrect_big.center);
    //    if (dist_up <= dist_down) {
    //        angle = 90 - rrect_small.angle;
    //        pts_2d.push_back(points[1]); pts_2d.push_back(points[2]);
    //        pts_2d.push_back(points[3]); pts_2d.push_back(points[0]);

    //    }
    //    else {
    //        angle = 270 - rrect_small.angle;
    //        pts_2d.push_back(points[3]); pts_2d.push_back(points[0]);
    //        pts_2d.push_back(points[1]); pts_2d.push_back(points[2]);
    //    }
    //}
    //else {
    //    cv::Point2f pt_cen_up = (points[0] + points[1]) / 2;
    //    cv::Point2f pt_cen_down = (points[2] + points[3]) / 2;
    //    float dist_up = Tool::calculateDistance(pt_cen_up, rrect_big.center);
    //    float dist_down = Tool::calculateDistance(pt_cen_down, rrect_big.center);
    //    if (dist_up <= dist_down) {
    //        angle = -rrect_small.angle;
    //        pts_2d.push_back(points[2]); pts_2d.push_back(points[3]);
    //        pts_2d.push_back(points[0]); pts_2d.push_back(points[1]);

    //    }
    //    else {
    //        angle = 180 - rrect_small.angle;
    //        pts_2d.push_back(points[0]); pts_2d.push_back(points[1]);
    //        pts_2d.push_back(points[2]); pts_2d.push_back(points[3]);
    //    }
    //}

}
#define IMSHOW_2_ROI
//获取扇叶状态：激活 or 未激活
void BuffFlabellum::getStatus(cv::Mat &img) {
	cv::Point2f offset_height = pts_2d.at(0) - pts_2d.at(3);
    //对延伸出来的距离再进行调整
    offset_height.x *= 0.5f;
    offset_height.y *= 0.5f;

    //经验值补偿？
    cv::Point pt_cen_l = pts_2d.at(3) - 2*offset_height; // 右边的点取一小块(前提条件 扇叶在下面)
    cv::Point pt_cen_r = pts_2d.at(2) - 2*offset_height; // 左边的点取一小块(前提条件 扇叶在下面)

	//roi的补偿长宽
    int w_roi = 15;		
    int h_roi = 15;

    cv::Point pt_l1 = cv::Point(pt_cen_l.x - w_roi, pt_cen_l.y - h_roi);
    cv::Point pt_l2 = cv::Point(pt_cen_l.x + w_roi, pt_cen_l.y + h_roi);

    cv::Point pt_r1 = cv::Point(pt_cen_r.x - w_roi, pt_cen_r.y - h_roi);
    cv::Point pt_r2 = cv::Point(pt_cen_r.x + w_roi, pt_cen_r.y + h_roi);

    cv::Rect rect_l(pt_l1, pt_l2);
    cv::Rect rect_r(pt_r1, pt_r2);

    //计算ROI区域的平均强度
    int intensity_l = Tool::calculateRoiAverageIntensity(img, rect_l);
    int intensity_r = Tool::calculateRoiAverageIntensity(img, rect_r);

    //视频测试出未激活的roi强度为90-100 激活的roi强度为150-200
    //ACE大风车测试时为激活为32  已激活为60-70
    if (intensity_l > 10 && intensity_r > 10)   // 通过ROI区域的平均强度 来判断这个扇叶是否被激活
        type = BUFF_ACTION;
    else
        type = BUFF_INACTION;
   
#ifdef IMSHOW_2_ROI
    cv::rectangle(img, rect_l, BGR(255), 1);
    cv::rectangle(img, rect_r, BGR(255), 1);
    cv::putText(img, std::to_string(intensity_l), pt_cen_l, cv::FONT_HERSHEY_SIMPLEX, 0.5, BGR(128, 0, 128));
    cv::putText(img, std::to_string(intensity_r), pt_cen_r, cv::FONT_HERSHEY_SIMPLEX, 0.5, BGR(128, 0, 128));
    cv::imshow("roi_test", img);
#endif
}

//虽然目前视频测试效果还行，TODO:先遍历全部轮廓后取出筛选出只有一个子轮廓的（即筛选掉装甲板轮廓的父轮廓含有多个子轮廓的），装甲板父轮廓的面积限制

// 找出图像中的能量机关扇叶，并进行分类 vec_target 扇叶候选目标集
void BuffDetector::findFlabellums(std::vector<BuffFlabellum>& vec_target) {

    Parameter& param = Parameter::getParameter();
    Data& data = Data::getData();

    cv::Mat element = getStructuringElement(cv::MORPH_ELLIPSE, Size(5, 5)); // 形态学操作所需要的核（获取结构元素）
    //膨胀 暂时膨胀了两次
    cv::dilate(img, img, element);
    cv::dilate(img, img, element);


    // **预处理** -图像进行相应颜色的二值化
    std::vector<cv::Mat> channels_bgr;
    cv::split(img, channels_bgr);
    cv::Mat img_res;
    //通道相减
    if (param.other.color_enemy != 0)
        cv::subtract(channels_bgr[2], channels_bgr[0], img_res);
	else
        cv::subtract(channels_bgr[0], channels_bgr[2], img_res);

    cv::Mat img_bin;

//    大津法二值化
//    double th = threshold(img_res, img_bin, 50, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
//    if (th - 10 > 0)
//        threshold(img_res, img_bin, th - 10, 255, CV_THRESH_BINARY);
//    if (th < 20)
//        return 0;

    cv::threshold(img_res, img_bin, param.buff.th_color, 255, cv::THRESH_BINARY); // 直接固定阈值式二值化

#ifdef DEBUG_SHOW_BIN
    cv::imshow("mask", img_bin);
#endif


    // **寻找击打矩形目标** -通过几何关系
    // 寻找识别物体并分类到Buff
    // vector<Rect> vec_color_rect;


    // 定义轮廓
    //std::vecotr<cv::Point>轮廓，通过一组点来描述一个轮廓，而要求是要六个点来描述一个轮廓（因为后面要用到拟合椭圆函数需要6个点）
    //std::vector<std::vector<cv::Point>> 一个个轮廓组成的轮廓容器
    std::vector<std::vector<cv::Point>> contours;

    //定义层次结构
    std::vector<cv::Vec4i> hierarchy;

    // 提取所有轮廓，并建立双层结构关系
    cv::findContours(img_bin, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);

    //    调试排错时使用
    //    for(int z = 0;z<contours.size();z++){
    //        if(contours[z].size() >= 6)
    //        {
    //            cv::RotatedRect which_contour = cv::fitEllipse(contours[z]);
    //            cv::Point2f which_point_tmp[4];
    //            which_contour.points(which_point_tmp);

    //            for (int k = 0; k < 4; k++)
    //            {
    //                cv::line(img_show, which_point_tmp[k], which_point_tmp[(k + 1) % 4], cv::Scalar(128, 0, 128), 2);
    //            }
    //        }
    //    }

    //
    for (size_t i = 0; i < contours.size(); i++) {
        // 用于寻找小轮廓，1、没有父轮廓的轮廓跳过 2、不满足6点拟合椭圆的轮廓跳过 3、有父轮廓的但是不满足6点拟合椭圆的轮廓跳过
        if (hierarchy[i][3] < 0 || contours[i].size() < 6
				|| contours[static_cast<uint>(hierarchy[i][3])].size() < 6)
            continue;

        // 小轮廓面积 周长判断
		double s_small_rect = cv::contourArea(contours[i]);
        if (s_small_rect < 500)//1500
            continue;

        double l_small_rect = cv::arcLength(contours[i], true);
        if (l_small_rect < 100)
            continue;

        // 大轮廓面积 周长判断
        double s_big_rect = cv::contourArea(contours[static_cast<uint>(hierarchy[i][3])]); // 大轮廓面积一般为包括整个扇叶的面积
        if (s_big_rect < 3000 && s_big_rect >6000)
            continue;

		double l_big_rect = cv::arcLength(contours[static_cast<uint>(hierarchy[i][3])], true);
        if (l_big_rect < 300)
            continue;

        // 通过检测出来的轮廓点 用椭圆拟合 来声明外轮廓和内轮廓
		cv::RotatedRect rrect_small = cv::fitEllipse(contours[i]);
		cv::RotatedRect rrect_big = cv::fitEllipse(contours[static_cast<uint>(hierarchy[i][3])]);

//#ifdef DEBUG_DRAW_CONTOURS		//绘制大小扇叶轮廓
		cv::Point2f pts_small_tmp[4];
		rrect_small.points(pts_small_tmp);
		cv::Point2f pts_big_tmp[4];
		rrect_big.points(pts_big_tmp);
		for (int k = 0; k < 4; k++) {
            cv::line(data.img_show, pts_small_tmp[k], pts_small_tmp[(k + 1) % 4], cv::Scalar(128, 0, 128), 2);
            cv::line(data.img_show, pts_big_tmp[k], pts_big_tmp[(k + 1) % 4], cv::Scalar(128, 0, 128), 2);
		}
//#endif

		//minAreaRect
		//flabellum.rrect_small = cv::minAreaRect(contours[i]);
		//flabellum.rrect_big = cv::minAreaRect(contours[static_cast<uint>(hierarchy[i][3])]);

        float ratio_small_rect_sz = rrect_small.size.width > rrect_small.size.height ?  // 始终保持扇叶内轮廓的长宽比大于1
			static_cast<float>(rrect_small.size.width) / rrect_small.size.height
			: static_cast<float>(rrect_small.size.height) / rrect_small.size.width;

        if (ratio_small_rect_sz >= 3) // 非检测对象(或者误检） 长宽比过大
			continue;
		
        float angle_diff_rect = fabsf(rrect_big.angle - rrect_small.angle); // 外轮廓的角度与内轮廓的角度之差的绝对值

        if (angle_diff_rect >= 100 || angle_diff_rect <= 80)
            continue;                // 非检测对象(或者误检）  角度之差的绝对值过大


        BuffFlabellum flabellum(rrect_small, rrect_big);// 通过 经过检测出来的轮廓点声明的外轮廓和内轮廓来声明一个能量机关扇叶类
        flabellum.update2DPoints(); //   更新能量机关装甲板的绝对位置
        flabellum.getStatus(img_bin);//  根据两个ROI区域亮度值判断能量机关扇叶的状态（激活or未激活）
        vec_target.push_back(flabellum); // vec_target 扇叶候选目标集
    }
}



//功能：根据规则从扇叶集中选出最终要击打的能量机关装甲
//@输入：
//@param vec_target 候选扇叶集
//@输出：
//@target_final 目标装甲
//@return       选择情况
bool BuffDetector::chooseTarget(std::vector<BuffFlabellum>& vec_target, BuffFlabellum& target_final) {

    if (vec_target.empty())
        return false;

    // 初始化
    Data& data = Data::getData();
    bool flag_find = false;
    float th_angle_diff = 1e8;


    // 遍历所有结果并处理/选择需要击打的目标
    // 需要击打的能量机关类型 1(true)击打未激活 0(false)击打激活
    for (size_t i = 0; i < vec_target.size(); i++) {
        BuffFlabellum buff_tmp = vec_target.at(i);
        // 普通模式击打未激活机关
        if (buff_tmp.type == BUFF_INACTION) {
            flag_find = true; // 直接置为真

            //debug
            //std::cout << "get final target" << std::endl;
            //cv::putText(img_show, "target_final", cv::Point2f(10, -50) + target_final.rrect_small.center, FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));

            float angle = fabs(vec_target[i].angle_diff - 90.0f);
            if (angle < th_angle_diff) {
                target_final = vec_target.at(i);
                th_angle_diff = angle;
            }


            //解析：从chooseTarget中确定旋转角度 以 用于后面判断getResult旋转方向上用到（函数内部数据节点确认）
            //用于获取旋转方向
            angle_buff = target_final.angle;
            //用于获取旋转方向和角度解算
            data.angle_buff = target_final.angle;



#ifdef DEBUG_PUT_TEST_ANGLE
            for (size_t j = 0; j < 4; j++)
                cv::putText(img, to_string(j), cv::Point2f(5, 5) + target_final.pts_2d[j], FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255));
#endif
        }
    }

    //当找到旋转的扇叶之后
    if (flag_find) { // 打印相关信息

//#ifdef DEBUG_PUT_TEST_TARGET
        cv::putText(data.img_show, "<<---final target", cv::Point2f(5,5)+ target_final.rrect_small.center, FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255));
//#endif
//#ifdef DEBUG_DRAW_TARGET
        target_final.drawTarget(data.img_show);
//#endif
    }

    return flag_find;
}


//将相对roi的矩形转换到原图上（设置更新要击打的装甲点）
void BuffDetector::getPoints(BuffFlabellum& target_final) {

    Data& data = Data::getData();

    Parameter& param = Parameter::getParameter();
    cv::Point2f buff_cap_offset = cv::Point2f(param.buff.pt_offset_cap.x - 100,
            100 - param.buff.pt_offset_cap.y);// 构建要补偿的点
    data.pts_2d.clear();// 检测出的目标点清0
    for (size_t k = 0; k < 4; k++)
        data.pts_2d.push_back(target_final.pts_2d.at(k) + buff_cap_offset); // 修正
}

//获取能量机关象限位置
int BuffDetector::getQuadrant(float angle) {
    Data& data = Data::getData();
    int quadrant = 0;
    if (angle >=0 && angle < 90)
        quadrant = BUFF_QUADRANT2;
	else if (angle >= 90 && angle < 180)
        quadrant = BUFF_QUADRANT1;
	else if (angle >= 180 && angle < 270)
        quadrant = BUFF_QUADRANT4;
	else if (angle >= 270 && angle <= 360)
        quadrant = BUFF_QUADRANT3;
	else
        quadrant = BUFF_ORIGIN;


    //判断完之后再赋值
    data.quadrant_buff = quadrant;
    return quadrant;
}



//能量机关检测任务(能量机关检测任务流程)
bool BuffDetector::run() {
    Data& data = Data::getData();
//#ifdef IMAGE_SHOW
    data.img_show = img.clone();
//#endif

    //流程
    //1、找出图像中的能量机关扇叶，并进行分类 vec_target 扇叶候选目标集
    std::vector<BuffFlabellum> vec_target;
    findFlabellums(vec_target);

    //2、找到最后要击打的扇叶
    BuffFlabellum target_final;

    bool flag_find = false;
    flag_find = chooseTarget(vec_target, target_final);

    if (flag_find) {
        getPoints(target_final);
        data.quadrant_buff = getQuadrant(angle_buff);

        //获取旋转方向(每15帧)
        cnt_find++;
        if (cnt_find % 15 == 0) // 每15帧获取一次旋转方向
            getDirection(angle_buff);
    }
    return flag_find;
}

//获取旋转方向
int BuffDetector::getDirection(float angle) {
    angle_diff = angle - angle_last; // 两帧图像中目标的角度差，用于判断旋转方向
    angle_last = angle;              // 上一帧目标的旋转角度
    if (fabs(angle_diff) < 10 && fabs(angle_diff) > 1e-6)
        d_angle = (1 - rate) * d_angle + rate * angle_diff; // float rate r为刷新频率，介于0.0 - 1.0之间 为什么要这样子加呢？ (d_angle 用于判断旋转方向的变量)
    if (d_angle > 2)
        direction = BUFF_CLOCKWISE;    //  顺时针
    else if (d_angle < -2)
        direction = BUFF_ANTICLOCKWISE;   //逆时针
    else
        direction = BUFF_NONE;
	return direction;
}
