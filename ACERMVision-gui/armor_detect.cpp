#include "armor_detect.h"
#include <iterator>

LEDPair::LEDPair(const LEDStick& l1, const LEDStick& l2) {
    if (l1.rrect.center.x < l2.rrect.center.x) {
        led_sticks[0] = l1;
        led_sticks[1] = l2;
    }
    else {
        led_sticks[0] = l2;
        led_sticks[1] = l1;
    }

    angle_err = fabs(l1.rrect.angle - l2.rrect.angle);
    rate_len = fabs(l1.length / l2.length);
    rect.width = abs(static_cast<int>(l1.rrect.center.x - l2.rrect.center.x));
    rect.height = static_cast<int>((l1.rrect.size.height + l2.rrect.size.height) / 2);
    pt_cen.x = static_cast<int>((l1.rrect.center.x + l2.rrect.center.x) / 2);
    pt_cen.y = static_cast<int>((l1.rrect.center.y + l2.rrect.center.y) / 2);

    rect.x = pt_cen.x - rect.width / 3;
    rect.y = pt_cen.y - rect.height / 3;
    rect.width = static_cast<int>(rect.width * 2.0/3);
    rect.height = static_cast<int>(rect.height * 2.0/3);
}

int LEDPair::getAverageIntensity(cv::Mat& img) const {
    if (rect.width < 1 || rect.height < 1 || rect.x < 0 || rect.y < 0)
        return 255;
    return Tool::calculateRoiAverageIntensity(img, rect);
}

bool LEDPair::isSuitableSize() const {

    if(led_sticks[0].rrect.size.height*0.7f > led_sticks[1].rrect.size.height
        || led_sticks[0].rrect.size.height*1.3f < led_sticks[1].rrect.size.height)
        return false;

    float h_avg = (led_sticks[0].rrect.size.height + led_sticks[1].rrect.size.height)/2.0f;
    if(fabs(led_sticks[0].rrect.center.y - led_sticks[1].rrect.center.y) < 0.8f* h_avg
        && h_avg*2.7f > rect.width && h_avg < 2.0f* rect.width)
        return true;

    return false;
}

std::vector<cv::Point2f> Armor::points = {
    cv::Point(0, 64), cv::Point(64, 64),
    cv::Point(64, 0), cv::Point(0, 0)
};

Armor::Armor(const LEDPair& pair, const cv::Size& size_image):
        LEDPair(pair), size_img(size_image), is_empty(false) {

    rect.width = static_cast<int>(led_sticks[1].rrect.center.x -
            led_sticks[0].rrect.center.x);
    double height_led = static_cast<double>((led_sticks[0].rrect.size.height +
            led_sticks[0].rrect.size.height) / 2);
    pt_cen.x = static_cast<int>((led_sticks[0].rrect.center.x +
            led_sticks[1].rrect.center.x) / 2);
    pt_cen.y = static_cast<int>((led_sticks[0].rrect.center.y +
            led_sticks[1].rrect.center.y) / 2);

    //灯条大约6cm，装甲高大约12cm，小装甲宽大约12cm，大装甲宽大约24cm
    rect.height = static_cast<int>(height_led * 2);
    rect.x = pt_cen.x - rect.width / 2;
    rect.y = pt_cen.y - rect.height / 2;
    if (rect.y < 0)
        rect.y = 0;
    if (rect.height + rect.y > size_img.height)
        rect.height = size_img.height - rect.y - 1;


    pts[0] = cv::Point(rect.x, rect.y + rect.height);
    pts[1] = cv::Point(rect.x + rect.width, rect.y + rect.height);
    pts[2] = cv::Point(rect.x + rect.width, rect.y);
    pts[3] = cv::Point(rect.x, rect.y);
}

void Armor::getROIImage(const cv::Mat& img_roi,cv::Mat& img_num) {
   // double t_begin=0,t_end=0;
   // t_begin=cv::getTickCount();
    //-------------提升img_roi亮度,得到img_gamma----------------//
    cv::Mat img_gamma ;
    cv::Mat lookUpTable(1,256,CV_8UC3);//0~255分别非线性矫正，放在lookUpTable
    uchar *p = lookUpTable.ptr();
    double gammaa=0.15;//gamma小于1时，亮度提升
    for(int i = 0;i < 256;i++)
        p[i] = cv::saturate_cast<uchar>(pow((float)i/255.0,gammaa) * 255.0f);//像素点的幂函数转换
    cv::LUT(img_roi,lookUpTable,img_gamma);
    //--------------对图片进行透视变换--------------------//
    std::vector<cv::Point2f> points(pts, pts + 4);
    cv::Mat img_gray = cv::Mat::zeros(64, 64, CV_8UC1);
    cv::Mat mat_trans = cv::getPerspectiveTransform(points, Armor::points);
    cv::warpPerspective(img_gamma, img_gray, mat_trans, img_gray.size());
    //---------涂黑灯条，只截取中间数字部分,减少涣散的灯光对成像质量的影响----------------//
    cv::cvtColor(img_gray, img_gray, cv::COLOR_BGR2GRAY);
    cv::rectangle(img_gray, cv::Point(0, 0), cv::Point(15, 63), BGR::all(0), -1);
    cv::rectangle(img_gray, cv::Point(50, 0), cv::Point(63, 63), BGR::all(0), -1);
    //-------------直方图均衡化--滤波去噪-----------------//
    cv::equalizeHist(img_gray, img_gray);
    cv::medianBlur(img_gray,img_gray,3);
    //------------训练的lenet要求输入48*48的彩色图-----------------//
    resize(img_gray,img_gray,cv::Size(48,48),0,0,cv::INTER_LINEAR);
    cvtColor(img_gray, img_num, cv::COLOR_GRAY2BGR);
   // t_end=cv::getTickCount();
   // std::cout<<"spend "<<((t_end-t_begin)/cv::getTickFrequency()*1000)<<" ms"<<std::endl;
    cv::imshow("num",img_num);
}

void Armor::getTypeBySize() {

    double height_led = static_cast<double>((led_sticks[0].rrect.size.height +
            led_sticks[0].rrect.size.height) / 2);
    const double ratio_wh = rect.width / height_led;	//灯条长和装甲宽度之比
    if (ratio_wh > 3. && ratio_wh < 5.)
        type_armor = TARGET_BIG_ARMOR;
    else if (ratio_wh > 1.2 && ratio_wh <= 3.)
        type_armor = TARGET_SMALL_ARMOR;
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

void ArmorClassifier::getResult(const cv::Mat& img, int& type_sz, int& id) {

    cv::Mat input_blob = cv::dnn::blobFromImage(img, 1, cv::Size(64, 64));

    net.setInput(input_blob, "data");

    cv::Mat pred = net.forward("prob");

    cv::Point pt;
    cv::minMaxLoc(NULL, NULL, NULL, NULL, &pt);
    int res = pt.x;
    type_sz = res < 8 ? TARGET_SMALL_ARMOR : TARGET_BIG_ARMOR;
    id = res % 8 + 1;


    draw(img, type_sz, id);


}

#elif TYPE_ARMOR_CLASSIFIER == 2

void ArmorClassifier::getResult(const cv::Mat& img_num, int& type_sz, int& id){

    cv::Mat img_trans;          //transform后的装甲板图像
    // ImageNet的均值和标准差，用于归一化Normalize
    std::vector<float> mean_value = {0.485, 0.456, 0.406};
    std::vector<float> std_value  = {0.229, 0.224, 0.225};

    std::vector<cv::Mat> bgrChannels(3);
    cv::split(img_num, bgrChannels);
    // 数据缩放和归一化
    for(unsigned int i = 0; i < bgrChannels.size(); i++){
        // dst = ((src / 255) - mean_value) / std_value
        bgrChannels[i].convertTo(bgrChannels[i], CV_32FC1, 1.0 / 255, -mean_value[i]);
        bgrChannels[i].convertTo(bgrChannels[i], CV_32FC1, 1.0 / std_value[i], 0.0);
    }
    cv::merge(bgrChannels, img_trans);

    cv::Mat blob = cv::dnn::blobFromImage(img_trans);
    net.setInput(blob);
    cv::Mat pred = net.forward();                        // pred为1×n的vector(n为要分类的装甲板数字个数)

    cv::Point maxLoc;
    cv::minMaxLoc(pred, NULL, NULL, NULL, &maxLoc);     // 找到pred中置信度最高的项
    id = maxLoc.x + 1;                              // 置信度最高项的坐标x值即为1×n vector的索引值，即装甲板id
    std::cout << "num: " << id << std::endl;
    type_sz = id == 1 ? TARGET_BIG_ARMOR : TARGET_SMALL_ARMOR;
}

#endif

bool ArmorDetector::run() {

    Data& data = Data::getData();

    data.img_show = img.clone();

    setROI(); //roi区域矩形

    Armor arm_target;    //目标装甲
    fitArmor(arm_target);  //对得到的灯条进行匹配

    if (arm_target.is_empty)
        return false;

    getPoints(arm_target);
    return true;

}

//哨兵模式辅助相机使用
bool ArmorDetector::findArmor(cv::Mat& img_src, Parameter& parameter, float angle_err, float intensity_avg) {

    Data& data = Data::getData();

    cv::Mat img_color;	//颜色通道相减图像
    cv::Mat img_gray;

    std::vector<cv::Mat> channels;	//bgr通道
    cv::split(img_src, channels);

    if (parameter.other.color_enemy == TARGET_RED)
        subtract(channels[2], channels[0], img_color);
    else
        subtract(channels[0], channels[2], img_color);

    cv::threshold(img_gray, img_gray, parameter.armor.th_gray, 255, cv::THRESH_BINARY);
    cv::threshold(img_color, img_color, parameter.armor.th_color, 255, cv::THRESH_BINARY);

    std::vector<std::vector<cv::Point>> contours_color;	//颜色通道相减图像（外部）轮廓
    std::vector<std::vector<cv::Point>> contours_gray;	//灰度图像（外部）轮廓
    cv::findContours(img_color, contours_color, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    cv::findContours(img_gray, contours_gray, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    SticksSet led_sticks;

    size_t sz_gray = contours_gray.size();		//灰度轮廓数量
    for (size_t i = 0; i < sz_gray; i++) {
        double area = cv::contourArea(contours_gray[i]);	//灰度单条轮廓面积
        if (area < 20.0 || 1e5 < area)
            continue;
        for (size_t j = 0; j < contours_color.size(); j++) {
            if (cv::pointPolygonTest(contours_color[j], contours_gray[i][0], false) >= 0.0) {
                double length = cv::arcLength(cv::Mat(contours_gray[i]), true);	//灰度轮廓周长
                if (length < 30 || length > 400)//排除周长不合理
                    continue;
                cv::RotatedRect rrect = cv::fitEllipse(cv::Mat(contours_gray[i]));	//拟合的旋转矩阵
                if (rrect.angle > 90.0f)	//标准化
                    rrect.angle = rrect.angle - 180.0f;

                if (abs(rrect.angle) >= 30)		//筛选倾斜过度的灯条
                    continue;
                led_sticks.emplace_back(rrect);


                cv::RotatedRect rrect_draw = rrect;
                Tool::drawRotatedRectangle(img_src, rrect_draw, BGR(255, 0, 255), 1);


                break;
            }
        }
    }

//    cv::resize(img_color, img_color, cv::Size(img_color.cols/2, img_color.rows/2));
//    cv::imshow("color", img_color);

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

void ArmorDetector::setROI() {

    Data& data = Data::getData();

    cv::Rect2f rect_roi;  //roi矩形框
    float ratio_w = 2.5;
    float ratio_h = 1.5;

    if (is_useroi && history.flag && (is_ignore2 == true ||
            ((history.id_last == 2 && history.id_last != 2)))) {

        rect_roi = history.rect_last;
        rect_roi.x -= rect_roi.width * ((ratio_w-1)/2);
        rect_roi.width = static_cast<int>(rect_roi.width * ratio_w);
        rect_roi.y -= rect_roi.height * ((ratio_h-1)/2);
        rect_roi.height *= ratio_h;

        if (rect_roi.x < 0)
            rect_roi.x = 0;
        if (rect_roi.y < 0)
            rect_roi.y = 0;
        if (rect_roi.x + rect_roi.width >= img.cols)
            rect_roi.width = img.cols - rect_roi.x - 1;
        if (rect_roi.y + rect_roi.height >= img.rows)
            rect_roi.height = img.rows - rect_roi.y - 1;
        if (rect_roi.width <= 0 || rect_roi.height <= 0)
            rect_roi = cv::Rect(0, 0, img.cols, img.rows);
    }
    else
        rect_roi = cv::Rect(0, 0, img.cols, img.rows);
    img_roi = img(rect_roi);


    if (is_useroi)
        cv::rectangle(data.img_show, rect_roi, BGR::all(255), 3);


    pt_roi = cv::Point2f(rect_roi.x, rect_roi.y);


    if (rect_roi.width != img.cols &&
            rect_roi.height != img.rows) {
        //cv::rectangle(img_show, rect_roi, BGR::all(255), 3);
        win_roi.showImage(img_roi);
    }
    else
        win_roi.destory();



}

void ArmorDetector::findLamps(cv::Mat& img_gray, SticksSet& led_sticks) {

    Data& data = Data::getData();
    Parameter& param = Parameter::getParameter();
    led_sticks.clear();
    cv::Mat img_color;	//颜色通道相减图像

    std::vector<cv::Mat> channels;	//bgr通道
    cv::split(img_roi, channels);

    if (param.other.color_enemy == TARGET_RED)
        subtract(channels[2], channels[0], img_color);
    else
        subtract(channels[0], channels[2], img_color);

    cv::threshold(img_gray, img_gray, param.armor.th_gray, 255, cv::THRESH_BINARY);
    cv::threshold(img_color, img_color, param.armor.th_color, 255, cv::THRESH_BINARY);

    std::vector<std::vector<cv::Point>> contours_color;	//颜色通道相减图像（外部）轮廓
    std::vector<std::vector<cv::Point>> contours_gray;	//灰度图像（外部）轮廓
    cv::findContours(img_color, contours_color, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    cv::findContours(img_gray, contours_gray, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    size_t sz_gray = contours_gray.size();		//灰度轮廓数量
    for (size_t i = 0; i < sz_gray; i++) {
        double area = cv::contourArea(contours_gray[i]);	//灰度单条轮廓面积
        if (area < 20.0 || 1e5 < area)
            continue;
        for (size_t j = 0; j < contours_color.size(); j++) {
            if (cv::pointPolygonTest(contours_color[j], contours_gray[i][0], false) >= 0.0) {
                double length = cv::arcLength(cv::Mat(contours_gray[i]), true);	//灰度轮廓周长
                if (length < 30 || length > 400)//排除周长不合理
                    continue;
                cv::RotatedRect rrect = cv::fitEllipse(cv::Mat(contours_gray[i]));	//拟合的旋转矩阵
                if (rrect.angle > 90.0f)	//标准化
                    rrect.angle = rrect.angle - 180.0f;

                if (abs(rrect.angle) >= 30)		//筛选倾斜过度的灯条
                    continue;
                led_sticks.emplace_back(rrect);


                cv::RotatedRect rrect_draw = rrect;
                rrect_draw.center = convertSourcePoint(rrect_draw.center);
                Tool::drawRotatedRectangle(data.img_show, rrect_draw, BGR(255, 0, 255), 1);


                break;
            }
        }
    }

//    cv::resize(img_color, img_color, cv::Size(img_color.cols/2, img_color.rows/2));
//    cv::imshow("color", img_color);

}

void ArmorDetector::fitArmor(Armor& arm_target, float angle_err, float intensity_avg) {
    Parameter& param = Parameter::getParameter();
    Data& data = Data::getData();
    cv::Mat img_gray;	                          //灰度图像
    cv::Mat img_num(cv::Size(64,64),CV_8UC3);     //数字图像
    cv::cvtColor(img_roi, img_gray, cv::COLOR_BGR2GRAY);

    SticksSet led_sticks;
    findLamps(img_gray, led_sticks);

//    cv::Point pt_aim = cv::Point(img.cols/2 + param.armor.pt_offset_cap.x - 100, \
//            img.rows/2 + 100 - param.armor.pt_offset_cap.y) - pt_roi;    //补偿后的中心
    cv::Point pt_aim = cv::Point(img.cols/2, img.rows/2) - pt_roi;    //补偿后的中心
    typedef std::set<Armor, Armor::ArmorComparer> ArmorSet;
    Armor::ArmorComparer comparer_arm(pt_aim);
    ArmorSet arms(comparer_arm);

    size_t sz_leds = led_sticks.size();	//灯条数量
    ArmorSet::iterator iter;
    for (size_t i = 0; i < sz_leds; i++)
        for (size_t j = i + 1; j < sz_leds; j++) {
            LEDPair pair_led(led_sticks.at(i), led_sticks.at(j));	//临时灯条对
            if (pair_led.angle_err > angle_err
                    || pair_led.rate_len > 1.5f
                    || pair_led.isSuitableSize() == false
                    || pair_led.getAverageIntensity(img_gray) > intensity_avg)
                continue;
            Armor arm(pair_led, img.size());
            if (is_classifier) {
                arm.getROIImage(img_roi, img_num);
                classifier.getResult(img_num, arm.type_armor, arm.id);//取装甲识别的数值
                cv::putText(data.img_show, std::to_string(arm.id), convertSourcePoint(arm.pt_cen),
                    cv::FONT_HERSHEY_SIMPLEX, 2, BGR::all(255), 2);//putText 在图像上绘制文字
//                if (rule_choose == ARMOR_LAST_ID_FIRST && arm.id == history.id_last
//                        && (is_ignore2 || arm.id != 2)) {
//                    history.update(arm, pt_roi);
//                    arm_target = arm;
//                    goto END_FIT_ARMOR;
//                }
            }
            else
                arm.getTypeBySize();

            arms.emplace(std::move(arm));

        }

    for (iter=arms.begin(); iter!=arms.end(); ++iter) {
        if (iter->id == 2 && is_ignore2)
            continue;
        arm_target = *iter;
        history.update(arm_target, pt_roi);
        break;
    }
    if (iter == arms.end())
        history.update(img.size());


//END_FIT_ARMOR:
//    ;

    Tool::drawCross(data.img_show, convertSourcePoint(pt_aim), 5, BGR::all(255), 1);

//#ifdef IMAGE_SHOW
//    cv::resize(img_gray, img_gray, cv::Size(img_gray.cols/2, img_gray.rows/2));
//    cv::imshow("gray", img_gray);
//#endif

//#ifdef IMAGE_ARMOR_GET_ID
//    cv::imshow("classifier", classifier.img_bkg);
//    classifier.resetBackground();
//#endif

}

void ArmorDetector::getPoints(Armor& arm_target) {
    Data& data = Data::getData();
    Parameter& param = Parameter::getParameter();
    cv::Point2f pts_tmp[4];
    cv::Point2f pts_2d[4];
//    cv::Point pt_offset = cv::Point(100, 100) - static_cast<cv::Point>(param.armor.pt_offset_cap);

    arm_target.led_sticks[0].rrect.points(pts_tmp);
//    pts_2d[0] = convertSourcePoint(static_cast<cv::Point>(pts_tmp[1]) + pt_offset);
//    pts_2d[3] = convertSourcePoint(static_cast<cv::Point>(pts_tmp[0]) + pt_offset);
    pts_2d[0] = convertSourcePoint(static_cast<cv::Point>(pts_tmp[1]));
    pts_2d[3] = convertSourcePoint(static_cast<cv::Point>(pts_tmp[0]));
    arm_target.led_sticks[1].rrect.points(pts_tmp);
//    pts_2d[1] = convertSourcePoint(static_cast<cv::Point>(pts_tmp[2]) + pt_offset);
//    pts_2d[2] = convertSourcePoint(static_cast<cv::Point>(pts_tmp[3]) + pt_offset);
    pts_2d[1] = convertSourcePoint(static_cast<cv::Point>(pts_tmp[2]));
    pts_2d[2] = convertSourcePoint(static_cast<cv::Point>(pts_tmp[3]));

    //瞄准点
    cv::Point2f pt_aim = cv::Point((pts_2d[0].x + pts_2d[2].x)/2,(pts_2d[0].y + pts_2d[2].y)/2);
    cv::circle(data.img_show, pt_aim, 3, BGR(255, 255, 255), 2);
    //装甲
    for(int i = 0; i < 4; i++)
        cv::line(data.img_show, pts_2d[i%4], pts_2d[(i+1)%4], BGR(128, 0, 128), 3);

    data.pts_2d.clear();
    for (int i = 0; i < 4; i++)
        data.pts_2d.push_back(pts_2d[i]);

}

void ArmorHistory::update(Armor arm, cv::Point pt_roi) {
    flag = true;
    if (arm.id != 0)
        id_last = arm.id;
    type_last = arm.type_armor;
    cnt_update = 0;
    rect_last = arm.rect;
    rect_last.x += pt_roi.x;
    rect_last.y += pt_roi.y;
}

void ArmorHistory::update(cv::Size sz) {
    if (++cnt_update >= cnt_max) {
        flag = false;
        id_last = 0;

        rect_last.x -= sz.width/4;
        if (rect_last.x < 0)
            rect_last.x = 0;
        rect_last.y += sz.height/4;
        if (rect_last.y < 0)
            rect_last.y = 0;
        rect_last.width += sz.width/4;
        if (rect_last.x + rect_last.width >= sz.width)
            rect_last.width = sz.width - rect_last.x;
        rect_last.height += sz.height/4;
        if (rect_last.y + rect_last.height >= sz.height)
            rect_last.height = sz.height - rect_last.y;
    }
}

