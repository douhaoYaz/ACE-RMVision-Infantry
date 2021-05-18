#include "top_detect.h"
#include <iterator>

bool TopDetector::run() {

    Data& data = Data::getData();

    data.img_show = img.clone();


    setROI(); //roi区域矩形

    SticksSet led_sticks;
    std::vector<Armor> arms;	//最终得到的装甲列表
    fitArmor(led_sticks,arms);  //对得到的灯条进行匹配

    LEDPair pair_target;    //目标装甲
    generateArmor(led_sticks,arms, pair_target);

    if (pair_target.is_empty)
        return false;
    getPoints(pair_target);

}

void TopDetector::setROI() {

    Data& data = Data::getData();

    cv::Rect2f rect_roi;  //roi矩形框
    float ratio_w = 2.5;
    float ratio_h = 1.5;

    if (is_useroi && history.flag && (is_ignore2 == false ||
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

void TopDetector::findLamps(cv::Mat& img_gray, SticksSet& led_sticks) {

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

void TopDetector::fitArmor(SticksSet& led_sticks,std::vector<Armor>& arms,
        float angle_err, float intensity_avg) {
    Data& data = Data::getData();
    cv::Mat img_gray;	//灰度图像
    cv::cvtColor(img_roi, img_gray, cv::COLOR_BGR2GRAY);

    findLamps(img_gray, led_sticks);

    arms.clear();
    size_t sz_leds = led_sticks.size();	//灯条数量

    for (size_t i = 0; i < sz_leds; i++) {
        for (size_t j = i + 1; j < sz_leds; j++) {
            LEDPair pair_led(led_sticks.at(i), led_sticks.at(j));	//临时灯条对
            if (pair_led.angle_err > angle_err
                    || pair_led.rate_len > 1.5f
                    || pair_led.isSuitableSize() == false
                    || pair_led.getAverageIntensity(img_gray) > intensity_avg)
                continue;

            Armor arm(pair_led, img.size());
            if (is_classifier) {
                classifier.getResult(img, arm.type_armor, arm.id);


                cv::putText(data.img_show, std::to_string(arm.id), convertSourcePoint(arm.pt_cen),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, BGR::all(255), 2);

//                if (rule_choose == ARMOR_LAST_ID_FIRST && arm.id == history.id_last
//                        && (is_ignore2 || arm.id != 2)) {
//                    history.update(arm, pt_roi);
//                    arm_target = arm;
//                    goto END_FIT_ARMOR;
            }
            else
                arm.getTypeBySize();

            arms.emplace_back(pair_led,img.size());
            led_sticks.at(i).is_match = true;
            led_sticks.at(j).is_match = true;
        }
    }

    //清除已配对成装甲板的灯条
    for(auto iter = led_sticks.begin();iter!=led_sticks.end();) {
        if(iter->is_match == true) {
            iter = led_sticks.erase(iter);
        }else{
            ++iter;
        }
    }

}

void TopDetector::generateArmor(SticksSet& led_sticks,std::vector<Armor>& arms, LEDPair& pair_target) {
    Parameter& param = Parameter::getParameter();
    Data& data = Data::getData();
    //根据灯条分布情况来生成装甲板坐标

    if (arms.empty()) {
        if(!(history.virual_pair_last.is_empty)){
            pair_target = history.virual_pair_last;
        }
        history.update(img.size());
        return;
    }

    cv::Point pt_aim = cv::Point(img.cols/2 + param.armor.pt_offset_cap.x - 100, \
            img.rows/2 + param.armor.pt_offset_cap.y - 100) - pt_roi;    //补偿后的中心
    Tool::drawCross(data.img_show, convertSourcePoint(pt_aim), 5, BGR::all(255), 1);


    int sz = arms.size();

    if (sz = 1) {
        if(led_sticks.size() == 0){
            std::cout<<"one armor no led_stick"<<std::endl;
            //一块装甲没有灯条
            pair_target = arms[0];
            history.update(arms[0],pt_roi);
            history.virual_pair_last = arms[0];
        }else if(led_sticks.size() == 1){
            std::cout<<"one armor one led_stick"<<std::endl;
            //一块装甲和一根灯条
            if(led_sticks.at(0).rrect.center.x >=arms[0].pt_cen.x){

                LEDPair virual_pair(arms[0].led_sticks[0], led_sticks.at(0));	//临时灯条对组成的虚拟装甲板(云台中心)
                pair_target = virual_pair;
                Armor virual_arm(virual_pair,img.size());
                virual_arm.id = arms[0].id;

                //cv::Rect virual_armor(virual_arm.pt_cen.x-0.5*arms[0].rect.width,virual_arm.pt_cen.y-0.5*arms[0].rect.height,arms[0].rect.width,arms[0].rect.height);

                history.update(virual_arm,pt_roi);
                history.virual_pair_last = virual_pair;
            }else{
                LEDPair virual_pair(led_sticks.at(0), arms[0].led_sticks[1]);	//临时灯条对组成的虚拟装甲板(云台中心)
                pair_target = virual_pair;
                Armor virual_arm(virual_pair,img.size());
                virual_arm.id = arms[0].id;
                history.update(virual_arm,pt_roi);
                history.virual_pair_last = virual_pair;
            }


        }else if(led_sticks.size() == 2){
            std::cout<<"one armor two led_stick"<<std::endl;
            //一块装甲和两根灯条
            if(led_sticks.at(0).rrect.center.x <= led_sticks.at(1).rrect.center.x){
                if(arms[0].pt_cen.x <= led_sticks.at(0).rrect.center.x){
                    pair_target = arms[0];
                    history.update(arms[0],pt_roi);
                    history.virual_pair_last = arms[0];
                }
                else{
                    if(arms[0].pt_cen.x <= led_sticks.at(1).rrect.center.x){
                        LEDPair virual_pair(led_sticks.at(0), led_sticks.at(1));	//临时灯条对组成的虚拟装甲板(云台中心)
                        pair_target = virual_pair;
                        Armor virual_arm(virual_pair,img.size());
                        virual_arm.id = arms[0].id;
                        history.update(virual_arm,pt_roi);
                        history.virual_pair_last = virual_pair;
                    }else{
                        LEDPair virual_pair(led_sticks.at(0), arms[0].led_sticks[1]);	//临时灯条对组成的虚拟装甲板(云台中心)
                        pair_target = virual_pair;
                        Armor virual_arm(virual_pair,img.size());
                        virual_arm.id = arms[0].id;
                        history.update(virual_arm,pt_roi);
                        history.virual_pair_last = virual_pair;
                    }
                }
            }
            else{
                if(arms[0].pt_cen.x <= led_sticks.at(1).rrect.center.x){
                    LEDPair virual_pair(arms[0].led_sticks[0], led_sticks.at(0));	//临时灯条对组成的虚拟装甲板(云台中心)
                    pair_target = virual_pair;
                    Armor virual_arm(virual_pair,img.size());
                    virual_arm.id = arms[0].id;
                    history.update(virual_arm,pt_roi);
                    history.virual_pair_last = virual_pair;
                }
                else{
                    if(arms[0].pt_cen.x <= led_sticks.at(0).rrect.center.x){
                        pair_target = arms[0];
                        history.update(arms[0],pt_roi);
                        history.virual_pair_last = arms[0];
                    }else{
                        LEDPair virual_pair(led_sticks.at(1), arms[0].led_sticks[1]);	//临时灯条对组成的虚拟装甲板(云台中心)
                        pair_target = virual_pair;
                        Armor virual_arm(virual_pair,img.size());
                        virual_arm.id = arms[0].id;
                        history.update(virual_arm,pt_roi);
                        history.virual_pair_last = virual_pair;
                    }
                }
            }
        }
        else{
            std::cout<<"one armor more led_stick"<<std::endl;
            //一块装甲和多根灯条
            pair_target = arms[0];
            history.update(arms[0],pt_roi);
            history.virual_pair_last = arms[0];
            return;
        }
    }else if(sz = 2){
//        //两块装甲没有灯条
//        if(led_sticks.size()<1){
//            //与历史ID比较
        std::cout<<"two armor"<<std::endl;
            if(arms[0].id == arms[1].id){
                if(arms[0].pt_cen.x <= arms[1].pt_cen.x){
                    LEDPair virual_pair(arms[0].led_sticks[1], arms[1].led_sticks[0]);	//临时灯条对组成的虚拟装甲板(云台中心)
                    pair_target = virual_pair;
                    Armor virual_arm(virual_pair,img.size());
                    virual_arm.id = arms[0].id;
                    history.update(virual_arm,pt_roi);
                    history.virual_pair_last = virual_pair;
                }else{
                    LEDPair virual_pair(arms[1].led_sticks[1], arms[0].led_sticks[0]);	//临时灯条对组成的虚拟装甲板(云台中心)
                    pair_target = virual_pair;
                    Armor virual_arm(virual_pair,img.size());
                    virual_arm.id = arms[0].id;
                    history.update(virual_arm,pt_roi);
                    history.virual_pair_last = virual_pair;
                }

            }else if(arms[0].id == history.id_last){
                pair_target = arms[0];
                history.update(arms[0],pt_roi);
                history.virual_pair_last = arms[0];
            }else if(arms[1].id == history.id_last){
                pair_target = arms[1];
                history.update(arms[1],pt_roi);
                history.virual_pair_last = arms[1];
            }else{
                if(arms[0].id == arms[1].id){
                    if(arms[0].pt_cen.x <= arms[1].pt_cen.x){
                        LEDPair virual_pair(arms[0].led_sticks[1], arms[1].led_sticks[0]);	//临时灯条对组成的虚拟装甲板(云台中心)
                        pair_target = virual_pair;
                        Armor virual_arm(virual_pair,img.size());
                        virual_arm.id = arms[0].id;
                        history.update(virual_arm,pt_roi);
                        history.virual_pair_last = virual_pair;
                    }else{
                        LEDPair virual_pair(arms[1].led_sticks[1], arms[0].led_sticks[0]);	//临时灯条对组成的虚拟装甲板(云台中心)
                        pair_target = virual_pair;
                        Armor virual_arm(virual_pair,img.size());
                        virual_arm.id = arms[0].id;
                        history.update(virual_arm,pt_roi);
                        history.virual_pair_last = virual_pair;
                    }
                }
            }

//        }else{
//            //两块装甲有灯条
//            history.update();
//            pair_target = history.virual_pair_last;
//            return;
//        }

    }else{//其他情况
        std::cout<<"else else"<<std::endl;
        history.update(img.size());
        pair_target = history.virual_pair_last;
        return;
    }
}

//计算虚拟装甲板的大概视野尺寸并获取四个定点同时更新装甲历史类
void TopDetector::getPoints(LEDPair& pair_target) {
    Data& data = Data::getData();
    Parameter& param = Parameter::getParameter();
    cv::Point2f pts_tmp[4];
    cv::Point2f pts_2d[4];
    //cv::Point pt_offset = cv::Point(100, 100) - static_cast<cv::Point>(param.armor.pt_offset_cap);
    cv::Point pt_offset = cv::Point(0, 0);

    pair_target.led_sticks[0].rrect.points(pts_tmp);
    pts_2d[0] = convertSourcePoint(static_cast<cv::Point>(pts_tmp[1]) + pt_offset);
    pts_2d[3] = convertSourcePoint(static_cast<cv::Point>(pts_tmp[0]) + pt_offset);
    pair_target.led_sticks[1].rrect.points(pts_tmp);
    pts_2d[1] = convertSourcePoint(static_cast<cv::Point>(pts_tmp[2]) + pt_offset);
    pts_2d[2] = convertSourcePoint(static_cast<cv::Point>(pts_tmp[3]) + pt_offset);


    //瞄准点
    cv::Point2f pt_aim = cv::Point((pts_2d[0].x + pts_2d[2].x)/2,(pts_2d[0].y + pts_2d[2].y)/2);
    cv::circle(data.img_show, pt_aim, 3, BGR(255, 255, 255), 2);
    //装甲
    for(int i=0;i<4;i++)
        cv::line(data.img_show,pts_2d[i%4],pts_2d[(i+1)%4],BGR(128, 0, 128),3);
//    cv::circle(data.img_show, pts_2d[0], 3, BGR(255), 1);
//    cv::circle(data.img_show, pts_2d[1], 3, BGR(0, 0, 255), 1);
//    cv::circle(data.img_show, pts_2d[2], 3, BGR(0, 255, 0), 1);
//    cv::circle(data.img_show, pts_2d[3], 3, BGR(255, 0, 0), 1);

    data.pts_2d.clear();
    for (int i = 0; i < 4; i++)
        data.pts_2d.push_back(pts_2d[i]);
}

void TopHistory::update(Armor arm, cv::Point pt_roi) {
    flag = true;
    if (arm.id != 0)
        id_last = arm.id;
    type_last = arm.type_armor;
    cnt_update = 0;
    rect_last = arm.rect;
    rect_last.x += pt_roi.x;
    rect_last.y += pt_roi.y;
}

void TopHistory::update(cv::Size sz) {
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
