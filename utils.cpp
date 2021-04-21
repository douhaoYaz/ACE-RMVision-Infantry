#include "utils.h"

#ifdef GUI
#include "GUI/main_widget.h"
#endif

bool DestoryableWindow::create() {
    if(name=="" || flag)
        return false;
    cv::namedWindow(name);
    flag = true;
    return true;

}

bool DestoryableWindow::destory() {
    if (!flag)
        return false;
    cv::destroyWindow(name);
    flag = false;
    return true;
}

void DestoryableWindow::showImage(const cv::Mat& img) {
    cv::imshow(name, img);
    flag = true;
}

void Timer::AverageTimeCounter::update(double time) {
    q.push(time);
    tol += time;
    while (q.size() > cnt) {
        tol -= q.front();
        q.pop();
    }
}

void Tool::drawRotatedRectangle(cv::Mat& img, cv::RotatedRect rrect,
		const BGR color, int thickness, int type_line, int shift) {
    cv::Point2f pts[4];
    rrect.points(pts);
    for (int i = 0; i < 4; i++)
        cv::line(img, pts[i], pts[(i+1)%4], color, thickness, type_line, shift);
}

int Tool::calculateRoiAverageIntensity(cv::Mat& img, cv::Rect rect) {
    //判断
    if (rect.width < 1 || rect.height < 1 || rect.x < 1 || rect.y < 1
			|| rect.width + rect.x > img.cols || rect.height + rect.y > img.rows)
        return 255;
    //设置ROI 直接取灯条对区域  \\Range对象可以用来表示矩阵的多个连续的行或者多个连续的列
    cv::Mat roi = img(cv::Range(rect.y, rect.y + rect.height), cv::Range(rect.x, rect.x + rect.width));
    // static_cast 是温柔的，靠谱的，能做好的事情肯定能给你做，做不来的你别求他， 求也没用，别勉强
    //cv::mean(&cv::Mat)，该函数会得到Mat中各个通道的均值 \\此处直接取ROI区域的单灰度图像进行区域均值求解
    int average_intensity = static_cast<int>(cv::mean(roi).val[0]);
    //
    return average_intensity;
}

int Tool::calculateRoiAverageIntensity(cv::Mat& img) {
	int average_intensity = static_cast<int>(cv::mean(img).val[0]);
	return average_intensity;
}

float Tool::limitAngle(float angle, float angle_max) {
    if (angle > 0)
        return angle > angle_max ? angle_max : angle;
    else
        return angle < -angle_max ? -angle_max : angle;
}


float Tool::smoothAngleChange(float cur_angle, float factor, float th_angle) {
    //next=k*cur
    if(cur_angle >= 0)
		return cur_angle > th_angle ? cur_angle * factor : 0;
	else 
        return cur_angle < -th_angle ? cur_angle * factor : 0;
}

cv::Point2f Tool::LineFitting(std::vector<int> x, std::vector<float> y, int size)
{
    float x_mean = 0.0f;
    float y_mean = 0.0f;
    for(int i = 0; i < size; i++)
    {
        x_mean += x[i];
        y_mean += y[i];
    }
    x_mean /= size;
    y_mean /= size;

    float sumx2 = 0.0f;
    float sumxy = 0.0f;
    for(int i = 0; i < size; i++)
    {
        sumx2 += (x[i] - x_mean) * (x[i] - x_mean);
        sumxy += (y[i] - y_mean) * (x[i] - x_mean);
    }

    float k = sumxy / sumx2;
    float b = y_mean - k*x_mean;

    cv::Point2f point_coefficient;
    point_coefficient.x = k;
    point_coefficient.y = b;

    return point_coefficient;
}

#ifndef GUI
//int WindowPainter::end = 0;

//WindowPainter WindowPainter::painter = { };

//WindowPainter& WindowPainter::operator<<(const char* str) {
//    TempData tmp(str);
//    tmps.push_back(tmp);
//    return *this;
//}

//WindowPainter& WindowPainter::operator<<(const int &flag) {
//    if (flag == end) {
//        for (auto tmp: tmps) {
//            if (tmp.type == 0)
//                mvwaddstr(win_cmd, ++row_cmd, 0, (' '+std::string(tmp.data.str)+'\n').c_str());
//        }
//        tmps.clear();
//        box(win_cmd, 0, 0);
//        wrefresh(win_cmd);
//    }
//    return *this;
//}

//WindowPainter& WindowPainter::operator>>(char* str) {
//    waddch(win_input, ':');
//    wgetstr(win_input, str);
//    strs_last.emplace_back(str);
//    return *this;
//}
#endif
