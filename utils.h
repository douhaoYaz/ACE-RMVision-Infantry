#ifndef UTILS_H_INCLUDE
#define UTILS_H_INCLUDE

#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <cxxabi.h>

#ifdef GUI
#include <QString>
#endif

#define registerReflector(name_class, name_member, type) {                      \
    size_t offset = reinterpret_cast<size_t>(&((name_class*)0)->name_member);   \
    Reflector<name_class>::registerMember(#name_member, type, offset);          \
}

typedef cv::Scalar BGR;
typedef cv::Scalar RGB;
typedef cv::Scalar HSV;


/**
 *@brief 字体是颜色
 *@FONT_+:
 *  @NONE       0，不做设置
 *  @BLACK      1，黑色
 *	@RED        2，红色
 *	@GREEN      3，绿色
 *	@YELLOW     4，黄色
 *	@BLUE       5，蓝色
 *	@DARK_GREEN 6，深绿
 *	@WHITE      7，白（ubuntu默认）
 */
enum FontColor {
	FONT_NONE       = 0,
	FONT_BLACK      = 1,
	FONT_RED        = 2,
	FONT_GREEN      = 3,
	FONT_YELLOW     = 4,
	FONT_BLUE	    = 5,
	FONT_DARK_GREEN = 6,
	FONT_WHITE      = 7
};

enum DataType {
	DATA_INTEGER = 0,
	DATA_DOUBLE  = 1,
	DATA_BOOLEAN = 2,
	DATA_STRING  = 3
};

/**
 *@brief 基于opencv highgui的可销毁窗口
 */
class DestoryableWindow {
public:
    std::string name;   //窗口名字
    bool flag;          //是否被使用标志

    DestoryableWindow() { }

    /**
     *@brief 构造函数
     *@param name_win  窗口名字
     *@param is_create 是否要创建窗口
     */
    DestoryableWindow(std::string name_win, bool is_create=false)
        :name(name_win), flag(is_create) { }

    /**
     *@brief 创建窗口
     *@return 是否创建成功
     */
    bool create();

    /**
     *@brief 销毁窗口
     *@输出:
     * @return 是否销毁成功
     */
    bool destory();

    /**
     *@brief 显示图片，若flag为false会自动创建窗口
     *@param img 要显示的图片
     */
    void showImage(const cv::Mat& img);

};

/**
 *@brief 基于opencv的简易计时器
 */
class Timer {
public:
    /**
     *@brief 开始计时
     */
    void start() {
        t_begin = cv::getTickCount();
    }

    /**
     *@brief 计算平均时间
     *@param cnt 历史100次的时间
     *@return
     */
    double calculateAverageTime() {
        return counter.tol / counter.q.size();
    }

    /**
     *@brief 结束本次计时
     *@return 时间，单位ms
     */
    double end() {
        double t_end = cv::getTickCount();
        double time = (t_end - t_begin) / cv::getTickFrequency() * 1000;
        counter.update(time);
        return time;
    }

private:
    double t_begin;     //开始的时间
    class AverageTimeCounter {
    public:
        uint cnt;                //最大数量
        double tol;             //总数
        std::queue<double> q;   //时间队列

        /**
         *@param count 最大数量
         */
        AverageTimeCounter(uint count=100): cnt(count) { }

        /**
         *@brief 更新队列和总数
         *@param 要更新的时间
         */
        void update(double time);
    }
    counter;   //计时器内部类对象
};

template <typename _Type>
class Reflector {
public:

    enum Type {
        CHAR   = 0,
        BOOL   = 1,
        INT    = 2,
        FLOAT  = 3,
        DOUBLE = 4,
        STRING = 5,
        OTHER  = 6
    };

    struct ClassInfomation {

        std::string name;
        int type;
        size_t offset;

        ClassInfomation(const std::string& name_member, int type_member, size_t offset_member):
                name(name_member), type(type_member), offset(offset_member) { }

        bool operator<(const ClassInfomation& info) const {
            return offset < info.offset;
        }

        bool operator==(const ClassInfomation& info) const {
            return name == info.name;
        }

        bool operator==(const std::string& name_info) const {
            return name == name_info;
        }

    };

public:

    static void registerMember(const std::string& name_member, int type, size_t offset) {
        auto& infos = getInfomations();
        ClassInfomation info(name_member, type, offset);
        infos.emplace(name_member, info);
    }

    static std::vector<ClassInfomation> getVector() {
        auto& infos = getInfomations();
        std::vector<ClassInfomation> v;
        std::map<int, int> m;

        for (auto var: infos)
            v.push_back(var.second);

        return std::move(v);
    }

    static void* get(_Type* p, const std::string& name) {
        auto& infos = getInfomations();
        auto iter = infos.find(name);
        if (iter == infos.end())
            return nullptr;
        size_t offset = iter->second.offset;
        return p+offset;
    }

    template <typename _Data>
    static void set(_Type* p, const std::string& name, _Data data) {
        auto& infos = getInfomations();
        auto iter = infos.find(name);
        if (iter == infos.end())
            return;
        std::map<int, int> m;
        size_t offset = iter->second.offset;
        int type = iter->second.type;
        switch (type) {
        case INT:
            *(int*)(p+offset) = data;
            break;
        case FLOAT:
            *(float*)(p+offset) = data;
            break;
        case CHAR:
            *(char*)(p+offset) = data;
            break;
        case BOOL:
            *(bool*)(p+offset) = data;
            break;
        case STRING:
            *(std::string*)(p+offset) = data;
            break;
        case OTHER:
        default:
            *(_Data*)(p+offset) = data;
            break;
        }
    }

    static std::string getName() {
        static std::string name;
        if (name.empty())
            name = abi::__cxa_demangle(typeid(_Type).name(), nullptr, nullptr, nullptr);
        return name;
    }


protected:
    static std::map<std::string, ClassInfomation>& getInfomations() {
        static std::map<std::string, ClassInfomation> infos;
        return infos;
    }

    Reflector<_Type>()=delete;

};

/**
 *@brief 散装的工具函数，静态方法形式
 */
class Tool {
public:
     //1、绘制旋转矩形
    /**
     *@brief 绘制opencv旋转矩形
     *@param img   要绘制的原图
     *@param rrect 要绘制的旋转矩形
     *@param ..    其余与cv::line基本一致
     */
    static void drawRotatedRectangle(cv::Mat& img, cv::RotatedRect rrect, const BGR color,
            int thickness=1, int type_line=cv::LINE_8, int shift=0);

    //2、计算距离
    /**
     *@brief 计算直线距离
     *@param pt1, pt2 要计算的两个点
     *@return 距离
     */
    template<class T1, class T2>
    static double calculateDistance(cv::Point_<T1> pt1, cv::Point_<T2> pt2) {
        return sqrt((pt1.x - pt2.x)*(pt1.x - pt2.x)+((pt1.y - pt2.y)*(pt1.y - pt2.y)));
    }

    //3、把弧度转化为角度
    /**
     *@brief 把弧度转化为角度
     *@param rad 要转化的弧度
     *@return 转化出的角度
     */
    static double radian2Angle(double rad) {
        return rad * 180 / CV_PI;
    }

    //4、把角度转化为弧度
    /**
     *@brief 把角度转化为弧度
     *@param angle 要转化的角度
     *@return 转化出的弧度
     */
    static double angle2Radian(double angle) {
        return angle / 180. * CV_PI;
    }

    //5、在点处绘制十字标记
    /**
     *@brief 在点处绘制十字标记
     *@param img       要绘制的原图
     *@param pt        十字原点
     *@param len_cross 十字一侧的线长
     *@param ...       其余与cv::line基本一致
     */
    static void drawCross(cv::Mat& img, cv::Point pt, int len_cross, const BGR color,
            int thickness=1, int type_line=cv::LINE_8, int shift=0) {
        cv::line(img, cv::Point2f(pt.x, pt.y-len_cross),  \
                cv::Point2f(pt.x, pt.y+len_cross),
                color, thickness, type_line, shift);
        cv::line(img, cv::Point2f(pt.x-len_cross, pt.y),  \
                cv::Point2f(pt.x+len_cross, pt.y),
                color, thickness, type_line, shift);
    }

    //6、计算图像ROI区域强度
    /**
     *@brief 计算图像ROI区域强度
     *@param img      要计算的原图
     *@param rect_roi 要计算的矩形区域
     *@return 区域强度
     */
    static int calculateRoiAverageIntensity(cv::Mat& img, cv::Rect rect_roi);

    //7、计算图像区域强度
	/**
	 *@brief 计算图像区域强度
	 *@param img  要计算的图像
	 *@return 区域强度
	 */
	static int calculateRoiAverageIntensity(cv::Mat& img);

    //8、限制角度
    /**
         *@brief 限制角度
         *@param angle      要限制的角度
         *@param angle_max  最大角度
         *@return 角度值
         */
	static float limitAngle(float angle, float angle_max);

    //9、平滑角度变化(防止角度数据跳动过大)
	/**
	 *@brief 平滑角度变化(防止角度数据跳动过大)
	 *@param angle     要限制的角度
	 *@param angle_max 最大角度
	 *@return 角度值
	 */
	static float smoothAngleChange(float cur_angle, float factor, float th_angle);

    //10、最小二乘法线性拟合
    /**
     *@brief 最小二乘法线性拟合
     *@param angle     要限制的角度
     *@param angle_max 最大角度
     *@param angle_max 最大角度
     *@return 角度值
     */
    static cv::Point2f LineFitting(std::vector<int> x, std::vector<float> y, int size);

    template <typename _Type>
    static std::string toString(_Type val) {
        std::stringstream ss;
        ss << val;
        std::string str;
        ss >> str;
        return str;
    }
};

template <>
inline std::string Tool::toString<const char*>(const char* val) {
    return std::string(val);
}

template <>
inline std::string Tool::toString<const std::string&>(const std::string& val) {
    return val;
}

#ifdef GUI

template <>
inline std::string Tool::toString<const QString&>(const QString& val) {
    return val.toStdString();
}

#else

#include <curses.h>

class WindowPainter {
public:

    static WindowPainter painter;
    static int end;

    WindowPainter(const WindowPainter&)=delete;
    ~WindowPainter() {
        delwin(win_param);
        delwin(win_cmd);
        delwin(win_input);
    }

    WindowPainter& operator<<(const char* str);
    WindowPainter& operator<<(const std::string& str) {
        return this->operator<<(str.c_str());
    }
    WindowPainter& operator<<(const int& flag);

    static WindowPainter getPainter();
    static void setPainter(const WindowPainter &value);

    WindowPainter& operator>>(char* str);
    WindowPainter& operator>>(std::string& str) {
        char s[32];
        this->operator>>(s);
        str = s;
        return *this;
    }
    template<typename... _Arg>
    void drawBlock(int w, _Arg... args) {
        if (w == 0)
            w = this->w-2;
        wmove(win_param, ++row_param, 1);
        for (int i = 0; i < w; i++)
            waddch(win_param, '-');
        //DrawBlockTemplate<0, sizeof...(args), _Arg...>(*this, w, args...);
        _drawBlock(w, args...);
        wmove(win_param, ++row_param, 1);
        for (int i = 0; i < w; i++)
            waddch(win_param, '-');
    }

    void freshParameterWindow() {
        wrefresh(win_param);
    }


private:
    struct TempData {
        TempData(int flag) {
            type = 1;
            data.flag = flag;
        }

        TempData(const char* str) {
            type = 0;
            strcpy(data.str, str);
        }
        int type;
        union Data {
            char str[64];
            int flag;
        } data;
    };

private:
    WINDOW* win_param;
    WINDOW* win_cmd;
    WINDOW* win_input;
    std::vector<std::string> strs_last;
    std::vector<TempData> tmps;
    int row_cmd;
    int row_param;
    int w;


    WindowPainter(): row_cmd(0), row_param(0) {
        initscr();
        curs_set(0);

        int h = LINES-1;
        w = (COLS-1) / 2;

        win_param = newwin(h-2, w, 0, 0);
        box(win_param, 0, 0);
        scrollok(win_param, false);
        clearok(win_param, true);
        leaveok(win_param, false);
        wrefresh(win_param);

        win_cmd = newwin(h, w, 0, w);
        box(win_cmd, 0, 0);
        scrollok(win_cmd, true);
        clearok(win_cmd, true);
        leaveok(win_cmd, false);
        wrefresh(win_cmd);

        win_input = newwin(1, w, h-2, 0);
        scrollok(win_input, true);
        clearok(win_input, true);
        leaveok(win_param, true);
        wrefresh(win_input);

    }

    template<typename... _Arg>
    void _drawBlock(int w, unsigned long value, _Arg... args) {
        wmove(win_param, ++row_param, 1);
        waddch(win_param, '|');
        waddch(win_param, value);
        mvwaddch(win_param, row_param, w, '|');
        _drawBlock(w, args...);
    }

    template<typename... _Arg>
    inline void _drawBlock(int w, const char* value, _Arg... args) {
        wmove(win_param, ++row_param, 1);
        waddch(win_param, '|');
        waddstr(win_param, value);
        mvwaddch(win_param, row_param, w, '|');
        _drawBlock(w, args...);
    }

    void _drawBlock(int) { }
};
#endif

#endif // UTILS_H_INCLUDE
