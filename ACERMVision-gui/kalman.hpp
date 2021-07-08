#ifndef KALMAN_HPP_INCLUDE
#define KALMAN_HPP_INCLUDE

#include <opencv2/opencv.hpp>

// 二阶ｋａｌｍａｎ滤波
class Predict
{
public:
    Predict();

    /**
     *@brief 初始化，参数和opencv的一致
     *@param Qp          角度的系统误差
     *@param Qv          速度的系统误差
     *@param Rp          角度的测量误差
     *@param Rv          速度的测量误差
     */
    Predict(float Qp, float Qv, float Rp,float Rv,float dta, float pre_dta);

    /**
     *@brief 清除卡尔曼滤波器的数据
     */
    void ClearFilters(void);

    /**
     *@brief 初始化，参数和opencv的一致
     *@param Qp          角度的系统误差
     *@param Qv          速度的系统误差
     *@param Rp          角度的测量误差
     *@param dta         时间的间隔
     *@param pre_dta     上一次时间的变化
     */
    void setQRT(int Qp, int Qv, int Rp , int dta, float pre_dta);

    /**
     *@brief 设置时间的间隔
     *@param dta         时间的间隔
     * */
    void setdelta(float t){
        delta = t;
    }

    /**
     *@brief 卡尔曼运算
     *@param gim_angle        角度
     *@param v                速度
     *@param u                加速度
     * */
    float run_position(float gim_angle);
    float run_position(float gimbal_anlge, float v);
    float run_position(float gimbal_anlge, float v, float u);

    /**
     *@brief 获得角速度
     *@param v                速度
     * */
    float getSpeed()
    {
        return v;
    }

    bool exit_flag;
private:
    cv::Mat Qvar;   // 两个观测变量系统误差
    cv::Mat R;      // 单观测变量方差
    cv::Mat Rvar;   // 两个观测变量协方差
    cv::Mat A;      //角度与速度的状态方程
    cv::Mat B;      //加速度的状态方程
    cv::Mat H;      // 单变量观测矩阵
    cv::Mat H2;     // 两变量观测矩阵
    float delta;    //变量deltaT：时间的变化
    float pre_delta;//变量pre_deltaT：上一次时间的变化

    //state
    cv::Mat po;         //最小均方误差
    cv::Mat po_pre;     //修正的最小均方误差
    cv::Mat x_pre;    //最新的测量角度,即预测值
    cv::Mat kg;       //卡尔曼增益
    cv::Mat xkf;      //状态值,即根据预测值计算出当前的值，会受到系统误差 Q，测量误差 R影响
    float v;          //速度
    float last_v;      //上一次速度
    float predict;  //最终的角度预测的值
};

/**
 *@brief 通用卡尔曼预测器
 *@加了带步长的预测
 */
class KalmanFilterPredictor {
public:
    cv::KalmanFilter cvkf_cur;  //opencv的卡尔曼滤波器

    /**
     *@brief 初始化，参数和opencv的一致
     *@param cnt_state   状态参数数量
     *@param cnt_measure 测量参数数量
     *@param cnt_ctrl    控制参数数量
     *@param dtype       数据类型
     */
    KalmanFilterPredictor(int  cnt_state, int cnt_measure, int cnt_ctrl, int dtype=CV_32F):
            cvkf_cur(cnt_state, cnt_measure, cnt_ctrl, dtype),
            cvkf_pre(cnt_state, cnt_measure, cnt_ctrl, dtype) {
    }

    /**
     *@brief 带步长的预测，实际参数不会更新
     *@输入:
     *  @stride 预测步长，即当前数据预测n次的结果
     *@输出:
     *  @return 得到的状态向量
     */
    cv::Mat predict(int stride=1);

    /**
     *@brief 更新，实际即原opencv卡尔曼的KalmanFilter.predict()
     *@输入:
     *  @mat_ctrl 控制矩阵
     *@输出:
     *  @return 更新后的状态向量
     */
    cv::Mat update(const cv::Mat& mat_ctrl=cv::Mat()) {
        return cvkf_cur.predict(mat_ctrl);
    }

    /**
     *@brief 修正，实际即原opencv卡尔曼的KalmanFilter.correct()
     *@输入:
     *  @mat_measurement 测量矩阵
     *@输出:
     *  @return 修正后的状态值
     */
    const cv::Mat& correct(const cv::Mat& mat_measurement) {
        return cvkf_cur.correct(mat_measurement);
    }

protected:

    cv::KalmanFilter cvkf_pre;  //用于步长预测的opencv卡尔曼滤波器

    /**
     * @brief 同步预测用的滤波器
     */
    void setPredictFilter() {
        cvkf_pre.transitionMatrix = cvkf_cur.transitionMatrix.clone();
        cvkf_pre.measurementMatrix = cvkf_cur.measurementMatrix.clone();
        cvkf_pre.processNoiseCov = cvkf_cur.processNoiseCov.clone();
        cvkf_pre.errorCovPost = cvkf_cur.errorCovPost.clone();
        cvkf_pre.statePost = cvkf_cur.statePost.clone();
        cvkf_pre.controlMatrix = cvkf_cur.controlMatrix.clone();
    }
};

/**
 *@brief 用于角度解算的卡尔曼滤波器
 */
class AngleKalmanFilter:
        public KalmanFilterPredictor {
public:

    cv::Mat mat_measurement;    //测量值
    cv::Mat angles_kalman;      //角度滤波值

    /**
     *@brief 用角度初始化
     *@param x,y 角度的x, y值
     */
    AngleKalmanFilter(int x, int y):
            KalmanFilterPredictor(4, 2, 0) {
        mat_measurement = cv::Mat::zeros(2, 1, CV_32F);     // (x, y)
        //x
        cvkf_cur.transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 1, 0,
                                                              0, 1, 0, 1,
                                                              0, 0, 1, 0,
                                                              0, 0, 0, 1);
        cv::setIdentity(cvkf_cur.measurementMatrix, cv::Scalar::all(1));        //H
        cv::setIdentity(cvkf_cur.processNoiseCov, cv::Scalar::all(140000));     //Q
        cv::setIdentity(cvkf_cur.measurementNoiseCov, cv::Scalar::all(1e3));    //R
        cv::setIdentity(cvkf_cur.errorCovPost, cv::Scalar::all(1));

        this->setPredictFilter();

        cvkf_cur.statePost = (cv::Mat_<float>(4, 1) << x, y, 0, 0); //X
    }


    /**
     *@brief 预测并更新
     *@输入:
     *  @x, y 角度解算得出的角度(测量值)
     *@输出:
     *@return 预测向量
     */
    cv::Mat run(float x, float y) {
        cv::Mat mat_prediction = cvkf_cur.predict();

        mat_measurement.at<float>(0, 0) = x;
        mat_measurement.at<float>(1, 0) = y;

        cvkf_cur.correct(mat_measurement);

        return mat_prediction;
    }
};

#endif // KALMAN_HPP_INCLUDE

