#ifndef KALMAN_HPP_INCLUDE
#define KALMAN_HPP_INCLUDE

#include <opencv2/opencv.hpp>

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

