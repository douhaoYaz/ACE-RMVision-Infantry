#include "kalman.hpp"

cv::Mat KalmanFilterPredictor::predict(int stride) {
    if (stride <= 0)
        return cv::Mat();
    this->setPredictFilter();

    cv::Mat mat_predict;
    for (int i = 0; i < stride; i++)
        mat_predict = cvkf_pre.predict();
    return mat_predict;
}

Predict::Predict(){}

Predict::Predict(float Qp, float Qv, float Rp,float Rv,float deltaTime, float pre_deltaTime)
{
    Qvar = (cv::Mat_<float>(2,2)<< Qp, 0, 0, Qv); //设置系统误差
    delta = deltaTime;                            //设置时间间隔
    pre_delta = pre_deltaTime;                    //设置上一次时间间隔
    //预测值 (x'(k)): x(k)=A*x(k-1)+B*u(k)
    //状态值 (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k))
    //x = v * t + 1/2 * a *t2
    A = (cv::Mat_<float>(2,2) << 1, deltaTime, 0, 1); //状态转移矩阵 v = v0 + a * t;  a = a
    B = (cv::Mat_<float>(2,1) << 1/2*pow(deltaTime,2), deltaTime);
    //测量矩阵
    H = (cv::Mat_<float>(1,2) << 1, 0);
    H2 = (cv::Mat_<float>(2, 2) << 1, 1, 0, 1);
    //测量误差
    R = static_cast<double>(Rp);
    Rvar = (cv::Mat_<float>(2,2)<< Rp, 0, 0, Rv);

    //state
    po = (cv::Mat_<float>(2,2)<< 1000, 1000, 1000, 1000);//0.00623 don't worried, because it can update
    po_pre = po;
    x_pre = (cv::Mat_<float>(2,1)<< 0,0);
    kg = (cv::Mat_<float>(2,1)<< 0,0);
    xkf = (cv::Mat_<float>(2,1)<< 0,0);
    predict = 0;
    v = 0;
    last_v = 0;
}

//预测值 (x'(k)): x(k)=A*x(k-1)+B*u(k)
//状态值 (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k))
//x = v * t + 1/2 * a *t2
//状态转移矩阵 v = v0 + a * t;  a = a
//以下通用此模式

float Predict::run_position(float gim_angle)
{
    cv::Mat I = cv::Mat::eye(2,2,CV_32F);
    x_pre = A * xkf ;
    po_pre = A * po * A.t() + Qvar;
    kg = po_pre * H.t() * (H * po_pre * H.t() + R).inv();
    xkf = x_pre + kg* (static_cast<double>(gim_angle) - H * x_pre);
    v = xkf.at<float>(1,0);
    predict = xkf.at<float>(0,0);
    po = (I - kg * H) * po_pre;
    return predict;
}

float Predict::run_position(float gimbal_anlge, float v)
{
    cv::Mat Z = (cv::Mat_<float>(2,1)<< gimbal_anlge, v);
    cv::Mat I = cv::Mat::eye(2,2,CV_32F);
    x_pre = A * xkf ;
    po_pre = A * po * A.t() + Qvar;
    kg = po_pre * H2.t() * (H2 * po_pre * H2.t() + Rvar).inv();
    xkf = x_pre + kg* (Z - H2 * x_pre);
    v = xkf.at<float>(1,0);
    predict = xkf.at<float>(0,0) + v * pre_delta;
    po = (I - kg * H2) * po_pre;
    return predict;
}

float Predict::run_position(float gimbal_anlge, float v, float u)
{
    cv::Mat Z = (cv::Mat_<float>(2,1)<< gimbal_anlge, v);
    cv::Mat I = cv::Mat::eye(2,2,CV_32F);
    x_pre = A * xkf + B*u;
    po_pre = A * po * A.t() + Qvar;
    kg = po_pre * H2.t() * (H2 * po_pre * H2.t() + Rvar).inv();
    xkf = x_pre + kg* (Z - H2 * x_pre);
    v = xkf.at<float>(1,0);
    predict = xkf.at<float>(0,0) + v * pre_delta;
    po = (I - kg * H2) * po_pre;
    return predict;
}

/**
 *@brief 初始化，参数和opencv的一致
 *@param Qp          角度的系统误差
 *@param Qv          速度的系统误差
 *@param Rp          角度的测量误差
 *@param dta         时间的间隔
 *@param pre_dta     上一次时间的变化
 */

void Predict::setQRT(int Qp, int Qv, int Rp ,int dta, float pre_dta)
{
    Qvar = (cv::Mat_<float>(2,2)<< Qp*0.001, 0, 0, Qv*0.001);
    Rvar = Rp*0.001;
    delta = static_cast<float>(dta);
    pre_delta = pre_dta;
}

/**
 *@brief 清除卡尔曼滤波器的数据
 */


void Predict::ClearFilters()
{
    po = (cv::Mat_<float>(2,2)<< 1000, 0, 0, 1000);// don't worried, because it can update
    po_pre = po;
    x_pre = (cv::Mat_<float>(2,1)<< 0,0);
    kg = (cv::Mat_<float>(2,1)<< 0,0);
    xkf = (cv::Mat_<float>(2,1)<< 0,0);
    predict = 0;
}
