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
