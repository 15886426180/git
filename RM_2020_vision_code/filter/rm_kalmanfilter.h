#ifndef RM_KALMANFILTER_H
#define RM_KALMANFILTER_H

#include "configure.h"
#include "control/debug_control.h"

class RM_kalmanfilter
{
public:
    RM_kalmanfilter();
    ~RM_kalmanfilter();
    Point2f predict_point(double _t, Point _p);
    void reset();

    float anti_range = 1.5;

private:
    cv::KalmanFilter KF_;
    Mat measurement_matrix;

    double t = 0.005;
    float x = CAMERA_RESOLUTION_COLS*0.5;
    float y = CAMERA_RESOLUTION_ROWS*0.5;
};

#endif // RM_KALMANFILTER_H
