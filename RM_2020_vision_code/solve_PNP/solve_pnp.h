#ifndef SOLVE_PNP_H
#define SOLVE_PNP_H
#include "configure.h"
#include "control/debug_control.h"

class RM_SolveAngle{

public:
    RM_SolveAngle();
    ~RM_SolveAngle();
    //调用解算函数
    void run_SolvePnp(RotatedRect & rect, float _W, float _H);
    void draw_Coordinate(Mat & input);
    float angle_x, angle_y, dist;
    
private:
    void vertex_Sort(RotatedRect & box);
    Mat camera_ptz(Mat & t);
    void get_Angle(const Mat & pos_in_ptz);
    
    //标定数据
    string file_path = CAMERA_PARAM_FILE;

    Mat cameraMatrix, distCoeffs;
    Mat rvec = Mat::zeros(3, 3, CV_64FC1);
    Mat tvec = Mat::zeros(3, 1, CV_64FC1);

    vector<Point3f> object_3d;
    vector<Point2f> target2d;

    const float ptz_camera_x = 0;
    const float ptz_camera_y = 0;
    const float ptz_camera_z = 0;
    const float barrel_ptz_offset_x = 0;
    const float barrel_ptz_offset_y = 0;
    const float overlap_dist = 0;

};

#endif // SOLVE_PNP_H
