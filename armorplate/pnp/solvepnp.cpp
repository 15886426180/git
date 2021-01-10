#include "solvepnp.h"

//下次修改内容
//70行增加辅助拟合曲线

/**
 * @brief 转换装甲板2d位置
 * 
 * @param rects 装甲板旋转举证
 * @param light_left 左灯条旋转矩阵
 * @param light_right 右灯条旋转矩阵
 */
float SolveP4p::arrange_Point(RotatedRect rects, float _w, float _h)
{
    this->target2d.clear();
    //左上
    this->target2d.push_back(Point(rects.center.x - (rects.size.width / 2), rects.center.y - (rects.size.height / 2)));
    //右上
    this->target2d.push_back(Point(rects.center.x + (rects.size.width / 2), rects.center.y - (rects.size.height / 2)));
    //右下
    this->target2d.push_back(Point(rects.center.x + (rects.size.width / 2), rects.center.y + (rects.size.height / 2)));
    //左下
    this->target2d.push_back(Point(rects.center.x - (rects.size.width / 2), rects.center.y + (rects.size.height / 2)));

    return this->run_SolvePnp(_w, _h);
}
/**
 * @brief 
 * 
 * @param rects 
 * @param _w 
 * @param _h 
 * @return float 
 */
float SolveP4p::armor_Point(RotatedRect left_light, float _w, float _h)
{
    this->target2d.clear();
    float light_w = MIN(left_light.size.width, left_light.size.height);
    float light_h = MAX(left_light.size.width, left_light.size.height);

    if (_w / _h < 2.5)
    {
        //左上
        this->target2d.push_back(Point(left_light.center.x + (light_w / 2), left_light.center.y - (light_h / 2)));
        //右上
        this->target2d.push_back(Point(left_light.center.x + (light_h * 2.27), left_light.center.y - (light_h / 2)));
        //右下
        this->target2d.push_back(Point(left_light.center.x + (light_h * 2.27), left_light.center.y + (light_h / 2)));
        //左下
        this->target2d.push_back(Point(left_light.center.x + (light_w / 2), left_light.center.y + (light_h / 2)));
        cout<<"SMALL"<<endl;
    }
    else
    {
        //左上
        this->target2d.push_back(Point(left_light.center.x - (light_w / 2), left_light.center.y - (light_h / 2)));
        //右上
        this->target2d.push_back(Point(left_light.center.x + (light_h * 3.85), left_light.center.y - (light_h / 2)));
        //右下
        this->target2d.push_back(Point(left_light.center.x + (light_h * 3.85), left_light.center.y + (light_h / 2)));
        //左下
        this->target2d.push_back(Point(left_light.center.x - (light_w / 2), left_light.center.y + (light_h / 2)));
        cout<<"BIG"<<endl;
    }

    //计算pnp
    return this->run_SolvePnp(_w, _h);
}
/**
 * @brief P4p计算旋转向量
 * 
 * @param _W 计算物品宽度
 * @param _H 计算物品高度
 */
float SolveP4p::run_SolvePnp(float _W, float _H)
{
    float half_x = _W * 0.5;
    float half_y = _H * 0.5;

    object_3d.clear();
    object_3d.push_back(Point3f(-half_x, -half_y, 0));
    object_3d.push_back(Point3f(half_x, -half_y, 0));
    object_3d.push_back(Point3f(half_x, half_y, 0));
    object_3d.push_back(Point3f(-half_x, half_y, 0));

    solvePnP(this->object_3d, this->target2d, this->cameraMatrix, this->distCoeffs, this->rvec, this->tvec, false, SOLVEPNP_P3P);
    Mat ptz = camera_ptz(tvec);//云台Pitch轴当前角度
    return sqrt(pow(ptz.at<double>(0, 2), 2)+ pow(ptz.at<double>(0, 0), 2));
    //将旋转向量变换成旋转矩阵
    // Rodrigues(this->rvec, this->rotM);
    // cout<<rotM<<endl;
    
    // Rodrigues(this->tvec, this->rotT);

    //将相机坐标系归0 Pw = -inverse(rotM)*tvec
    // this->rvec_invert = this->rotM.inv(DECOMP_LU);
    // this->world_point = this->rvec_invert * this->tvec;
    // cout<<world_point<<endl;
    // this->dist = sqrt(pow(this->world_point.at<double>(0, 2), 2)+ pow(this->world_point.at<double>(0, 0), 2));

    // return get_angle();

    // return this->dist;
    //计算旋转角
    
}
/**
 * @brief 计算旋转角 pitch roll yaw
 * 
 */
float SolveP4p::get_angle()
{
    cout<<this->rotM.at<double>(2, 1);
    cout<<this->rotM.at<double>(2, 2);
    this->theta_x = atan2(this->rotM.at<double>(2, 1), this->rotM.at<double>(2, 2));
    this->theta_y = atan2(-this->rotM.at<double>(2, 0),
                          sqrt(this->rotM.at<double>(2, 1) * this->rotM.at<double>(2, 1) + this->rotM.at<double>(2, 2) * this->rotM.at<double>(2, 2)));
    this->theta_z = atan2(this->rotM.at<double>(1, 0), this->rotM.at<double>(0, 0));

    //将弧度转化为角度
    this->theta_x = this->theta_x * (180 / PI); //pitch
    this->theta_y = this->theta_y * (180 / PI); //roll
    this->theta_z = this->theta_z * (180 / PI); //yaw

    cout << this->theta_x << endl;
    cout << this->theta_y << endl;
    cout << this->theta_z << endl;

    return calcu_depth();
}
/**
 * @brief 旋转角计算距离
 * 
 */
float SolveP4p::calcu_depth()
{
    int w = abs(CAMERA_RESOLUTION_COLS / 2 - armor_point.x);
    int h = abs(CAMERA_RESOLUTION_ROWS / 2 - armor_point.y);
    float d_s = CAMERA_HEIGHT / tan(this->theta_x + atan(h / cameraMatrix.at<double>(1, 1)));
    float k_s = w * sqrt(pow(d_s, 2) + pow(CAMERA_HEIGHT, 2)) / sqrt(pow(cameraMatrix.at<double>(0, 0), 2) + pow(h, 2));
    float d = (d_s / cos(atan(k_s / d_s))) * cos(atan(k_s / d_s) + this->theta_z);
    float k = (d_s / cos(atan(k_s / d_s))) * sin(atan(k_s / d_s) + this->theta_z);
    float depth = sqrt(pow(k, 2) + pow(d, 2));

    // cout << depth << endl;
    return depth;
}

float SolveP4p::Pinhole_imaging(RotatedRect rects, float _h)
{
    float light_h = MAX(rects.size.height, rects.size.width);
    float dist = _h * this->cameraMatrix.at<double>(1, 1) / light_h;
    // cout << dist << endl;
    return dist;
}


Mat SolveP4p::camera_ptz(Mat & t)
{
    //设相机坐r_camera_ptz标系绕X轴你是逆时针旋转θ后与云台坐标系的各个轴向平行
    double theta = 0;/*-atan(static_cast<double>(ptz_camera_y + barrel_ptz_offset_y))/static_cast<double>(overlap_dist);*/
    double r_data[] = {1,0,0,0,cos(theta),sin(theta),0,-sin(theta),cos(theta)};
    //设相机坐标系的原点在云台坐标系中的坐标为(x0,y0,z0)
    double t_data[] = {static_cast<double>(ptz_camera_x),static_cast<double>(ptz_camera_y),static_cast<double>(ptz_camera_z)};
    Mat r_camera_ptz(3,3,CV_64FC1,r_data);
    Mat t_camera_ptz(3,1,CV_64FC1,t_data);

    Mat position_in_ptz = r_camera_ptz * t - t_camera_ptz;
    // cout << position_in_ptz << endl;
    return position_in_ptz;
}