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
void SolveP4p::Rotate_Point(RotatedRect rects, RotatedRect light_left, RotatedRect light_right)
{
    this->target2d.clear();
    int left_w = MAX(light_left.size.width, light_left.size.height) / 2;
    int left_h = MIN(light_left.size.width, light_left.size.height) / 2;

    int right_w = MAX(light_right.size.width, light_right.size.height) / 2;
    int right_h = MIN(light_right.size.width, light_right.size.height) / 2;

    //左上
    this->target2d.push_back(Point(light_left.center.x, light_left.center.y - left_h));
    //右上
    this->target2d.push_back(Point(light_right.center.x, light_right.center.y - right_h));
    //右下
    this->target2d.push_back(Point(light_right.center.x, light_right.center.y + left_h));
    //左下
    this->target2d.push_back(Point(light_left.center.x, light_left.center.y + left_h));

    //保存装甲板中心点
    this->armor_point = rects.center;
    //小装甲板
    if (rects.size.width - rects.size.height > rects.size.height / 2)
    {
        cout << "BIG" << endl;
        run_SolvePnp(BIG_ARMORPLATE_WIDTH, ARMORPLATE_HIGHT);
    }
    else
    {
        cout << "SMALL" << endl;
        run_SolvePnp(Small_ARMORPLATE_WIDTH, ARMORPLATE_HIGHT);
    }
}
/**
 * @brief P4p计算旋转向量
 * 
 * @param _W 计算物品宽度
 * @param _H 计算物品高度
 */
void SolveP4p::run_SolvePnp(float _W, float _H)
{
    float half_x = _W * 0.5;
    float half_y = _H * 0.5;

    object_3d.clear();
    object_3d.push_back(Point3f(-half_x, -half_y, 0));
    object_3d.push_back(Point3f(half_x, -half_y, 0));
    object_3d.push_back(Point3f(half_x, half_y, 0));
    object_3d.push_back(Point3f(-half_x, half_y, 0));

    solvePnP(this->object_3d, this->target2d, this->cameraMatrix, this->distCoeffs, this->rvec, this->tvec, false, SOLVEPNP_P3P);
    //将旋转向量变换成旋转矩阵
    Rodrigues(this->rvec, this->rotM);
    // Rodrigues(this->tvec, this->rotT);

    //将相机坐标系归0 Pw = -inverse(rotM)*tvec
    this->rvec_invert = this->rotM.inv(DECOMP_LU);
    this->world_point = this->rvec_invert * this->tvec;

    //120cm精度在-1cm 200cm精度在+2cm
    //保存计算距离
    this->dist = sqrt(pow(this->world_point.at<double>(0, 2), 2) + pow(this->world_point.at<double>(0, 1), 2) + pow(this->world_point.at<double>(0, 0), 2));
    //修改精度，增添补偿函数
    cout << this->dist << endl; //转换cm
    //计算旋转角
    // get_angle();
}
/**
 * @brief 计算旋转角 pitch roll yaw
 * 
 */
void SolveP4p::get_angle()
{
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

    calcu_depth();
}
/**
 * @brief 旋转角计算距离
 * 
 */
void SolveP4p::calcu_depth()
{
    int w = abs(IMG_COLS / 2 - armor_point.x);
    int h = abs(IMG_ROWS / 2 - armor_point.y);
    float d_s = CAMERA_HEIGHT / tan(this->theta_x + atan(h / cameraMatrix.at<double>(1, 1)));
    float k_s = w * sqrt(pow(d_s, 2) + pow(CAMERA_HEIGHT, 2)) / sqrt(pow(cameraMatrix.at<double>(0, 0), 2) + pow(h, 2));
    float d = (d_s / cos(atan(k_s / d_s))) * cos(atan(k_s / d_s) + this->theta_z);
    float k = (d_s / cos(atan(k_s / d_s))) * sin(atan(k_s / d_s) + this->theta_z);
    float depth = sqrt(pow(k, 2) + pow(d, 2));

    cout << depth << endl;
}