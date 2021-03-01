#include "configure.h"
#include "control.h"

class Max_Buff
{
public:
    Max_Buff() {}
    ~Max_Buff() {}
    //预处理
    void pretreat(Mat frame, int enemy_color);
    //找大神符中心的R
    void Looking_for_center();
    //计算坐标
    void Calculating_coordinates(int i);
    int average_color(Mat roi);
    Mat max_buff_roi(int i);
    float Distance(Point a, Point b)
    {
        return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    }
    double Cross(Point a, Point b, Point c)
    {
        return ((b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x));
    }
    int Getstate();

    int hit_subscript = 0;

    float buff_angle_ = 0;
    float diff_angle_ = 0;
    float last_angle = 0;
    float d_angle_ = 1; //0
    int find_cnt_ = 0;
    int direction_tmp_ = 0;

    Mat frame;
    Mat mask;
    Mat gray_img;
    Point2f R_center;
    Rect roi;
    vector<Point> armor_center;
    vector<RotatedRect> max_buff_rects;
    Point2f pre_center;
    float radius, small_radius;
    float forecast_angle = 0;
    float angle_cos, angle_sin, angle;
    bool R_success = false;
    bool choice_success = false;
    Point2f calculation_position[2];
    //蓝色th参数
    int blue_armor_gray_th = 132;
    int blue_armor_color_th = 44;
    //红色th参数
    int red_armor_gray_th = 70;
    int red_armor_color_th = 15;
};