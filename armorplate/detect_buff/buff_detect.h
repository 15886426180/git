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
    int hit_subscript = 0;
    Mat frame;
    Mat mask;
    Mat gray_img;
    Point R_center;

    vector<Point> armor_center;
    vector<RotatedRect> max_buff_rects;
    float radius;
    float forecast_angle = 20;
    float angle_cos, angle_sin, angle;
    bool R_success = false;
    bool choice_success = false;
    Point2f calculation_position;
    //蓝色th参数
    int blue_armor_gray_th = 132;
    int blue_armor_color_th = 44;
    //红色th参数
    int red_armor_gray_th = 70;
    int red_armor_color_th = 15;
};