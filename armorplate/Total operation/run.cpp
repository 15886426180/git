#include "run.h"

WorKing::WorKing() : capture(USB_CAPTURE_DEFULT), cap(ISOPEN_INDUSTRY_CAPTURE) {}

/**
 * @brief 运行函数
 * 
 */
void WorKing::Run()
{

    for (;;)
    {
#if FPS_SHOW == 1
        double t = (double)cv::getTickCount(); //开始计时
#endif                                         // #endif
        if (cap.isindustryimgInput())
        {
            frame = cvarrToMat(cap.iplImage, true);
        }
        else
        {
            capture >> frame;
        }

        // armor.success_armor = false;
        Mat src_img;
        resize(frame, src_img, Size(CAMERA_RESOLUTION_COLS, CAMERA_RESOLUTION_ROWS));
#if ROI_IMG == 1
        //ROI
        if (armor.lost_success_armor)
        {
            src_img = src_img(armor.armor_roi);
        }
        else
        {
            src_img = src_img; //直接赋值
        }
#endif
        //图像预处理
        img.pretreat(src_img, COLOR);
        //找到灯条后

        if (rgb.find_light(img.mask))
        {
            //装甲板大于等于1块时
            if (rgb.armor_fitting(img.gray_img))
            {
                armor.rect_num = rgb.optimal_armor() / 2;
#if CALL_DEPTH_INFORMATION == 1
                float depth = 0;
                if (rgb.armor[armor.rect_num].size.width / rgb.armor[armor.rect_num].size.height > 2)
                {
                    depth = pnp.arrange_Point(rgb.light[rgb.light_subscript[armor.rect_num * 2]], rgb.light[rgb.light_subscript[armor.rect_num * 2 + 1]], BIG_ARMORPLATE_WIDITH, ARMORPLATE_HIGHT);
                }
                else
                {
                    depth = pnp.arrange_Point(rgb.light[rgb.light_subscript[armor.rect_num * 2]], rgb.light[rgb.light_subscript[armor.rect_num * 2 + 1]], SMALL_ARMORPLATE_WIDTH, ARMORPLATE_HIGHT);
                }
                armor.depth = depth;
#endif
#if ROI_IMG == 1
                //ROI
                if (armor.lost_success_armor)
                {
                    int point_x = rgb.armor[armor.rect_num].center.x - (rgb.armor[armor.rect_num].size.width * 1.8) + armor.armor_roi.x;
                    int point_y = rgb.armor[armor.rect_num].center.y - (rgb.armor[armor.rect_num].size.height * 1.8) + armor.armor_roi.y;
                    int width = rgb.armor[armor.rect_num].size.width * 4;
                    int height = rgb.armor[armor.rect_num].size.height * 4;
                    if (point_x + width >= CAMERA_RESOLUTION_COLS - 1)
                    {
                        width = CAMERA_RESOLUTION_COLS - point_x - 1;
                    }
                    if (point_y + height >= CAMERA_RESOLUTION_ROWS - 1)
                    {
                        height = CAMERA_RESOLUTION_ROWS - point_y - 1;
                    }
                    if (point_x < 0)
                    {
                        point_x = 0;
                    }
                    if (point_y < 0)
                    {
                        point_y = 0;
                    }
                    armor.armor_roi = Rect(
                        point_x,
                        point_y,
                        width,
                        height);
                    rgb.armor[armor.rect_num] = RotatedRect(
                        Point(rgb.armor[armor.rect_num].center.x + armor.armor_roi.x, rgb.armor[armor.rect_num].center.y + (armor.armor_roi.y)),
                        rgb.armor[armor.rect_num].size,
                        rgb.armor[armor.rect_num].angle);
                }
                else
                {
                    int point_x = rgb.armor[armor.rect_num].center.x - (rgb.armor[armor.rect_num].size.width * 2);
                    int point_y = rgb.armor[armor.rect_num].center.y - (rgb.armor[armor.rect_num].size.height * 2);
                    int width = rgb.armor[armor.rect_num].size.width * 4;
                    int height = rgb.armor[armor.rect_num].size.height * 4;
                    if (point_x + width >= CAMERA_RESOLUTION_COLS - 1)
                    {
                        width = CAMERA_RESOLUTION_COLS - point_x - 1;
                    }
                    if (point_y + height >= CAMERA_RESOLUTION_ROWS - 1)
                    {
                        height = CAMERA_RESOLUTION_ROWS - point_y - 1;
                    }
                    if (point_x < 0)
                    {
                        point_x = 0;
                    }
                    if (point_y < 0)
                    {
                        point_y = 0;
                    }
                    armor.armor_roi = Rect(
                        point_x,
                        point_y,
                        width,
                        height);
                }
#endif
                armor.success_armor = true;                     //识别正确
                armor.lost_success_armor = armor.success_armor; //保存上一帧的参数
                armor.yaw = rgb.armor[armor.rect_num].center.x;
                armor.pitch = rgb.armor[armor.rect_num].center.y;
                armor.is_shooting = 1;
                armor.data_type = 1;
                //绘图
                rectangle(src_img, rgb.armor[armor.rect_num].boundingRect(), Scalar(0, 255, 0), 3, 8);
                rectangle(src_img, armor.armor_roi, Scalar(255, 200, 0), 3, 8);
                rgb.lost_armor = rgb.armor[armor.rect_num];
            }
            else //丢失目标
            {
                armor.success_armor = false;
                armor.lost_success_armor = armor.success_armor;
            }
        }
#if CALL_SERIALPORT == 1
        serial.RMserialWrite(armor._yaw, armor.yaw, armor._pitch, armor.pitch, armor.depth, armor.data_type, armor.is_shooting);
#endif

        rgb.eliminate();
        armor.eliminate();
#if FPS_SHOW == 1
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency(); //结束计时
        int fps = int(1.0 / t);                                        //转换为帧率
        cout << "FPS: " << fps << endl;                                //输出帧率
#endif
#if CALL_KALMAN == 1
        Point ddd = kalman.point_Predict(1 / t, Point(armor.armor_roi.x + (armor.armor_roi.width / 2), armor.armor_roi.y + (armor.armor_roi.height / 2)));
        circle(src_img, ddd, 3, Scalar(0, 255, 255), 3, 8);
#endif
        imshow("frame", src_img);
        // serial.RMserialWrite();
        cap.cameraReleasebuff();
        char c = waitKey(1);
        if (c == 27) //"Esc"-退出
        {
            break;
        }
    }
}

WorKing::~WorKing() {}

void WorKing::ddd()
{
    int n = 0;
    for (;;)
    {
        if (cap.isindustryimgInput())
        {
            frame = cvarrToMat(cap.iplImage, true);
        }
        imshow("frame", frame);
        char c = waitKey(1);
        if (c == 27) //"Esc"-退出
        {
            break;
        }
        else if (c == 'q' || c == 'k')
        {
            int num = 0;
            while (true)
            {
                char *cstr = new char[120];
                sprintf(cstr, "%s%d%s", "/home/sms/rm步兵蓝车数据集/", n, ".jpg");
                imwrite(cstr, frame);
                n++;
                num++;
                if (num >= 10)
                {
                    break;
                }
            }
        }
        cap.cameraReleasebuff();
        cout << "第" << n << "张" << endl;
        // waitKey(2000);
    }
}

void WorKing::Run_MAX_Talisman()
{
    for (;;)
    {
#if FPS_SHOW == 1
        double t = (double)cv::getTickCount(); //开始计时
#endif                                         // #endif
        if (cap.isindustryimgInput())
        {
            frame = cvarrToMat(cap.iplImage, true);
        }
        else
        {
            capture >> frame;
        }
        // resize(frame, frame, Size(640, 400));
        buff.pretreat(frame, COLOR);
        buff.Looking_for_center();
        imshow("frame", frame);

        buff.armor_center.clear();
        buff.max_buff_rects.clear();
        serial.RMserialWrite(buff._yaw, buff.yaw, buff._pitch, buff.pitch, buff.depth, buff.data_type, buff.is_shooting);
        // cout << pnp.max_buff_Point(buff.max_buff_rects[buff.hit_subscript]) << endl;

#if FPS_SHOW == 1
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency(); //结束计时
        int fps = int(1.0 / t);                                        //转换为帧率
        cout << "FPS: " << fps << endl;                                //输出帧率
#endif
        char c = waitKey(1);
        if (c == 27) //"Esc"-退出
        {
            break;
        }
    }
}