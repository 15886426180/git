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
        img.pretreat(src_img, ENEMY_COLOR);
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

        Mat src_img;
        resize(frame, src_img, Size(CAMERA_RESOLUTION_COLS, CAMERA_RESOLUTION_ROWS));
        buff.pretreat(src_img, ENEMY_COLOR);
        if (buff.Looking_for_center())
        {
            int num = buff.Looking_for_target();
            float pre_center_angle;
            int _w = MAX(buff.max_buff_rects[buff.hit_subscript].size.width, buff.max_buff_rects[buff.hit_subscript].size.height);
            int _h = MIN(buff.max_buff_rects[buff.hit_subscript].size.width, buff.max_buff_rects[buff.hit_subscript].size.height);
            if (num < 6 && num > 0)
            {
                buff.Calculating_coordinates(buff.hit_subscript);
                pre_center_angle = atan2(buff.central_point.y - buff.pre_center.y, buff.central_point.x - buff.pre_center.x) * 180 / CV_PI + 90;
                buff.rects_2d = RotatedRect(buff.pre_center, Size(_w, _h), pre_center_angle);
                Point2f vertex[4];
                buff.rects_2d.points(vertex);
                for (int i = 0; i < 4; i++)
                {
                    line(src_img, vertex[i], vertex[(i + 1) % 4], Scalar(255, 100, 200), 5, CV_AA);
                }
            }
            else
            {
                buff.pre_center = buff.central_point;
                buff.rects_2d = RotatedRect(buff.pre_center, Size(_w, _h), 0);
                rectangle(buff.src_img, buff.rects_2d.boundingRect(), Scalar(0, 255, 0), 3, 8);
                pre_center_angle = 0;
            }
            pnp.vertex_Sort(buff.rects_2d);
            pnp.run_SolvePnp_Buff(src_img, pre_center_angle, MAX_BUFF_WIDTH, MAX_BUFF_HEIGHT);
            buff.yaw_data = pnp.angle_x;
            // buff.yaw_data = yaw_test;
            buff.pitch_data = pnp.angle_y;

            //test 半径补偿
            if (buff.yaw_data < 0)
            {
                buff.yaw_data += buff.offset_ratio / 10;
            }
            else
            {
                buff.yaw_data -= buff.offset_ratio / 10;
            }

            if (buff.pitch_data > 0)
            {
                buff.pitch_data += buff.offset_ratio / 10;
            }
            else
            {
                buff.pitch_data -= buff.offset_ratio / 10;
            }
            //test 半径补偿

            buff.depth = int(pnp.dist);

            if (buff._offset_x == 0)
            {
                buff.yaw_data = buff.yaw_data - buff.offset_x / 100;
            }
            else
            {
                buff.yaw_data = buff.yaw_data + buff.offset_x / 100;
            }

            if (buff._offset_y == 0)
            {
                buff.pitch_data = buff.pitch_data - buff.offset_y / 100;
            }
            else
            {
                buff.pitch_data = buff.pitch_data + buff.offset_y / 100;
            }
        }
        else
        {
            buff.yaw_data = 0;
            buff.pitch_data = 0;
            buff.yaw_data = 0;
            buff.pitch_data = 0;
        }
        serial.RMserialWrite(buff._yaw_data, fabs(buff.yaw_data) * 100, buff._pitch_data, fabs(buff.pitch_data) * 100, buff.depth, buff.choice_success, buff.diff_angle_);
        line(buff.src_img, Point(640, 0), Point(640, 800), Scalar(0, 255, 255), 3, 5);
        line(buff.src_img, Point(0, 400), Point(1280, 400), Scalar(0, 255, 255), 3, 5);
        imshow("frame", buff.src_img);
#if FPS_SHOW == 1
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency(); //结束计时
        int fps = int(1.0 / t);                                        //转换为帧率
        cout << "FPS: " << fps << endl;                                //输出帧率
#endif

        buff.choice_success = false;
        buff.central_success = false;
        buff.armor_center.clear();
        buff.max_buff_rects.clear();
        buff.contours.clear();
        buff.hierarchy.clear();
        buff.buff.clear();

        cap.cameraReleasebuff();
        char c = waitKey(1);
        if (c == 27) //"Esc"-退出
        {
            break;
        }
    }
}