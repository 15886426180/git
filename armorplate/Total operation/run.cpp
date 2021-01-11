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
        // #if FPS_IMSHOW == 1
        double t = (double)cv::getTickCount(); //开始计时
                                               // #endif
        if (cap.isindustryimgInput())
        {
            frame = cvarrToMat(cap.iplImage, true);
        }
        else
        {
            capture >> frame;
        }

        // armor.success_armor = false;
        Mat src_img, roi_img;
        resize(frame, src_img, Size(CAMERA_RESOLUTION_COLS, CAMERA_RESOLUTION_ROWS));
        //ROI
        if (armor.lost_success_armor)
        {
            roi_img = src_img(armor.armor_roi);
        }
        else
        {
            roi_img = src_img; //直接赋值
        }

        //图像预处理
        img.pretreat(roi_img, COLOR);
        //找到灯条后

        if (rgb.find_light(img.mask))
        {
            //装甲板大于等于1块时
            if (rgb.armor_fitting(img.gray_img))
            {
                armor.rect_num = rgb.optimal_armor() / 2;
                // cout<<rgb.armor[armor.rect_num].center<<endl;
                // Point ddd = kalman.point_Predict(36, rgb.armor[armor.rect_num].center);
                // cout<<ddd<<endl;
                // cout<<endl;
                //小孔成像(正面效果还行)
                // float left_light_depth = pnp.Pinhole_imaging(rgb.light[armor.rect_num * 2], ARMORPLATE_HIGHT);
                // float right_light_depth = pnp.Pinhole_imaging(rgb.light[armor.rect_num * 2 + 1], ARMORPLATE_HIGHT);
                float depth = 0;
                // cout<<depth/10<<endl;
                // if (rgb.armor[armor.rect_num].size.width / rgb.armor[armor.rect_num].size.height > 2)
                // {
                //     depth = pnp.arrange_Point(rgb.armor[armor.rect_num], BIG_ARMORPLATE_WIDITH, ARMORPLATE_HIGHT);
                // }
                // else
                // {
                //     depth = pnp.arrange_Point(rgb.armor[armor.rect_num], SMALL_ARMORPLATE_WIDTH, ARMORPLATE_HIGHT);
                // }
                // cout<<depth/10<<endl;
#if DRAW_ARMOR_IMG == 1
                // rectangle(armor.draw_img, rgb.armor[armor.rect_num].boundingRect(), Scalar(0, 255, 0), 3, 8);
                // rectangle(armor.draw_img, rgb.roi_rect.boundingRect(), Scalar(255, 200, 0), 3, 8);
#endif
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
                armor.success_armor = true;                     //识别正确
                armor.lost_success_armor = armor.success_armor; //保存上一帧的参数
                rectangle(src_img, rgb.armor[armor.rect_num].boundingRect(), Scalar(0, 255, 0), 3, 8);
                rectangle(src_img, armor.armor_roi, Scalar(255, 200, 0), 3, 8);

                rgb.lost_armor = rgb.armor[armor.rect_num];
            }
            else //丢失目标
            {
                armor.success_armor = false;
                armor.lost_success_armor = armor.success_armor;

                // if(armor.lost < 2)
                // {
                //     armor.lost++;
                //     armor.success_armor = false;//识别失败
                //     armor.lost_success_armor = true;
                //     int point_x = rgb.lost_armor.center.x - (rgb.lost_armor.size.width/2)+armor.armor_roi.x;
                //     int point_y = rgb.lost_armor.center.y - (rgb.lost_armor.size.height*2)+armor.armor_roi.y;
                //     int width = rgb.lost_armor.size.width*8;
                //     int height = rgb.lost_armor.size.height*8;
                //     if(point_x + width >CAMERA_RESOLUTION_COLS)
                //     {
                //         width = CAMERA_RESOLUTION_COLS- point_x;
                //     }
                //     if (point_y+height >CAMERA_RESOLUTION_ROWS)
                //     {
                //         height = CAMERA_RESOLUTION_ROWS- point_y;
                //     }
                //     armor.armor_roi = Rect(
                //         point_x,
                //         point_y,
                //         width,
                //         height
                //     );
                // }
                // else
                // {
                //     armor.success_armor = false;//识别失败
                //     armor.lost_success_armor = false;
                // }
            }
        }
        rgb.eliminate();
        armor.eliminate();

        // imshow("draw", armor.draw_img);
        // #if FPS_IMSHOW == 1
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency(); //结束计时
        int fps = int(1.0 / t);                                        //转换为帧率
        cout << "FPS: " << fps << endl;                                //输出帧率
                                                                       //#endif
        Point ddd = kalman.point_Predict(1 / t, Point(armor.armor_roi.x + (armor.armor_roi.width / 2), armor.armor_roi.y + (armor.armor_roi.height / 2)));
        circle(src_img, ddd, 3, Scalar(0, 255, 255), 3, 8);
        imshow("frame", src_img);
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
    cv::Size size(CAMERA_RESOLUTION_COLS, CAMERA_RESOLUTION_ROWS);
    //--- INITIALIZE VIDEOWRITER
    bool isColor = (frame.type() == CV_8UC3);
    VideoWriter writer;
    int codec = VideoWriter::fourcc('M', 'J', 'P', 'G'); // select desired codec (must be available at runtime)
    double fps = 25.0;                                   // framerate of the created video stream
    string filename = "/home/xx/视频/2.avi";             // name of the output video file
    writer.open(filename, codec, fps, size, isColor);
    // check if we succeeded
    if (!writer.isOpened())
    {
        cout << "Could not open the output video file for write\n";
    }
    //--- GRAB AND WRITE LOOP
    cout << "Writing videofile: " << filename << endl
         << "Press any key to terminate" << endl;
    /**************/
    int n = 0;
    for (;;)
    {

        if (cap.isindustryimgInput())
        {
            frame = cvarrToMat(cap.iplImage, true);
        }
        imshow("frame", frame);

        char *cstr = new char[120];
        sprintf(cstr, "%s%d%s", "/home/xx/视频/1/", n++, ".jpg");
        imwrite(cstr, frame);
        char c = waitKey(1);
        cap.cameraReleasebuff();
        n++;
        if (c == 27) //"Esc"-退出
        {

            break;
        }
    }
}