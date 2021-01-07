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

        armor.success_armor = false;
        Mat src_img;
        resize(frame, src_img, Size(CAMERA_RESOLUTION_COLS, CAMERA_RESOLUTION_ROWS));
        //图像预处理
        img.pretreat(src_img, COLOR);
        //找到灯条后

        if (rgb.find_light(img.mask))
        {
            //装甲板大于等于1块时
            if (rgb.armor_fitting(img.gray_img))
            {
                armor.rect_num = rgb.optimal_armor();

                //计算左右灯条深度
                float left_depth = pnp.arrange_Point(rgb.light[armor.rect_num], LIGHT_WIDITH, ARMORPLATE_HIGHT);
                float right_depth = pnp.arrange_Point(rgb.light[armor.rect_num + 1], LIGHT_WIDITH, ARMORPLATE_HIGHT);
                //线性拟合装甲板深度
                armor.depth = (MAX(right_depth, left_depth)* 0.7) + (MIN(right_depth, left_depth)*0.3);

#if DRAW_ARMOR_IMG == 1
                rectangle(armor.draw_img, rgb.armor[armor.rect_num].boundingRect(), Scalar(0, 255, 0), 3, 8);
                rectangle(armor.draw_img, rgb.roi_rect.boundingRect(), Scalar(255, 200, 0), 3, 8);
                rectangle(src_img, rgb.armor[armor.rect_num].boundingRect(), Scalar(0, 255, 0), 3, 8);
                rectangle(src_img, rgb.roi_rect.boundingRect(), Scalar(255, 200, 0), 3, 8);
#endif
                armor.success_armor = true;
            }
            else //丢失目标
            {

                //扩大搜索目标
#if ROI_IMG == 1
                armor.lost++;
                rgb.roi_rect = RotatedRect(
                    Point(IMG_COLS / 2, IMG_ROWS / 2),
                    Size(IMG_COLS, IMG_ROWS),
                    rgb.roi_rect.angle);
#endif
            }
        }

        rgb.eliminate();
        armor.eliminate();

        imshow("frame", src_img);
        imshow("draw", armor.draw_img);
// #if FPS_IMSHOW == 1
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency(); //结束计时
        int fps = int(1.0 / t);                                        //转换为帧率
        cout << "FPS: " << fps << endl;                                //输出帧率
// #endif
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
    for (;;)
    {
        if (cap.isindustryimgInput())
        {
            frame = cvarrToMat(cap.iplImage, true);
        }
        imshow("frame", frame);
        cap.cameraReleasebuff();
        char c = waitKey(1);
        if (c == 27) //"Esc"-退出
        {
            break;
        }
    }
}