#include "configure.h"
#include "control.h"
#include "armor/armorplate.h"
#include "pnp/solvepnp.h"
#include "serial/serialport.h"
#include "camera/videocapture.h"
#include "kalmantest/kalmantest.h"
class WorKing
{
public:
    WorKing();
    ~WorKing();
    void Run();
    void ddd();
    ArmorPlate armor;
    LightBar rgb;
    ImageProcess img;
#if CALL_SERIALPORT == 1
    SerialPort serial;
#endif
    SolveP4p pnp;
    cv::VideoCapture capture;
    VideoCap cap;
    RM_kalmanfilter kalman;
    Mat frame;
};
