#include "configure.h"
#include "control.h"
#include "armor/armorplate.h"
#include "pnp/solvepnp.h"
#include "serial/serialport.h"
#include "camera/videocapture.h"

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
    SolveP4p pnp;
    cv::VideoCapture capture;
    VideoCap cap;

    Mat frame;
};
