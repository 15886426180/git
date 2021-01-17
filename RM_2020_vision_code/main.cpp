#include "control/rm_link.h"

int main()
{
    RM_Vision_Init run;
#if IS_SERIAL_OPEN == 1
    SerialPort serialport;
#endif
//    g_Ctrl.my_color = ALL_COLOR;
//    g_Ctrl.now_run_mode = DEFAULT_MODE;
    for(;;)
    {
        double t = double(getTickCount());
        /** run **/
        run.Run();

        t = (double(getTickCount() - t)) / getTickFrequency();
        run.armor._t = t;
#if COUT_FPS == 1
        int fps = int(1.0 / t);
        cout<< endl << "FPS: " << fps<< endl;
#endif
        if(run.is_exit()){
            break;
        }
    }
    return EXIT_SUCCESS;
}
