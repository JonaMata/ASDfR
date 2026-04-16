#ifndef TEMPLATE20SIM_HPP
#define TEMPLATE20SIM_HPP

#include "XenoFrt20Sim.hpp"
#include "LoopController.h"

#pragma pack (1)    //https://carlosvin.github.io/langs/en/posts/cpp-pragma-pack/#_performance_test
struct LogStruct
{
    double setSteerLeft = 0;
    double setSteerRight = 0;
    double posLeft = 0;
    double posRight = 0;
};

#pragma pack(0)

class ControlLoop : public XenoFrt20Sim
{
public:
    ControlLoop(uint write_decimator_freq, uint monitor_freq);
    ~ControlLoop();
private:
    XenoFileHandler file;
    struct LogStruct data_to_be_logged;
    LoopController controller;

    double u[4];
    double y[4];

    int prevChannel1;
    int prevChannel2;
protected:
    //Functions
    int initialising() override;
    int initialised() override;
    int run() override;
    int stopping() override;
    int stopped() override;
    int pausing() override;
    int paused() override;
    int error() override;

    // current error
    int current_error = 0;
};

#endif // TEMPLATE20SIM_HPP