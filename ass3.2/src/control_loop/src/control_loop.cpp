#include "control_loop.hpp"

ControlLoop::ControlLoop(uint write_decimator_freq, uint monitor_freq) :
    XenoFrt20Sim(write_decimator_freq, monitor_freq, file, &data_to_be_logged),
    file(1,"./xrf2_logging/TEMPLATE","bin"), // change template to your project name
    controller()
{
     printf("%s: Constructing rampio\n", __FUNCTION__);
    // Add variables to logger to be logged, has to be done before you can log data
    logger.addVariable("set_steer_left", double_);
    logger.addVariable("set_steer_right", double_);
    logger.addVariable("pos_left", double_);
    logger.addVariable("pos_right", double_);

    // To infinite run the controller, uncomment line below
    controller.SetFinishTime(0.0);
}

ControlLoop::~ControlLoop()
{
    
}

int ControlLoop::initialising()
{
    // Set physical and cyber system up for use in a 
    // Return 1 to go to initialised state

    evl_printf("Hello from initialising\n");      // Do something

    // The logger has to be initialised at only once
    logger.initialise();
    // The FPGA has to be initialised at least once
    ico_io.init();

    return 1;
}

int ControlLoop::initialised()
{
    // Keep the physical syste in a state to be used in the run state
    // Call start() or return 1 to go to run state

    evl_printf("Hello from initialised\n");       // Do something

    return 0;
}

int ControlLoop::run()
{
    // Do what you need to do
    // Return 1 to go to stopping state

    // Start logger
    logger.start();                             
    monitor.printf("Hello from run\n");  

    u[0] = sample_data.channel1;
    u[1] = sample_data.channel2;
    u[2] = ros_msg.steer_left;
    u[3] = ros_msg.steer_right;

    controller.Calculate(u, y);

    xeno_msg.pos_left = y[0];
    xeno_msg.pos_right = y[1];
    actuate_data.pwm1 = y[2];
    actuate_data.pwm2 = y[3];
    
    //  Change some data for logger            
    data_to_be_logged.setSteerLeft = u[0];
    data_to_be_logged.setSteerRight = u[1];
    data_to_be_logged.posLeft = y[0];
    data_to_be_logged.posRight = y[1];
    if(controller.IsFinished())
        return 1;


    return 0;
}

int ControlLoop::stopping()
{
    // Bring the physical system to a stop and set it in a state that the system can be deactivated
    // Return 1 to go to stopped state
    logger.stop();                                // Stop logger
    evl_printf("Hello from stopping\n");          // Do something

    return 1;
}

int ControlLoop::stopped()
{
    // A steady state in which the system can be deactivated whitout harming the physical system

    monitor.printf("Hello from stopping\n");          // Do something

    return 0;
}

int ControlLoop::pausing()
{
    // Bring the physical system to a stop as fast as possible without causing harm to the physical system

    evl_printf("Hello from pausing\n");           // Do something
    return 1 ;
}

int ControlLoop::paused()
{
    // Keep the physical system in the current physical state

    monitor.printf("Hello from paused\n");            // Do something
    return 0;
}

int ControlLoop::error()
{
    // Error detected in the system 
    // Can go to error if the previous state returns 1 from every other state function but initialising 

    monitor.printf("Hello from error\n");             // Do something

    return 0;
}
