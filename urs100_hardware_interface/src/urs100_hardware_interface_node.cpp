#include "urs100_hardware_interface/urs100_hardware_interface.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "urs100_hardware_interface");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    urs100_hardware_interface::Urs100HardwareInterface Urs100Stage(nh);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
