#include "urs100_hardware_interface/urs100_hardware_interface.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "urs100_hardware_interface");
    ros::NodeHandle nh;
    // Increment thread count if more actions/services/publishers/... are added to urs100
    // See: https://answers.ros.org/question/313629/attempt-to-spin-a-callback-queue-from-two-spinners-one-of-them-being-single-threaded/?answer=313632#post-id-313632
    ros::AsyncSpinner spinner(2);
    urs100_hardware_interface::Urs100HardwareInterface Urs100Stage(nh);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
