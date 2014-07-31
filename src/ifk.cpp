
#include "flexforce_adapter.h"

float rate = 200;

main(int argc, char** argv)
{
    ros::init(argc, argv, "interfacekit");
    ros::NodeHandle nh;

    flexforce ff(nh, rate);

    while (ros::ok()) {
        ff.spinOnce();
    }

    return 0;
}
