#ifndef FLEXFORCE_ADAPTER_H
#define FLEXFORCE_ADAPTER_H

#include <stdio.h>
#include <phidget21.h>
#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"

int CCONV AttachHandler(CPhidgetHandle IFK, void *userptr);
int CCONV DetachHandler(CPhidgetHandle IFK, void *userptr);
int CCONV ErrorHandler(CPhidgetHandle IFK, void *userptr, int ErrorCode, const char *unknown);
int CCONV InputChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int State);
int CCONV OutputChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int State);
int CCONV SensorChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int Value);

class flexforce {
public:
    CPhidgetInterfaceKitHandle interface;
    const char *name;
    int serialNo;
    int version;
    int frequency;
    geometry_msgs::Vector3Stamped flexforce_msg;

    ros::Publisher flexforce_pub;

    flexforce(ros::NodeHandle, int freq, int timeout = 10000);
    void spinOnce();
    void publish_flexforce();
    void DisplayProperties(CPhidgetInterfaceKitHandle phid);
    double convertRAW(int raw);

private:
    ros::Rate rate;
};


#endif // FLEXFORCE_ADAPTER_H
