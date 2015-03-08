#include "flexforce_adapter.h"

flexforce::flexforce( ros::NodeHandle nh, int freq,  int timeout ) : rate(freq) {
    int result, i;
    const char *err;

    // Setup the data publishers
    frequency = freq;
    flexforce_pub = nh.advertise<geometry_msgs::Vector3Stamped>("flexforce", frequency);

    // Create the analog object
    interface = 0;
    CPhidgetInterfaceKit_create(&interface);

    // Handlers will be run in case of:
    //      Device attach
    //      Device detatch
    //      Software close
    //      Error
    CPhidget_set_OnAttach_Handler((CPhidgetHandle)interface, &AttachHandler, this);
    CPhidget_set_OnDetach_Handler((CPhidgetHandle)interface, &DetachHandler, this);
    CPhidget_set_OnError_Handler((CPhidgetHandle)interface, &ErrorHandler, this);

    // Callback that runs when input changes
    CPhidgetInterfaceKit_set_OnInputChange_Handler(interface, &InputChangeHandler, this);
    // Callback that runs if value changes more than trigger **DISABLED**
    //CPhidgetInterfaceKit_set_OnSensorChange_Handler(interface, &SensorChangeHandler, this);
    // Callback that runs if output changes
    CPhidgetInterfaceKit_set_OnOutputChange_Handler(interface, &OutputChangeHandler, this);

    // Open the analog device connections
    CPhidget_open((CPhidgetHandle)interface, -1);

    CPhidgetInterfaceKit_setSensorChangeTrigger(interface, 0, 1);

    // Program will now wait for analog device to be attached.
    ROS_INFO("Waiting for interface kit to be attached.");
    if ((result = CPhidget_waitForAttachment((CPhidgetHandle)interface, 10000))) {
        CPhidget_getErrorDescription(result, &err);
        return;
    }

    // Display analog device properties
    DisplayProperties(interface);
}

void flexforce::spinOnce() {
    publish_flexforce();
    //ros::spinOnce();
    //rate.sleep();
}

void flexforce::publish_flexforce() {
    int run1, run2;
    int temp1, temp2;

    CPhidgetInterfaceKit_getSensorValue(interface, 0, &temp1);
    CPhidgetInterfaceKit_getSensorValue(interface, 1, &temp2);
    flexforce_msg.vector.x = convertRAW(temp1);
    flexforce_msg.vector.y = convertRAW(temp2);

    flexforce_msg.vector.z = 0;
    flexforce_msg.header.stamp = ros::Time::now();
    flexforce_pub.publish(flexforce_msg);
}

void flexforce::DisplayProperties(CPhidgetInterfaceKitHandle phid) {
    int serialNo, version, numInputs, numOutputs, numSensors, triggerVal, ratiometric, i;
    const char* ptr;

    CPhidget_getDeviceType((CPhidgetHandle)phid, &ptr);
    CPhidget_getSerialNumber((CPhidgetHandle)phid, &serialNo);
    CPhidget_getDeviceVersion((CPhidgetHandle)phid, &version);

    CPhidgetInterfaceKit_getInputCount(phid, &numInputs);
    CPhidgetInterfaceKit_getOutputCount(phid, &numOutputs);
    CPhidgetInterfaceKit_getSensorCount(phid, &numSensors);
    CPhidgetInterfaceKit_getRatiometric(phid, &ratiometric);

    ROS_INFO("%s", ptr);
    ROS_INFO("Serial Number: %10d, Version: %8d", serialNo, version);
    ROS_INFO("# Digital Inputs: %d, # Digital Outputs: %d", numInputs, numOutputs);
    ROS_INFO("# Sensors: %d", numSensors);
    ROS_INFO("Ratiometric: %d", ratiometric);

    for(i = 0; i < numSensors; i++)
    {
        CPhidgetInterfaceKit_getSensorChangeTrigger (phid, i, &triggerVal);

        ROS_INFO("Sensor#: %d > Sensitivity Trigger: %d", i, triggerVal);
    }

    return;
}

double flexforce::convertRAW(int raw) {
    double temp;
    temp = 0.20197 * (double)raw;
    return temp;
}

int CCONV AttachHandler(CPhidgetHandle IFK, void *userptr) {
    int serialNo;
    const char *name;

    CPhidget_getDeviceName(IFK, &name);
    CPhidget_getSerialNumber(IFK, &serialNo);

    ROS_INFO("%s %10d attached!", name, serialNo);

    return 0;
}

int CCONV DetachHandler(CPhidgetHandle IFK, void *userptr) {
    int serialNo;
    const char *name;

    CPhidget_getDeviceName (IFK, &name);
    CPhidget_getSerialNumber(IFK, &serialNo);

    ROS_INFO("%s %10d detached!", name, serialNo);

    return 0;
}

int CCONV ErrorHandler(CPhidgetHandle IFK, void *userptr, int ErrorCode, const char *unknown) {
    ROS_INFO("Error handled. %d - %s", ErrorCode, unknown);
    return 0;
}

//callback that will run if an input changes.
//Index - Index of the input that generated the event, State - boolean (0 or 1) representing the input state (on or off)
int CCONV InputChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int State) {
    ROS_INFO("Digital Input: %d > State: %d", Index, State);
    return 0;
}

//callback that will run if an output changes.
//Index - Index of the output that generated the event, State - boolean (0 or 1) representing the output state (on or off)
int CCONV OutputChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int State) {
    ROS_INFO("Digital Output: %d > State: %d", Index, State);
    return 0;
}

//callback that will run if the sensor value changes by more than the OnSensorChange trigger.
//Index - Index of the sensor that generated the event, Value - the sensor read value
int CCONV SensorChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int Value) {
    ROS_INFO("Sensor: %d > Value: %d", Index, Value);
    return 0;
}
