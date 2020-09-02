#ifndef LEDS_NAVIGATION_H
#define LEDS_NAVIGATION_H

#include "ros/ros.h"
#include <roboticarts_leds/SetLeds.h>

class LedsNavigation{

    public: 
        LedsNavigation(ros::NodeHandle nodehandle);
        void run();

    private:
        ros::NodeHandle _nh;
        std::string nodeName;

        void showMsg();

};

#endif