#ifndef LEDS_TELEOP_H
#define LEDS_TELEOP_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <roboticarts_leds/SetLeds.h>

class LedsTeleop{

    public: 
        LedsTeleop(ros::NodeHandle nodehandle);
        void run();

    private:

        ros::NodeHandle _nh;
        ros::Subscriber vel_sub;

        std::string nodeName;

        ros::ServiceClient leds_client;

        std::string cmd_vel_topic;
        std::string cmd_led_service;

        float vel_x = 0, vel_y = 0, vel_z = 0;
        float velocity_threshold;

        std::string movement;
        std::string last_movement;

        bool retry_mode;
        double last_time;

        void getRosParams();
        void velCallback(const geometry_msgs::Twist::ConstPtr& msg);
        std::string getRobotMovement();
        void setLedSignal(std::string signal, bool enable);
        void runLedsTeleop();
        
};

#endif