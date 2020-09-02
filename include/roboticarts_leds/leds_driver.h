#ifndef LEDS_DRIVER_H
#define LEDS_DRIVER_H

#include "ros/ros.h"
#include <serial/serial.h>
#include <string>
#include <iostream>
#include "std_srvs/Trigger.h"
#include <xmlrpcpp/XmlRpcValue.h>
#include <roboticarts_leds/SetLeds.h>
#include <roboticarts_leds/SetLight.h>
#include <ros/console.h>


#define FORWARD 0x01
#define BACKWARD 0x02
#define TURN_LEFT 0x03
#define TURN_RIGHT 0x04
#define OMNI_LEFT 0x05
#define OMNI_RIGHT 0x06
#define EMERGENCY 0x07
#define CUSTOM_PAINT 0x08
#define CUSTOM_BLINK 0x09
#define CUSTOM_SHIFT 0x0A
#define CUSTOM_TURN 0x0B
#define CURRENT_COMMAND 0x0C
#define CUSTOM_LED_A 0x0D
#define CUSTOM_LED_B 0x0E
#define CUSTOM_LED_C 0x0F
#define CUSTOM_LED_D 0x10


#define RESPONSE_OK 0x01
#define RESPONSE_ERROR 0x02


#define SIGNAL_OK 0x01
#define SIGNAL_NONE_OK 0x02
#define SIGNAL_TIMEOUT 0x03
#define SIGNAL_MALFORMED 0x04
#define SIGNAL_NOT_FOUND 0x05
#define SIGNAL_ALREADY_TRUE 0x06
#define SIGNAL_ALREADY_FALSE 0x07
#define UNKNOWN_ERROR 0x08


#define MSG_SIZE 12
#define HEADER 0x99
#define WRITE 0x01
#define READ 0x02
#define TAIL 0x23



class LedsDriver{

    public:
        LedsDriver(ros::NodeHandle nodehandle);
        void run();

    private:

        ros::NodeHandle _nh;
        std::string nodeName;
        XmlRpc::XmlRpcValue leds_signals; 

        ros::ServiceServer set_light_service;
        ros::ServiceServer set_leds_service;
        ros::ServiceServer get_leds_service;

       struct LedProperties{
            std::string name;
            uint8_t  command;
            uint8_t  init_led;
            uint8_t  end_led;
            uint8_t  color_R;
            uint8_t  color_G;
            uint8_t  color_B;
            uint16_t time;
            uint8_t  direction;
            int status;
        };
 
        serial::Serial serial;
        std::string serial_port; 
        int baudrate;

        std::string _current_signal = "READY";

        bool setLightCallback(roboticarts_leds::SetLight::Request& req, roboticarts_leds::SetLight::Response& res);
        bool setSignalCallback(roboticarts_leds::SetLeds::Request& req, roboticarts_leds::SetLeds::Response& res);
        bool currentSignalCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
        
        void readRosParams();

        std::string setSignal(std::string signal, bool enable);
        int sendSignal(std::string signal);
        LedProperties getLedProperties(std::string signal);
        int  findSignal(std::string signal);
        std::string printStatus(int status, std::string signal);
        

        void openSerialPort();
        int  writeSerialLeds(struct LedProperties led_properties);
        int readSerialLeds(uint8_t *res);
        int sendMessage(uint8_t message[]);



};


#endif