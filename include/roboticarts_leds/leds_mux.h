#ifndef LEDS_MUX_H
#define LEDS_MUX_H

#include "ros/ros.h"
#include <xmlrpcpp/XmlRpcValue.h>
#include <roboticarts_leds/SetLeds.h>
#include <roboticarts_leds/service_handler.h>

#define MAX_SERVICIES 255

class LedsMux{

  public:
        LedsMux(ros::NodeHandle nodehandle);
        void run();
        
  private:
        ros::NodeHandle _nh;
        std::string nodeName;
        XmlRpc::XmlRpcValue leds_mux; 
        std::string cmd_leds_output_name;
        int valid_services_counter;
        
        ServiceHandler *leds_service[MAX_SERVICIES];

        struct ServiceProperties{
          std::string name;
          std::string service_name;
          double timeout;
          int priority;
          bool status;
        };


        void readRosParams();
        void createServiceMultiplexer();
        ServiceProperties getServiceProperties(int service);
        void runMuxTimeout();
};
 

#endif