
#ifndef SERVICE_HANDLER_H
#define SERVICE_HANDLER_H

#include "ros/ros.h"
#include <roboticarts_leds/SetLeds.h>
#include <array>
#include <algorithm>


std::array<uint8_t,256> priority_table = {0};


class ServiceHandler{

    public: 

      ServiceHandler(ros::NodeHandle nodehandle, std::string service_name, 
                     uint8_t priority, double timeout, std::string cmd_leds_output_name);

      void runTimeout();
      ServiceHandler(){}
      
    private:

      ros::NodeHandle _nh;
      std::string nodeName;
      ros::ServiceServer leds_service;
      ros::ServiceClient leds_client;
      uint8_t priority;
      std::string cmd_leds_output_name;

      std::string pending_signal = "";
      bool pending_enable = false;
      double timeout;
      bool flag = true;

      double last_time;
      bool timeout_request = false;

      bool serviceHandlerCallback(roboticarts_leds::SetLeds::Request& req, roboticarts_leds::SetLeds::Response& res);
  
      void getPriority();
      bool checkPriority();
      void releasePriority();
      void printPriorityTable();

};


ServiceHandler::ServiceHandler(ros::NodeHandle nodehandle, std::string service_name, uint8_t priority, 
                               double timeout, std::string cmd_leds_output_name):_nh(nodehandle){
  
  this->cmd_leds_output_name = cmd_leds_output_name;
  this->priority = priority; 
  this->timeout = timeout;

  leds_service = _nh.advertiseService(service_name, &ServiceHandler::serviceHandlerCallback, this);
  leds_client = _nh.serviceClient<roboticarts_leds::SetLeds>(cmd_leds_output_name);

}


 bool ServiceHandler::serviceHandlerCallback(roboticarts_leds::SetLeds::Request& req, 
                                             roboticarts_leds::SetLeds::Response& res){

  pending_signal = req.signal;
  pending_enable = req.enable;
  
  if(checkPriority()){
    
    getPriority();
    timeout_request = false;

    roboticarts_leds::SetLeds leds_srv;

    leds_srv.request.signal = req.signal;
    leds_srv.request.enable = req.enable;

    if (leds_client.call(leds_srv)){

      res.message = leds_srv.response.message;
      res.success = leds_srv.response.success;

      if(res.success){

        if (req.enable == false){
          timeout_request = true;
          last_time = ros::Time::now().toSec();
        }

      }
      else
        releasePriority();

    }
    else{
      res.message = "Failed to call service " +  cmd_leds_output_name;
      res.success = false;
      ROS_WARN("%s", res.message.c_str());
    }

  }

  else{
      res.message = "This service has not priority";
      res.success = false;
      ROS_DEBUG("%s", res.message.c_str());
  }

  return true;


 }




void ServiceHandler::getPriority(){

  //Save priortiy 
  priority_table[this->priority] = this->priority;
  ROS_ERROR("%d ---------- GET", this->priority);
  printPriorityTable();
}


bool ServiceHandler::checkPriority(){

  bool hasPriority;

  if (this->priority >= *std::max_element(priority_table.begin(), priority_table.end()))
      hasPriority = true;
  else
      hasPriority = false;


  return hasPriority;
}


void ServiceHandler::releasePriority(){

    priority_table[this->priority] = 0;
    ROS_ERROR("%d --------- RELEASE", this->priority);
    printPriorityTable();
}


void ServiceHandler::printPriorityTable(){

  std::string msg = "";

  for(int i=0; i<=255; i++)

    msg = msg + std::to_string(priority_table[i]) + " ";
  
  ROS_INFO("%s", msg.c_str());
}


void ServiceHandler::runTimeout(){


  if(timeout_request){

    if((ros::Time::now().toSec() - last_time) > timeout){
      
      releasePriority();
      timeout_request = false;
      ROS_DEBUG("Resource released");
    }
  }

    if(pending_enable == true){
    
      
      if (checkPriority() == true && flag == false){
         
          flag = true;
          getPriority();

          if(pending_enable == true){
            roboticarts_leds::SetLeds leds_srv;
            leds_srv.request.signal = pending_signal;
            leds_srv.request.enable = pending_enable;
            leds_client.call(leds_srv);
          }
      }
    
    }
    if(checkPriority() == false && flag == true)
      flag = false;

  
    
}



#endif
