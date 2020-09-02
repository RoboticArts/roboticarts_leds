#include <roboticarts_leds/leds_mux.h>


LedsMux::LedsMux(ros::NodeHandle nodehandle):_nh(nodehandle){

  // Enable ROS_DEBUG output
  //if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
  //    ros::console::notifyLoggerLevelsChanged();

  // Get name of this node
  nodeName = ros::this_node::getName();

  valid_services_counter = 0;

  // Read params from rosparams
  readRosParams();

  //Wait service
  ros::service::waitForService(cmd_leds_output_name);

  // Create services from yaml 
  createServiceMultiplexer();
     
}


void LedsMux::readRosParams(){

  ros::param::param<std::string>(nodeName + "/cmd_leds_output", cmd_leds_output_name, "cmd_leds_output");

  if (_nh.hasParam(nodeName + "/services")){
    ROS_INFO("leds_mux.yaml found");
    ros::param::get(nodeName + "/services", leds_mux);    
  }
  else{
    ROS_ERROR("Leds mux configuration not found, check %s/services param exits", nodeName.c_str());
    ros::shutdown();
    std::exit(0);
  }

}

void LedsMux::createServiceMultiplexer(){

  for (int service=0; service<leds_mux.size(); service++){
    
    struct ServiceProperties service_properties = getServiceProperties(service);

    if(service_properties.status){

      uint8_t priority = service_properties.priority;
      std::string service_name = service_properties.service_name;
      double timeout = service_properties.timeout;

      leds_service[valid_services_counter] = new ServiceHandler(_nh, service_name, priority, timeout, this->cmd_leds_output_name);

      valid_services_counter++;

    }
  }
}


LedsMux::ServiceProperties LedsMux::getServiceProperties(int service){

  ServiceProperties service_properties;
  service_properties.status = true;

  if(leds_mux[service].hasMember("name")){
    service_properties.name = static_cast<std::string>(leds_mux[service]["name"]);
    ROS_DEBUG("%s", service_properties.name.c_str());

  }
  else{
      ROS_WARN("Service [%d] is malformed, [name] not found", service);
      service_properties.status = false;
  }
  
  if(leds_mux[service].hasMember("service")){
    service_properties.service_name = static_cast<std::string>(leds_mux[service]["service"]);
    ROS_DEBUG("%s", service_properties.service_name.c_str());
  }
  else{
      ROS_WARN("Service [%d] is malformed, [service] not found", service);
      service_properties.status = false;
  }
  
  if(leds_mux[service].hasMember("timeout")){
    service_properties.timeout = double(leds_mux[service]["timeout"]);
    ROS_DEBUG("%.2f", service_properties.timeout);
  }
  else{
    ROS_WARN("Service [%d] is malformed, [timeout] not found", service);
    service_properties.status = false;
  }

  if(leds_mux[service].hasMember("priority")){
    service_properties.priority = int(leds_mux[service]["priority"]);
    ROS_DEBUG("%d", service_properties.priority);
  }    
  else{
    ROS_WARN("Service [%d] is malformed, [priority] not found", service);
    service_properties.status = false;
  }


  return service_properties;
}


void LedsMux::runMuxTimeout(){

  for(int service = 0; service < valid_services_counter; service++)
    leds_service[service]->runTimeout();

  

}


void LedsMux::run(){

    ros::Rate loop_rate(20);

    while (ros::ok()){
      
      runMuxTimeout();
      ros::spinOnce();
      loop_rate.sleep();

    }

}
