#include <roboticarts_leds/leds_driver.h>


LedsDriver::LedsDriver(ros::NodeHandle nodehandle):_nh(nodehandle)
{
  // Get name of tgis node
  nodeName = ros::this_node::getName();

  // Enable ROS_DEBUG output
 //if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
 //   ros::console::notifyLoggerLevelsChanged();

  // Create service servers
  set_light_service = _nh.advertiseService(nodeName + "/set_light", &LedsDriver::setLightCallback, this); 

  set_leds_service = _nh.advertiseService(nodeName + "/set_leds_signal", &LedsDriver::setSignalCallback, this);
  get_leds_service = _nh.advertiseService(nodeName+"/get_leds_signal",&LedsDriver::currentSignalCallback,this);

  // Read params from rosparams
  readRosParams();

  // Open serial port
  openSerialPort();
  
  // Node ready
  ROS_INFO("%s node ready!", nodeName.c_str());

}


void LedsDriver::openSerialPort(){

  try{
    serial.setPort(serial_port);
    serial.setBaudrate(baudrate);

    serial::Timeout to = serial::Timeout::simpleTimeout(200);
    serial.setTimeout(to);
    
    serial.open();
  }
  catch (serial::IOException& e)
  {
        ROS_ERROR("Unable to open port %s", serial_port.c_str());
        ros::shutdown();
  }

  if(serial.isOpen())
        ROS_INFO("Serial port %s open", serial_port.c_str());
  else
        ros::shutdown();

  
}


void LedsDriver::readRosParams(){

  // Read serial params
  ros::param::param<std::string>(nodeName + "/serial_port", serial_port, "/dev/ttyACM0");
  ros::param::param<int>(nodeName + "/baudrate", baudrate, 115200);

  if (_nh.hasParam(nodeName + "/leds_signals")){
    ROS_INFO("leds_signals.yaml found");
    ros::param::get(nodeName + "/leds_signals", leds_signals);    
  }
  else{
    ROS_ERROR("Led signal configuration not found, check %s/leds_signals param exits", nodeName.c_str());
    ros::shutdown();
  }

  ROS_DEBUG("%s", serial_port.c_str());

}


bool LedsDriver::setLightCallback(roboticarts_leds::SetLight::Request& req,
                                      roboticarts_leds::SetLight::Response& res)
{
  uint8_t message[MSG_SIZE] = {};
  int status = -1;

  //Build message
  message[0] = HEADER;           // Header
  message[1] = WRITE;            // Mode
  message[2] = CUSTOM_LED_A;     // Command
  message[5] = req.brightness;   // Brightness
	message[11] = TAIL;            // EOL

  status = sendMessage(message);

  if (status == RESPONSE_OK)
    res.success = true;
  else
    res.success = false;
  
  return true;
}



bool LedsDriver::setSignalCallback(roboticarts_leds::SetLeds::Request& req,
                                      roboticarts_leds::SetLeds::Response& res)
{

  res.message = setSignal(req.signal, req.enable);

  if (res.message.find("True") != std::string::npos || res.message.find("False") != std::string::npos)
    res.success = true;
  else
    res.success = false;

  return true;
}


bool LedsDriver::currentSignalCallback(std_srvs::Trigger::Request& req,
                                            std_srvs::Trigger::Response& res)
{
  res.success = true;
  res.message = _current_signal;

  return true;
}


int LedsDriver::findSignal(std::string signal){

  int index = -1;

  for (int i = 0; i<leds_signals.size(); i++){
   
    if(leds_signals[i].hasMember("name")){

        std::string name = leds_signals[i]["name"];

        if(name.compare(signal) == 0){
          ROS_DEBUG("Signal: [%s] found", name.c_str());
          index = i;
        }       
    }
  }
  return index;
}


// https://stackoverflow.com/questions/52896291/how-to-get-list-element-from-rosparam-parameter-from-the-cmd

LedsDriver::LedProperties LedsDriver::getLedProperties(std::string signal){

    int index = findSignal(signal);
    int status = SIGNAL_OK;
    struct LedProperties led_properties;

    // If signal exists
    if(index >= 0){

      // Read name
      if(leds_signals[index].hasMember("name")){

        std::string name =  leds_signals[index]["name"];
        led_properties.name = name;

        ROS_DEBUG("%s", name.c_str());

      }
      else{
        ROS_WARN("Signal [%s] is malformed, [behavior] not found", signal.c_str());
        status = SIGNAL_MALFORMED;
      }


      // Read behavior
      if(leds_signals[index].hasMember("behavior")){

        std::string behavior =  leds_signals[index]["behavior"];

        // Select command
        if(behavior == "paint")
            led_properties.command = CUSTOM_PAINT;
        else if(behavior == "blink")
            led_properties.command = CUSTOM_BLINK;
        else if(behavior == "shift_left")
            led_properties.command = CUSTOM_SHIFT;
        else if(behavior == "shift_right")
            led_properties.command = CUSTOM_SHIFT;
        else if(behavior == "shift_turn_left")
            led_properties.command = CUSTOM_TURN;   
        else if(behavior == "shift_turn_right")
            led_properties.command = CUSTOM_TURN;
        else{
            ROS_WARN("Signal [%s] is malformed, behavior [%s] does not exist", signal.c_str(), behavior.c_str());
            status = SIGNAL_MALFORMED;
        }

        // Select direction
        if(behavior == "shift_left" || behavior == "shift_turn_left")
            led_properties.direction = 0;   
        if(behavior == "shift_right" || behavior == "shift_turn_right")
            led_properties.direction = 1;

      }
      else{
        ROS_WARN("Signal [%s] is malformed, [behavior] not found", signal.c_str());
        status = SIGNAL_MALFORMED;
      }


      // Read zone
      if(leds_signals[index].hasMember("zone")){

        if(leds_signals[index]["zone"].size() == 2){

          // Select start and end led
          led_properties.init_led = int(leds_signals[index]["zone"][0]) - 1; // Counter starts in 0
          led_properties.end_led = int(leds_signals[index]["zone"][1]) - 1;

        }
        else{
          ROS_WARN("Signal [%s] is malformed, [zone] has not the correct size", signal.c_str());
          status = SIGNAL_MALFORMED;
        }

      }
      else{
        ROS_WARN("Signal [%s] is malformed, [zone] not found", signal.c_str());
        status = SIGNAL_MALFORMED;
      }


      // Read color
      if(leds_signals[index].hasMember("color")){

        if (leds_signals[index]["color"].size() == 3){
          
          // Select color
          led_properties.color_R = int(leds_signals[index]["color"][0]);
          led_properties.color_G = int(leds_signals[index]["color"][1]);
          led_properties.color_B = int(leds_signals[index]["color"][2]);
    
        }
        else{
            ROS_WARN("Signal [%s] is malformed, [color] has not the correct size", signal.c_str());
            status = SIGNAL_MALFORMED;
        }

      }
      else{
        ROS_WARN("Signal [%s] is malformed, [color] not found", signal.c_str());
        status = SIGNAL_MALFORMED;
      }


      // Read time 
      if(leds_signals[index].hasMember("time")){

        //Select time
        led_properties.time = int(leds_signals[index]["time"]);

      }
      else{
        ROS_WARN("Signal [%s] is malformed, [time] not found", signal.c_str());
        status = SIGNAL_MALFORMED;
      }

    }
    else{
        status = SIGNAL_NOT_FOUND;
    }

  led_properties.status = status;  

  ROS_DEBUG("-------------------------");
  ROS_DEBUG("Command: %d", led_properties.command);
  ROS_DEBUG("Init led: %d", led_properties.init_led);
  ROS_DEBUG("End led %d", led_properties.end_led);
  ROS_DEBUG("Color_R: %d", led_properties.color_R);
  ROS_DEBUG("Color_G: %d", led_properties.color_G);
  ROS_DEBUG("Color_B: %d", led_properties.color_B);
  ROS_DEBUG("Time: %d", led_properties.time);
  ROS_DEBUG("Direction: %d", led_properties.direction);

  return led_properties;
}


int LedsDriver::writeSerialLeds(struct LedProperties led_properties){

  uint8_t message[MSG_SIZE]={};
  int status = -1;

  //Build message
  message[0] = HEADER;                         // Header
  message[1] = WRITE;                          // Mode
  message[2] = led_properties.command;         // Led Command
	message[3] = led_properties.init_led;        // Start led 
	message[4] = led_properties.end_led;         // End led
	message[5] = led_properties.color_R;         // Color R
	message[6] = led_properties.color_G;         // Color G
	message[7] = led_properties.color_B;         // Color B
	message[8] = led_properties.time >> 8;       // Time MSB
	message[9] = led_properties.time & 0x00FF;   // Time LSB
	message[10] = led_properties.direction;      // Direction
	message[11] = TAIL;                          // EOL

  status = sendMessage(message);

  return status;
}


int LedsDriver::readSerialLeds(uint8_t *res){
    
    uint8_t message[MSG_SIZE] = {};
    int status = -1;

    //Build message
    message[0] = HEADER;
    message[1] = READ;
    message[2] = CURRENT_COMMAND;
    message[11] = TAIL;

    status = sendMessage(message);

  return status;
}


int LedsDriver::sendMessage(uint8_t message[]){

    int status = -1;
    int attemps = 0;

    while(status != RESPONSE_OK){

        // Write message
        serial.write(message, MSG_SIZE);

        // Get response, wait until 3 bytes or timeout
        uint8_t response[3];
      
        serial.read(response, 3);        
        
        // Check if response is OK
        if (response[0] == RESPONSE_OK && response[2] == TAIL)
            status = RESPONSE_OK;
        else
            status = RESPONSE_ERROR;
    
        if (attemps > 10){

          if (message[1] == READ)
            ROS_WARN("Error reading, after ten attempts the response was not received correctly");
          else if (message[1] == WRITE)
            ROS_WARN("Error writing, after ten attempts the response was not received correctly");
          else 
            ROS_ERROR("Error: WRITE or READ not found");

          break;
        }

        attemps++;
    }

    return status;

}


int LedsDriver::sendSignal(std::string signal){

    struct LedProperties led_properties;
    int status;

    led_properties = getLedProperties(signal);
    status = led_properties.status;

    if(status == SIGNAL_OK){
        
      if(writeSerialLeds(led_properties) == RESPONSE_OK){
          _current_signal = signal;
          if (signal == "NONE")
              status = SIGNAL_NONE_OK;
      }
      else
          status = SIGNAL_TIMEOUT;
    }

  return status;
}


std::string LedsDriver::setSignal(std::string signal, bool enable){

    int status;
    std::string message;

    if (enable == true && _current_signal != signal)
      status = sendSignal(signal);

    else if (enable == false && _current_signal == signal)
      status = sendSignal("NONE");

    else if (enable == true && _current_signal == signal)
      status = SIGNAL_ALREADY_TRUE;

    else if (enable == false && _current_signal != signal)

      if (findSignal(signal) > 0)
        status = SIGNAL_ALREADY_FALSE;
      else
        status = SIGNAL_NOT_FOUND;   

    message = printStatus(status, signal);

  return message;

}



std::string LedsDriver::printStatus(int status, std::string signal){

  std::string message;

  switch(status){
    case SIGNAL_OK: 
        message = "Signal " + signal + " set to True"; 
        ROS_DEBUG("%s", message.c_str());
        break;

    case SIGNAL_NONE_OK: 
        message = "Signal " + signal +" set to False"; 
        ROS_DEBUG("%s", message.c_str()); 
        break;

    case SIGNAL_TIMEOUT: 
        message = "Communication error, " + signal + " not set"; 
        ROS_WARN("%s", message.c_str()); 
        break;

    case SIGNAL_MALFORMED: 
        message = "Error signal malformed, signal " + signal + " not set"; 
        ROS_WARN("%s", message.c_str()); 
        break;

    case SIGNAL_NOT_FOUND: 
        message = "Error, signal " + signal + " does not exist"; 
        ROS_WARN("%s", message.c_str()); 
        break;

    case SIGNAL_ALREADY_TRUE: 
        message = "Signal " + signal + " is already true"; 
        ROS_WARN("%s", message.c_str()); 
        break; 

    case SIGNAL_ALREADY_FALSE: 
        message = "Signal " + signal + " is already false";
        ROS_WARN("%s", message.c_str()); 
        break; 

    case UNKNOWN_ERROR:  
        message = "Unknown error, signal " + signal + " not set"; 
        ROS_WARN("%s", message.c_str()); 
        break; 
  }

  return message;
}


void LedsDriver::run(){

    ros::Rate loop_rate(20);

    while (ros::ok()){

        ros::spinOnce();
        loop_rate.sleep();

    }

}
