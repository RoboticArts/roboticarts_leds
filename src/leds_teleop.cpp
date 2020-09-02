#include <roboticarts_leds/leds_teleop.h>

LedsTeleop::LedsTeleop(ros::NodeHandle nodehandle):_nh(nodehandle){

  // Get name of this node
  nodeName = ros::this_node::getName();

  // Enable ROS_DEBUG output
  //if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
  //    ros::console::notifyLoggerLevelsChanged();

  // Get ros params
  getRosParams();

  vel_sub = _nh.subscribe(cmd_vel_topic, 1, &LedsTeleop::velCallback, this);
  leds_client = _nh.serviceClient<roboticarts_leds::SetLeds>(cmd_led_service);

  //Wait service
  ros::service::waitForService(cmd_led_service);

  // Node ready
  ROS_INFO("%s node ready!", nodeName.c_str());

}



void LedsTeleop::getRosParams(){

  ros::param::param<std::string>(nodeName + "/cmd_vel_topic", cmd_vel_topic , "cmd_vel");
  ros::param::param<std::string>(nodeName + "/cmd_led_service", cmd_led_service, "roboticarts_teleop/cmd_leds");
  ros::param::param<float>(nodeName + "/velocity_threshold", velocity_threshold , 0.05);

}



void LedsTeleop::velCallback(const geometry_msgs::Twist::ConstPtr& msg){

    this->vel_x = msg->linear.x;
    this->vel_y = msg->linear.y;
    this->vel_z = msg->angular.z;  

    ROS_DEBUG("[ Linear X: %.2f ]  [Linear Y: %.2f]  [Angular Z: %.2f]", vel_x, vel_y, vel_z);  
}


std::string LedsTeleop::getRobotMovement(){

	float vel_x = this->vel_x;
	float vel_y = this->vel_y;
	float vel_z = this->vel_z;

	bool emergency_stop = true;
	float vel = this -> velocity_threshold;
    std::string state;
    bool direction;
   
	if (emergency_stop == true){

        // Get direction
        if (vel_x >= 0)
            direction = true;
        else
            direction = false;


        if (fabs(vel_y) > vel){
            
            // Right shift
            if (vel_y > vel)
                state = "OMNI_LEFT";

            // Left shift
            else if (vel_y < -vel)
                state = "OMNI_RIGHT";
        }

        else if (fabs(vel_z) > vel){

            // Forward turning
            if (direction == true){

                if (vel_z > vel)
                    state = "TURN_LEFT";

                else if (vel_z < -vel)
                    state = "TURN_RIGHT";

            }

            //Backward turning
            else{

                if (vel_z > vel)
                    //state = "turn_left_inverse";
                    state = "TURN_RIGHT";

                else if (vel_z < -vel)
                    //state = "turn_right_inverse";
                    state = "TURN_LEFT";
            }
        }

        else{

            // Only forward
            if (vel_x > vel)
                state = "FORWARD";

            // Only backward
            else if (vel_x < -vel)
                state = "BACKWARD";

            else
                state = "STOP";
        }

    }
    else
        state = "EMERGENCY";


	return state;


}

void LedsTeleop::setLedSignal(std::string signal, bool enable){

    roboticarts_leds::SetLeds leds_srv;

    leds_srv.request.signal = signal;
    leds_srv.request.enable = enable;

    if (leds_client.call(leds_srv)){

        if (leds_srv.response.success)
            ROS_INFO("[%s]: %s", nodeName.c_str(), leds_srv.response.message.c_str());
        else
            ROS_WARN("[%s]: %s", nodeName.c_str(), leds_srv.response.message.c_str());
        
    }

    else
        ROS_ERROR("Failed to call service %s ", cmd_led_service.c_str());

}


void LedsTeleop::runLedsTeleop(){

    movement = getRobotMovement();
    
    if (movement != last_movement){

        last_movement = movement;
        setLedSignal(movement, true);

        ROS_INFO("Active: %s", movement.c_str());
        ROS_INFO("Deactivate: %s", last_movement.c_str());
        ROS_INFO("-------------------------");

    }
    


}

void LedsTeleop::run(){

    ros::Rate loop_rate(20);

    while (ros::ok()){

        runLedsTeleop();
        ros::spinOnce();
        loop_rate.sleep();

    }

}


