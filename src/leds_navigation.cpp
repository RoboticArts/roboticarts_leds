#include <roboticarts_leds/leds_navigation.h>


LedsNavigation::LedsNavigation(ros::NodeHandle nodehandle):_nh(nodehandle){

  // Get name of this node
  nodeName = ros::this_node::getName();

  // Node ready
  ROS_INFO("%s node ready!", nodeName.c_str());

}


void LedsNavigation::showMsg(){

    ROS_INFO("HELLO WORLD");
}


void LedsNavigation::run(){

    ros::Rate loop_rate(20);

    while (ros::ok()){

        showMsg();
        ros::spinOnce();
        loop_rate.sleep();

    }

}
