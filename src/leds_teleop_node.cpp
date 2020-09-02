#include <roboticarts_leds/leds_teleop.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "leds_teleop");
  ros::NodeHandle n;

  LedsTeleop leds_teleop(n);
  leds_teleop.run();
}