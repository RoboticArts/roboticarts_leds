#include <roboticarts_leds/roboticarts_leds.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "roboticarts_leds");
  ros::NodeHandle n;

  RoboticartsLeds roboticarts_leds(n);
  roboticarts_leds.run();
}