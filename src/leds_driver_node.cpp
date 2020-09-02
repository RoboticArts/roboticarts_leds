#include <roboticarts_leds/leds_driver.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "leds_driver");
  ros::NodeHandle n;

  LedsDriver leds_driver(n);
  leds_driver.run();
}