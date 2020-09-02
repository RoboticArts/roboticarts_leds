#include <roboticarts_leds/leds_mux.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "leds_mux");
  ros::NodeHandle n;

  LedsMux leds_mux(n);
  leds_mux.run();


}
