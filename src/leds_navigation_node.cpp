 #include <roboticarts_leds/leds_navigation.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "leds_navigation");
  ros::NodeHandle n;

  LedsNavigation leds_navigation(n);
  leds_navigation.run();
}