#include "leds_common.h"
#include "LedsRobot.h"


LedsRobot leds_robot(LED_STRIP_SIZE, LED_PIN);



void setup() {

  leds_robot.begin();

}

void loop() {

  leds_robot.run();
  
}
