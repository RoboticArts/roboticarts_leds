#ifndef LEDS_ROBOT_H
#define LEDS_ROBOT_H




#include <Arduino.h>
#include "leds_common.h"
#include "LedsBehavior.h"
#include "LedsSerial.h"

class LedsRobot{ 

  public:

    LedsRobot(int led_strip_size, int pin);
    void begin();
    void run();

  private:

    LedsBehavior* leds_behavior;
    LedsSerial* leds_serial;
    
    // Used in updateLeds
    float show_time_now = 0;
    float show_time_last = 0;

    // Machine state
    uint8_t arduino_state = BOOTING;

    // Led properties
    LedProperties led_properties;
    LedProperties last_led_properties;
  
    void updateLeds(int refresh_time);
    void resetLeds();
  
};

#endif 
