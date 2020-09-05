#ifndef LEDS_BRIGHTNESS_H
#define LEDS_BRIGHTNESS_H

#include <Arduino.h>
#include "leds_common.h"
#include "LedsInterface.h"

#define CUSTOM_LED_A 0x0D
#define CUSTOM_LED_B 0x0E
#define CUSTOM_LED_C 0x0F
#define CUSTOM_LED_D 0x10

class LedBrightness : public LedsInterface{
  
  public:
      LedBrightness(const int &led_strip_size, const int &pin); 
      void begin();
      void provisionalLedBrightness(uint8_t response[]);
      void reset();
      
  private:
  
      uint8_t last_response[MSG_SIZE] = {};
      int checkResponse(uint8_t response[]);
};






#endif
