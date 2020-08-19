#ifndef LEDS_INTERFACE_H
#define LEDS_INTERFACE_H


#include <Arduino.h>
#include "leds_common.h"
#include <Adafruit_NeoPixel.h>


class LedsInterface{ 

  public:
    LedsInterface(int led_strip_size, int pin);
    void begin();
    void update();
    void clear();
    void fillLeds(int start_led, int end_led, uint32_t color, bool enableBrightness = true);
    void _setPixelColor(int led, uint32_t color);
    void _show();

    
  private:

    Adafruit_NeoPixel* leds;
    
    uint32_t led_buffer[LED_STRIP_SIZE] = {};

    uint32_t getColorWithBrightness(uint32_t color);

};



#endif
