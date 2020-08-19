#ifndef LEDS_BEHAVIOR_H
#define LEDS_BEHAVIOR_H


#include <Arduino.h>
#include "leds_common.h"
#include "LedsInterface.h"


class LedsBehavior : public LedsInterface{
  
  public:
    LedsBehavior(const int &led_strip_size, const int &pin);  
    void begin();
    void update();
    void reset();
    void setBehavior(struct LedProperties led_properties);
    void clearBehavior();

  private:
        
    // Used in blinkLeds
    float blink_time_now = 0;
    float blink_time_last = 0;
    bool  blink_state = LOW;
    
    // Used in shiftParts
    float shift_time_now = 0;
    float shift_time_last = 0;
    int   shift_counter = 0;

    // Used in setBehavior
    struct LedProperties last_led_properties;
 
    // Basic behaviors
    void paintLeds(int start_led, int end_led, uint32_t color, bool enableBrightness = true);
    void blinkLeds(int start_led, int end_led, uint32_t color, int blink_time = LED_BLINK_TIME, bool enableBrightness = true);
    void shiftParts(int start_parts[], int number_parts, int part_size,  uint32_t color, int shift_time, String direction);

    // Composed behaviors
    void turnLeds(String direction);
    void moveLeds(String direction);
    void omniLeds(String direction);
    void emergencyLeds();
    void bootingLeds();
    void readyLeds();
    void exitLeds();
               
    // Custom behaviors
    void customPaint(struct LedProperties* led_properties);
    void customBlink(struct LedProperties* led_properties);
    void customShift(struct LedProperties* led_properties);
    void customTurn(struct LedProperties* led_properties);

    // Custom led
    void customLed(int led, uint32_t color);


  }; 



#endif
