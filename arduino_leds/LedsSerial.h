#ifndef LEDS_SERIAL_H
#define LEDS_SERIAL_H


#include <Arduino.h>
#include "leds_common.h"


class LedsSerial{ 

  public:

    bool isFirstCommand();
    bool clearRequest();
    void reset();
    void getLedProperties(struct LedProperties *led_properties);
    void begin();
    void runSerial(int current_command);
    
  private:

     
    struct LedProperties _led_properties;
    uint8_t _last_response[MSG_SIZE] = {};
    bool firstCommand = false;
    bool clearLeds = false;

    int checkResponse(uint8_t response[]);
    int readResponse(uint8_t response[]);
    void writeResponse(int command);

    void updateLedProperties(uint8_t response[]);
    void sendCurrentCommand(int current_command);


};

#endif
