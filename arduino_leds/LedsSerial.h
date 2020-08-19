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
    
  private:

    int state = -1;
    int type = -1;
    uint8_t res[10] = {};
    uint8_t last_res[10] = {};
    bool firstCommand = false;
    bool clearLeds = false;

    int checkResponse(uint8_t response[]);
    int readResponse(uint8_t response[]);
    uint8_t getTypeResponse(uint8_t response[]);

   // Reponse types
    enum reponse_type {COMMAND_RESPONSE, LED_RESPONSE, RETURN_RESPONSE ,SAME_RESPONSE};


    
};

#endif
