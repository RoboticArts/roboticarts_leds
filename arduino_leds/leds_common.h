#ifndef LEDS_COMMON_H
#define LEDS_COMMON_H

// Led strip
#define LED_STRIP_SIZE 41
#define LED_PIN 6

// Colors
#define RED   0x140000
#define GREEN 0x001400
#define BLUE  0x000014
#define WHITE 0x141414 
#define CLEAR 0x000000

// States
#define BOOTING 0xF1
#define READY 0xF2
#define RUN 0xF3
#define EXIT 0xF4

// Response utils
#define RESPONSE_OK 0x01
#define RESPONSE_ERROR 0x02
#define UNKNOWN_COMMAND 0x03

// Message utils
#define MSG_SIZE 12
#define HEADER 0x99
#define WRITE 0x01
#define READ 0x02
#define TAIL 0x23

// Commands
#define CUSTOM_PAINT 0x08
#define CUSTOM_BLINK 0x09
#define CUSTOM_SHIFT 0x0A
#define CUSTOM_TURN 0x0B
#define CURRENT_COMMAND 0x0C



// Led properties
struct LedProperties{

  uint8_t   command;
  uint8_t   init_led;
  uint8_t   end_led;
  uint32_t  color;
  uint16_t  time;
  uint8_t   direction;

};




#endif
