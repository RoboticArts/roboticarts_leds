#ifndef LEDS_COMMON_H
#define LEDS_COMMON_H


#define LED_STRIP_SIZE 41
#define LED_PIN 6
#define LED_BRIGHTNESS 20
#define LED_BLINK_TIME 1000
#define LED_SHIFT_TIME 100
     
#define TURN_MIN_1 0
#define TURN_MIN_2 11
#define TURN_MIN_3 21
#define TURN_MIN_4 32
#define TURN_SIZE 10

#define OMNI_LEFT_MIN 34
#define OMNI_LEFT_MAX 9
#define OMNI_RIGHT_MIN 12
#define OMNI_RIGHT_MAX 31

#define MOVE_FOWARD_MIN 24
#define MOVE_FOWARD_MAX 40
#define MOVE_BACKWARD_MIN 3  
#define MOVE_BACKWARD_MAX 18 

#define LED_A 0
#define LED_B 1
#define LED_C 2
#define LED_D 3


#define RED   0xFF0000
#define GREEN 0x00FF00
#define BLUE  0x0000FF
#define WHITE 0xFFFFFF
#define CLEAR 0x000000

// Responses
#define RESPONSE_OK 0x01
#define RESPONSE_ERROR 0x02
#define UNKNOWN_COMMAND 0x03

#define MSG_SIZE 12
#define HEADER 0x99
#define WRITE 0x01
#define READ 0x02
#define TAIL 0x23

// Commands
#define FOWARD 0x01
#define BACKWARD 0x02
#define TURN_LEFT 0x03
#define TURN_RIGHT 0x04
#define OMNI_LEFT 0x05
#define OMNI_RIGHT 0x06
#define EMERGENCY 0x07
#define CUSTOM_PAINT 0x08
#define CUSTOM_BLINK 0x09
#define CUSTOM_SHIFT 0x0A
#define CUSTOM_TURN 0x0B
#define CURRENT_COMMAND 0x0C

#define CUSTOM_LED_A 0x0D
#define CUSTOM_LED_B 0x0E
#define CUSTOM_LED_C 0x0F
#define CUSTOM_LED_D 0x10

#define BOOTING 0xF1
#define READY 0xF2
#define RUN 0xF3
#define EXIT 0xF4


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
