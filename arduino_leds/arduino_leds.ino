#include <Adafruit_NeoPixel.h>

#define LED_STRIP_SIZE 41
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



// Led strip buffer
uint32_t led_buffer[LED_STRIP_SIZE] = {};

// Used in updateLeds
float show_time_now = 0;
float show_time_last = 0;

// Used in blinkLeds
float blink_time_now = 0;
float blink_time_last = 0;
bool  blink_state = LOW;

// Used in shiftParts
float shift_time_now = 0;
float shift_time_last = 0;
int   shift_counter = 0;

// ROS command
int command = 0;
int last_command = 0;
uint32_t color_data  = 0;
uint32_t last_color_data  = 0;
int init_led = 0;
int end_led = 0;
int time_data = 0;
int direction_data = 0;

// Reponse types
enum reponse_type {COMMAND_RESPONSE, LED_RESPONSE, RETURN_RESPONSE ,SAME_RESPONSE};


struct LedProperties{

  uint8_t   command;
  uint8_t   init_led;
  uint8_t   end_led;
  uint32_t  color;
  uint16_t  time;
  uint8_t   direction;

};

LedProperties led_properties;
LedProperties last_led_properties;


Adafruit_NeoPixel leds(LED_STRIP_SIZE+4, 6, NEO_GRB + NEO_KHZ800);


void updateLeds(int refresh_time){

  
  show_time_now = millis();
  
  if (show_time_now - show_time_last >= refresh_time  ){

     show_time_last = show_time_now;

     for(int led = 0; led < LED_STRIP_SIZE; led++){

          uint32_t color = led_buffer[led];
          leds.setPixelColor(led, color);
     }
     
     leds.show();
         
  }
  
} 


uint32_t getColorWithBrightness(uint32_t color){
  
  uint8_t r, g, b;
  uint32_t c;
  
  if (color != last_color_data){
  
      r = (uint8_t)(color >> 16);
      g = (uint8_t)(color >> 8);
      b = (uint8_t)(color);
    
      r = (r* LED_BRIGHTNESS) >> 8;
      g = (g* LED_BRIGHTNESS) >> 8;
      b = (b* LED_BRIGHTNESS) >> 8;
    
      c = ( (uint32_t(r) << 16) | (uint32_t(g) << 8) ) | uint32_t(b);
  }
  else
    c = color;

  return c; 
  
}


void fillLeds(int start_led, int end_led, uint32_t color, bool enableBrightness = true){

  if (enableBrightness)
    color = getColorWithBrightness(color);



  if (start_led > end_led){

    for(int led=start_led; led < LED_STRIP_SIZE; led++){
      
        led_buffer[led] = color;
    
    }
    
    start_led = 0; 
  }

  for(int led=start_led; led <= end_led; led++){
      
      if (led < LED_STRIP_SIZE)
      
         led_buffer[led] = color;  
      
  }
 
}


void paintLeds(int start_led, int end_led, uint32_t color, bool enableBrightness = true){
  
  fillLeds(start_led, end_led, color, enableBrightness);  
  
}

void blinkLeds(int start_led, int end_led, uint32_t color, int blink_time = LED_BLINK_TIME, bool enableBrightness = true){

  blink_time_now = millis();

  if(blink_time_now - blink_time_last > blink_time){
    
    blink_time_last = blink_time_now;

    if (blink_state == LOW){

      fillLeds(start_led, end_led, color, enableBrightness);
      blink_state = HIGH;

    }
    else{
      
      fillLeds(start_led, end_led, CLEAR);
      blink_state = LOW;
    
    }
  }
  
}


void shiftParts(int start_parts[], int number_parts, int part_size,  uint32_t color, int shift_time, String direction){
  
  shift_time_now = millis();
  
  if(shift_time_now - shift_time_last > shift_time){
  
    shift_time_last = shift_time_now;

    int start_led;
    int end_led;

    if (shift_counter <= part_size){
      
        for (int part = 0; part < number_parts; part++){
            
            if (direction.equals("left")){
                start_led = start_parts[part];
                end_led = start_parts[part] + shift_counter;
            }
            else {
                end_led = start_parts[part] + part_size;
                start_led = end_led - shift_counter;
            }
            
            fillLeds(start_led, end_led, color);     
            
        }
        
        shift_counter++; 
    }

    else{
        fillLeds(0, LED_STRIP_SIZE , CLEAR);
        shift_counter = 0;
    }

         
        
  }
}


void turnLeds(String direction){

  int start_parts[4] = {TURN_MIN_1, TURN_MIN_2, TURN_MIN_3, TURN_MIN_4};
  int part_size =  TURN_SIZE;
  int number_parts = 4;
  
  if (direction.equals("left"))
    shiftParts(start_parts, number_parts, part_size, RED, LED_SHIFT_TIME, "left");
  else
    shiftParts(start_parts, number_parts, part_size, RED, LED_SHIFT_TIME, "right");
}


void moveLeds(String direction){
  
  if (direction.equals("foward"))   
    blinkLeds(MOVE_FOWARD_MIN ,MOVE_FOWARD_MAX, RED, 500); 
  else
    blinkLeds(MOVE_BACKWARD_MIN ,MOVE_BACKWARD_MAX, RED, 500);
    
}

void omniLeds(String direction){
  
  if (direction.equals("left"))
    blinkLeds(OMNI_LEFT_MIN , OMNI_LEFT_MAX , RED, 500); 

  else{
    blinkLeds(OMNI_RIGHT_MIN , OMNI_RIGHT_MAX , RED, 500);

  }
  
}

void emergencyLeds(){
  blinkLeds(0, LED_STRIP_SIZE-1, RED, 500);
}

void bootingLeds(){
  blinkLeds(0, LED_STRIP_SIZE-1, WHITE, 500);
}

void readyLeds(){
  paintLeds(0, LED_STRIP_SIZE-1, GREEN );
}

void exitLeds(){
  paintLeds(0, LED_STRIP_SIZE-1, BLUE );
}

void customPaint(struct LedProperties* led_properties){

  // Disable brightness ans set color
  paintLeds(led_properties->init_led, led_properties->end_led, led_properties->color, false);
  
  
}


void customBlink(struct LedProperties* led_properties){

  // Disable brightness ans set color and blik time
  blinkLeds(led_properties->init_led, led_properties->end_led, led_properties->color, led_properties->time, false);
}

void customShift(struct LedProperties* led_properties){
  
  int start_part[1] = {led_properties->init_led};
  int part_size =  led_properties->end_led - led_properties->init_led;
  int number_parts = 1;
  
  if (led_properties->direction)
    shiftParts(start_part, number_parts, part_size, led_properties->color, led_properties->time, "left");
  else
    shiftParts(start_part, number_parts, part_size, led_properties->color, led_properties->time, "right");

}


void customTurn(struct LedProperties* led_properties){

  
  int start_parts[4] = {TURN_MIN_1, TURN_MIN_2, TURN_MIN_3, TURN_MIN_4};
  int part_size =  TURN_SIZE;
  int number_parts = 4;
  
  if (led_properties->direction)
    shiftParts(start_parts, number_parts, part_size, led_properties->color, led_properties->time, "left");
  else
    shiftParts(start_parts, number_parts, part_size, led_properties->color, led_properties->time, "right");
  
}

void customLed(int led, uint32_t color){

  leds.setPixelColor(LED_STRIP_SIZE + led, color);
  leds.show();

}


void clearLeds(){

  for (int led = 0; led < LED_STRIP_SIZE; led++){
    led_buffer[led] = 0;
    leds.setPixelColor(led, CLEAR);
  }
  
  leds.show();

  show_time_now = 0;
  show_time_last = 0;
  
  blink_time_now = 0;
  blink_time_last = 0;
  blink_state = LOW;
  
  shift_time_now = 0;
  shift_time_last = 0;
  shift_counter = 0;


}

int checkResponse(uint8_t response[]){

  int status = -1;
  
  if (response[9] == TAIL){

      if(response[0] >= FOWARD && response[0] <= CUSTOM_LED_D)
          status = RESPONSE_OK;
      else
          status = UNKNOWN_COMMAND;
  }

  else
    status = RESPONSE_ERROR;

  return status;
}


int readResponse(uint8_t response[]){

   int status = -1;
  
   if (Serial.available() >= 10) { 

       //Clear buffer
       memset(response,'0', 10);

       //Read data
       Serial.readBytes(response, 10);

       //Check response
       status = checkResponse(response);
       Serial.print(status);


   }

  return status;
}

uint8_t getTypeResponse(uint8_t response[]){

  uint8_t type;
  
  if (response[0] == command)
  
      type = SAME_RESPONSE;
  
  else {  
  
      if (response[0] >= FOWARD && response[0] <= CUSTOM_SHIFT)
          type = COMMAND_RESPONSE; 
      else 
          type = LED_RESPONSE;

  }
  
  return type;
        
}


void setup() {

 leds.begin();
 leds.clear();
 leds.show();

 Serial.begin(115200);
 Serial.setTimeout(200);

 while (!Serial){};

}





void loop() {

 int state = -1;
 uint8_t res[10] = {};

 state = readResponse(res);

 if (state == RESPONSE_OK){

    uint8_t type = getTypeResponse(res);
    
    if (type == COMMAND_RESPONSE)
        clearLeds();
  
    if (type != SAME_RESPONSE || type != RETURN_RESPONSE){
        last_led_properties = led_properties;
        
        led_properties.command  = res[0];
        led_properties.init_led = res[1];
        led_properties.end_led  = res[2];
        led_properties.color = ( (uint32_t(res[3]) << 16) | (uint32_t(res[4]) << 8) ) | uint32_t(res[5]);
        led_properties.time =  (uint16_t(res[6]) << 8) | res[7];
        led_properties.direction = res[8];
    }

    if(type == RETURN_RESPONSE)
        Serial.print(led_properties.command); 
 }

 switch(led_properties.command){

  case FOWARD:       moveLeds("foward");     break;
  case BACKWARD:     moveLeds("backward");   break;
  case TURN_LEFT:    turnLeds("left");       break;
  case TURN_RIGHT:   turnLeds("right");      break;
  case OMNI_LEFT:    omniLeds("left");       break;
  case OMNI_RIGHT:   omniLeds("right");      break;
  case EMERGENCY:    emergencyLeds();        break;
  case CUSTOM_PAINT: customPaint(&led_properties); break;
  case CUSTOM_BLINK: customBlink(&led_properties); break;
  case CUSTOM_SHIFT: customShift(&led_properties); break;
  case CUSTOM_TURN:  customTurn (&led_properties); break;
  case CUSTOM_LED_A: customLed(LED_A, led_properties.color); led_properties = last_led_properties; break;
  case CUSTOM_LED_B: customLed(LED_B, led_properties.color); led_properties = last_led_properties; break;
  case CUSTOM_LED_C: customLed(LED_C, led_properties.color); led_properties = last_led_properties; break;
  case CUSTOM_LED_D: customLed(LED_D, led_properties.color); led_properties = last_led_properties; break;

 }


 updateLeds(20);
 
}
