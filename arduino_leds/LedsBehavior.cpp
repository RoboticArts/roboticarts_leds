

  #include "LedsBehavior.h"


  LedsBehavior::LedsBehavior(const int &led_strip_size, const int &pin):
      LedsInterface(led_strip_size, pin){}


  void LedsBehavior::begin(){
      
     LedsInterface::begin();
  }

  void LedsBehavior::update(){
    
    LedsInterface::update();
  }
  

  // ==========================   Basic behaviors   ========================== //


  void LedsBehavior::paintLeds(int start_led, int end_led, uint32_t color, bool enableBrightness){
  
    LedsInterface::fillLeds(start_led, end_led, color, enableBrightness);  
  
  }
   
  void LedsBehavior::blinkLeds(int start_led, int end_led, uint32_t color, int blink_time, bool enableBrightness){
  
    blink_time_now = millis();
  
    if(blink_time_now - blink_time_last > blink_time){
      
      blink_time_last = blink_time_now;
  
      if (blink_state == LOW){
  
        LedsInterface::fillLeds(start_led, end_led, color, enableBrightness);
        blink_state = HIGH;
  
      }
      else{
        
        LedsInterface::fillLeds(start_led, end_led, CLEAR);
        blink_state = LOW;
      
      }
    }
    
  }
  
  
  void LedsBehavior::shiftParts(int start_parts[], int number_parts, int part_size,  uint32_t color, int shift_time, String direction){
    
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
              
              LedsInterface::fillLeds(start_led, end_led, color);     
              
          }
          
          shift_counter++; 
      }
  
      else{
          LedsInterface::fillLeds(0, LED_STRIP_SIZE , CLEAR);
          shift_counter = 0;
      }
  
           
          
    }
  }



  // ==========================   Composed behaviors   ========================== //


  
  void LedsBehavior::turnLeds(String direction){
  
    int start_parts[4] = {TURN_MIN_1, TURN_MIN_2, TURN_MIN_3, TURN_MIN_4};
    int part_size =  TURN_SIZE;
    int number_parts = 4;
    
    if (direction.equals("left"))
      shiftParts(start_parts, number_parts, part_size, RED, LED_SHIFT_TIME, "left");
    else
      shiftParts(start_parts, number_parts, part_size, RED, LED_SHIFT_TIME, "right");
  }
  
  
  void LedsBehavior::moveLeds(String direction){
    
    if (direction.equals("foward"))   
      blinkLeds(MOVE_FOWARD_MIN ,MOVE_FOWARD_MAX, RED, 500); 
    else
      blinkLeds(MOVE_BACKWARD_MIN ,MOVE_BACKWARD_MAX, RED, 500);
      
  }
  
  void LedsBehavior::omniLeds(String direction){
    
    if (direction.equals("left"))
      blinkLeds(OMNI_LEFT_MIN , OMNI_LEFT_MAX , RED, 500); 
  
    else{
      blinkLeds(OMNI_RIGHT_MIN , OMNI_RIGHT_MAX , RED, 500);
  
    }
    
  }
  
  void LedsBehavior::emergencyLeds(){
    blinkLeds(0, LED_STRIP_SIZE-1, RED, 500);
  }
  
  void LedsBehavior::bootingLeds(){
    blinkLeds(0, LED_STRIP_SIZE-1, WHITE, 500);
  }
  
  void LedsBehavior::readyLeds(){
    paintLeds(0, LED_STRIP_SIZE-1, GREEN );
  }
  
  void LedsBehavior::exitLeds(){
    paintLeds(0, LED_STRIP_SIZE-1, BLUE );
  }


  // ==========================   Custom behaviors   ========================== //

  
  void LedsBehavior::customPaint(struct LedProperties* led_properties){
  
    // Disable brightness ans set color
    paintLeds(led_properties->init_led, led_properties->end_led, led_properties->color, false);
    
    
  }
  
  
  void LedsBehavior::customBlink(struct LedProperties* led_properties){
  
    // Disable brightness ans set color and blik time
    blinkLeds(led_properties->init_led, led_properties->end_led, led_properties->color, led_properties->time, false);
  }
  
  void LedsBehavior::customShift(struct LedProperties* led_properties){
    
    int start_part[1] = {led_properties->init_led};
    int part_size =  led_properties->end_led - led_properties->init_led;
    int number_parts = 1;
    
    if (led_properties->direction)
      shiftParts(start_part, number_parts, part_size, led_properties->color, led_properties->time, "left");
    else
      shiftParts(start_part, number_parts, part_size, led_properties->color, led_properties->time, "right");
  
  }
  
  
  void LedsBehavior::customTurn(struct LedProperties* led_properties){
  
    
    int start_parts[4] = {TURN_MIN_1, TURN_MIN_2, TURN_MIN_3, TURN_MIN_4};
    int part_size =  TURN_SIZE;
    int number_parts = 4;
    
    if (led_properties->direction)
      shiftParts(start_parts, number_parts, part_size, led_properties->color, led_properties->time, "left");
    else
      shiftParts(start_parts, number_parts, part_size, led_properties->color, led_properties->time, "right");
    
  }


  // ======================= Led Behavior ========================= //
  
  void LedsBehavior::customLed(int led, uint32_t color){

    // Provisional: access to low level leds
    LedsBehavior::_setPixelColor(LED_STRIP_SIZE + led, color);
    LedsBehavior::_show();

   
  }
  
  
  void LedsBehavior::clearBehavior(){
  
    clear();
    reset();
  
  }

  void LedsBehavior::reset(){
  
    blink_time_now = 0;
    blink_time_last = 0;
    blink_state = LOW;
    
    shift_time_now = 0;
    shift_time_last = 0;
    shift_counter = 0;
  
  }


// ======================= Select Behavior ========================= // 


  void LedsBehavior::setBehavior(struct LedProperties led_properties){

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
      case BOOTING: bootingLeds(); break;
      case READY: readyLeds(); break;
      case EXIT: exitLeds(); break;
     }
      
  }
