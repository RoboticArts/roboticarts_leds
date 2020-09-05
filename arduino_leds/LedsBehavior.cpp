

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


  void LedsBehavior::paintLeds(int start_led, int end_led, uint32_t color){
  
    LedsInterface::fillLeds(start_led, end_led, color);  
  
  }
   
  void LedsBehavior::blinkLeds(int start_led, int end_led, uint32_t color, int blink_time){
  
    blink_time_now = millis();
  
    if(blink_time_now - blink_time_last > blink_time){
      
      blink_time_last = blink_time_now;
  
      if (blink_state == LOW){
  
        LedsInterface::fillLeds(start_led, end_led, color);
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
    paintLeds(led_properties->init_led, led_properties->end_led, led_properties->color);
    
    
  }
  
  
  void LedsBehavior::customBlink(struct LedProperties* led_properties){
  
    // Disable brightness ans set color and blik time
    blinkLeds(led_properties->init_led, led_properties->end_led, led_properties->color, led_properties->time);
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
  
    int zone = led_properties->end_led - led_properties->init_led;
    int side = floor(LED_STRIP_SIZE/4);
    int start_part;

    if (zone%2 == 0 && side%2 == 0)
       start_part = side/2 - zone/2;
    
    if (zone%2 == 0 && side%2 != 0)
       start_part = ceil(side/2) - zone/2;  

    if (zone%2 != 0 && side%2 == 0)
       start_part = side/2 - ceil(zone/2); 

    if (zone%2 != 0 && side%2 != 0)
       start_part = ceil(side/2) - zone/2;
       
    
    int start_parts[4] = {start_part, start_part+side, start_part+side*2, start_part+side*3};
    int part_size =  zone;
    int number_parts = 4;
    
    if (led_properties->direction)
      shiftParts(start_parts, number_parts, part_size, led_properties->color, led_properties->time, "left");
    else
      shiftParts(start_parts, number_parts, part_size, led_properties->color, led_properties->time, "right");
    
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


  void LedsBehavior::runBehavior(struct LedProperties led_properties){

     if (isNewBehavior(led_properties))
         clearBehavior();

     switch(led_properties.command){
    
      case CUSTOM_PAINT: customPaint(&led_properties); break;
      case CUSTOM_BLINK: customBlink(&led_properties); break;
      case CUSTOM_SHIFT: customShift(&led_properties); break;
      case CUSTOM_TURN:  customTurn (&led_properties); break;
      case BOOTING:  bootingLeds(); break;
      case READY:  readyLeds(); break;
      case EXIT:  exitLeds(); break;
     }
      
  }


  void LedsBehavior::clearBehavior(){

    LedsInterface::clear();
    reset();
  
  }

  bool LedsBehavior::isNewBehavior (struct LedProperties led_properties){
      
    bool isNew = false;

      if (led_properties.command != last_led_properties.command) isNew = true;
      if (led_properties.init_led != last_led_properties.init_led) isNew = true;
      if (led_properties.end_led != last_led_properties.end_led) isNew = true;
      if (led_properties.color != last_led_properties.color) isNew = true;
      if (led_properties.time != last_led_properties.time) isNew = true;
      if (led_properties.direction != last_led_properties.direction) isNew = true;
      
      if (isNew)
         last_led_properties = led_properties;
            
      return isNew;
       
  }
