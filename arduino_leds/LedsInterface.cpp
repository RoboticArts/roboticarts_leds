


  #include "LedsInterface.h"

  
  LedsInterface::LedsInterface(int led_strip_size, int pin){
    
      this->leds = new Adafruit_NeoPixel (led_strip_size, pin, NEO_GRB + NEO_KHZ800);

  }


  void LedsInterface::begin(){
    
    leds->begin();
    leds->clear();
    leds->show();
  }

  void LedsInterface::update(){
    
     for(int led = 0; led < LED_STRIP_SIZE; led++){

          uint32_t color = this->led_buffer[led];
          leds->setPixelColor(led, color);
     }
     
     leds->show();
    
  }

  void LedsInterface::clear(){

    for (int led = 0; led < LED_STRIP_SIZE; led++)
        this->led_buffer[led] = 0;

  }


  void LedsInterface::fillLeds(int start_led, int end_led, uint32_t color){
  
    if (start_led > end_led){
  
      for(int led=start_led; led < LED_STRIP_SIZE; led++){
        
          this->led_buffer[led] = color;
      
      }
      
      start_led = 0; 
    }
  
    for(int led=start_led; led <= end_led; led++){
        
        if (led < LED_STRIP_SIZE)
        
           this->led_buffer[led] = color;  
        
    }
 
  }
