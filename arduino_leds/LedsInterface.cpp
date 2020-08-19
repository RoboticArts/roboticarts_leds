


  #include "LedsInterface.h"

  
  LedsInterface::LedsInterface(int led_strip_size, int pin){
    
      this->leds = new Adafruit_NeoPixel (led_strip_size, pin, NEO_GRB + NEO_KHZ800);

  }

  void LedsInterface::_setPixelColor(int led, uint32_t color){
    
        leds->setPixelColor(led, color);
  }

  void LedsInterface::_show(){
    
         leds->show();  
  }


  void LedsInterface::begin(){
    
    leds->begin();
    leds->clear();
    leds->show();
  }

  void LedsInterface::update(){
    
     for(int led = 0; led < LED_STRIP_SIZE; led++){

          uint32_t color = this->led_buffer[led];
          _setPixelColor(led, color);
     }
     
     _show();  
    
  }

  void LedsInterface::clear(){

    
    for (int led = 0; led < LED_STRIP_SIZE; led++){
        this->led_buffer[led] = 0;
        _setPixelColor(led, CLEAR);
    }
    
    _show();
  }


  void LedsInterface::fillLeds(int start_led, int end_led, uint32_t color, bool enableBrightness){

    if (enableBrightness)
      color = this->getColorWithBrightness(color);
  
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

  uint32_t LedsInterface::getColorWithBrightness(uint32_t color){
  
    uint8_t r, g, b;
    uint32_t c;

    r = (uint8_t)(color >> 16);
    g = (uint8_t)(color >> 8);
    b = (uint8_t)(color);
  
    r = (r* LED_BRIGHTNESS) >> 8;
    g = (g* LED_BRIGHTNESS) >> 8;
    b = (b* LED_BRIGHTNESS) >> 8;
  
    c = ( (uint32_t(r) << 16) | (uint32_t(g) << 8) ) | uint32_t(b);
  
    return c; 
    
  }
