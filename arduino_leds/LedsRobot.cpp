
  #include "LedsRobot.h"

  LedsRobot::LedsRobot(int led_strip_size, int pin){

     this->leds_behavior = new LedsBehavior(led_strip_size, pin);
     this->leds_serial = new LedsSerial();
  }

  void LedsRobot::updateLeds(int refresh_time){
  
    show_time_now = millis();
    
    if (show_time_now - show_time_last >= refresh_time  ){
  
       show_time_last = show_time_now;     
        leds_behavior->update();
    }
    
  } 
  
  
  
  void LedsRobot::resetLeds(){
  
    leds_serial->reset();
    leds_behavior->reset(); 
    led_properties = {};   
  }
  
  void LedsRobot::begin(){
    
    leds_serial->begin(); // Input
    leds_behavior->begin(); // Output  
    
  }
  
  void LedsRobot::run(){

    leds_serial->runSerial(led_properties.command);
    
    switch(arduino_state){
  
      case BOOTING:
  
          led_properties.command = BOOTING;
  
          if(Serial){
             arduino_state = READY;
             led_properties.command = READY;
          }
          
          break;
      
      case READY:
    
          if(leds_serial->isFirstCommand())
            arduino_state = RUN;
  
          if(!Serial){
            arduino_state = EXIT;
            led_properties.command = EXIT;
          }
          
          break;
  
      case RUN:

          leds_serial->getLedProperties(&led_properties);
  
          if(leds_serial->clearRequest())
              leds_behavior->clearBehavior();
          
          if(!Serial){
             arduino_state = EXIT;
             led_properties.command = EXIT;
          }
          
          break;
  
      case EXIT:
  
          if(Serial){
            resetLeds();
            arduino_state = READY;
            led_properties.command = READY;
          }
  
          break;
    }
   
   leds_behavior->runBehavior(led_properties);
   updateLeds(20);
     
  }
