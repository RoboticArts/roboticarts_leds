

  #include "LedBrightness.h"



  LedBrightness::LedBrightness(const int &led_strip_size, const int &pin):
    LedsInterface(led_strip_size, pin){}


  void LedBrightness::begin(){
    
    LedsInterface::begin();   
    
  }

  void LedBrightness::reset(){
    
    static bool flag = true;
    
    if(!Serial && flag == true){
      flag = false;
      LedsInterface::clear();
      LedsInterface::update();
    }

    if (Serial && flag == false)
      flag = true;
      
  }

    
  int LedBrightness::checkResponse(uint8_t response[]){

      int status = -1;
      
      if (response[0] == HEADER && response[11] == TAIL){
    
          if(response[2] >= CUSTOM_LED_A && response[2] <= CUSTOM_LED_D)
              status = RESPONSE_OK;
          else
              status = UNKNOWN_COMMAND;

          if(response[1] != READ && response[1] != WRITE)
              status = UNKNOWN_COMMAND;
      }
    
      else
        status = RESPONSE_ERROR;
    
      return status;
  } 


  void LedBrightness::provisionalLedBrightness(uint8_t response[]){

    reset();
      
    int state = checkResponse(response);

    if(state == RESPONSE_OK){
      
      Serial.write(RESPONSE_OK);
      Serial.write(response[2]);
      Serial.write(TAIL);
    
      if(memcmp(last_response, response, MSG_SIZE) != 0){
    
        memcpy(last_response, response, MSG_SIZE);  // _last_response = res 
  
        if (response[1] == WRITE){
          
          uint32_t red_color = uint32_t(response[5]) << 16;
          
          switch(response[2]){

            case CUSTOM_LED_A: LedsInterface::fillLeds(0,0, red_color); break;
            case CUSTOM_LED_B: LedsInterface::fillLeds(1,1, red_color); break;
            case CUSTOM_LED_C: LedsInterface::fillLeds(2,2, red_color); break;
            case CUSTOM_LED_D: LedsInterface::fillLeds(3,3, red_color); break;
          }

          LedsInterface::update();
          
        }
      }
    }
  }
