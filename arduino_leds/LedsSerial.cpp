
  #include "LedsSerial.h"


  void LedsSerial::begin(){
    
     Serial.begin(115200);
    Serial.setTimeout(200);
    
  }


  int LedsSerial::readResponse(uint8_t response[]){

     int status = -1;
    
     if (Serial.available() >= 10) { 
        
         //Clear buffer
         memset(response,'0', 10);
  
         //Read data
         Serial.readBytes(response, 10);

         status = checkResponse(res);
         Serial.print(status);

    }

   return status;

  }

  
  int LedsSerial::checkResponse(uint8_t response[]){

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


  
  uint8_t LedsSerial::getTypeResponse(uint8_t response[]){
  
    int type=-1;
       
    if (memcmp(last_res, response, 10) == 0){
        type = SAME_RESPONSE;
    }

    if (type != SAME_RESPONSE){
  
        if (response[0] >= FOWARD && response[0] <= CUSTOM_TURN)
            type = COMMAND_RESPONSE; 
        else 
            type = LED_RESPONSE;
    
        memcpy(last_res, response, 10);  // last_res = response
    }
     
    return type;

 }

 bool LedsSerial::isFirstCommand(){

    bool isFirst = this->firstCommand;
    this->firstCommand = false;
    
    return isFirst;
 }


 bool LedsSerial::clearRequest(){

  bool request;

  if(this->clearLeds == true){
      request = true;
      
  }
  else
      request = false;
  
  this->clearLeds = false;
  
  return request; 
 }
  
 void LedsSerial::getLedProperties(struct LedProperties *led_properties){
    
     state = readResponse(res);
    
     if (state == RESPONSE_OK){
    
        type = this->getTypeResponse(res);
       
        if (type == COMMAND_RESPONSE)
            this->clearLeds = true;
      
        if (type != SAME_RESPONSE || type != RETURN_RESPONSE){

            if (type == COMMAND_RESPONSE)
                firstCommand = true;
            
            led_properties->command  = res[0];
            led_properties->init_led = res[1];
            led_properties->end_led  = res[2];
            led_properties->color = ( (uint32_t(res[3]) << 16) | (uint32_t(res[4]) << 8) ) | uint32_t(res[5]);
            led_properties->time =  (uint16_t(res[6]) << 8) | res[7];
            led_properties->direction = res[8];
        }
    
        if(type == RETURN_RESPONSE)
            Serial.print(led_properties->command); 

     }       
 }


  void LedsSerial::reset(){
    
    state = -1;
    type = -1;
    memset(res,'0', 10);
    memset(last_res,'0', 10);
    firstCommand = true;
    clearLeds = false;
  }
 

 
        
