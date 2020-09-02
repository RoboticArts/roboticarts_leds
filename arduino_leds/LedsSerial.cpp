
  #include "LedsSerial.h"


  void LedsSerial::begin(){
    
     Serial.begin(115200);
    Serial.setTimeout(200);
    
  }


  int LedsSerial::readResponse(uint8_t response[]){
     
     int status = -1;
     uint8_t res[MSG_SIZE] = {};
    
     if (Serial.available() >= MSG_SIZE) { 
       
         //Clear buffer
         memset(response,'0', MSG_SIZE);
  
         //Read data
         Serial.readBytes(res, MSG_SIZE);

         memcpy(response, res, MSG_SIZE);  // response = res         

         status = checkResponse(res);
        

    }

   return status;
  
  }

  void LedsSerial::writeResponse(int command){

      Serial.write(RESPONSE_OK);
      Serial.write(command);
      Serial.write(TAIL);
  }

  
  int LedsSerial::checkResponse(uint8_t response[]){

      int status = -1;
      
      if (response[0] == HEADER && response[11] == TAIL){
    
          if(response[2] >= FOWARD && response[2] <= CUSTOM_LED_D)
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



 bool LedsSerial::isFirstCommand(){

    bool isFirst = this->firstCommand;
    this->firstCommand = false;

    return isFirst;
 }


 bool LedsSerial::clearRequest(){

  bool request = this->clearLeds;
  this->clearLeds = false;

  return request;
 }


 void LedsSerial::getLedProperties(struct LedProperties *led_properties){

      led_properties->command  = _led_properties.command;
      led_properties->init_led = _led_properties.init_led;
      led_properties->end_led  = _led_properties.end_led;
      led_properties->color = _led_properties.color;
      led_properties->time = _led_properties.time;
      led_properties->direction = _led_properties.direction;
      
 }
  
 void LedsSerial::updateLedProperties(uint8_t response[]){

      _led_properties.command  = response[2];
      _led_properties.init_led = response[3];
      _led_properties.end_led  = response[4];
      _led_properties.color = ( (uint32_t(response[5]) << 16) | (uint32_t(response[6]) << 8) ) | uint32_t(response[7]);
      _led_properties.time =  (uint16_t(response[8]) << 8) | response[9];
      _led_properties.direction = response[10];

      if(response[2] <= CUSTOM_TURN)
        clearLeds = true;         
 }


void LedsSerial::sendCurrentCommand(int current_command){

    Serial.write(RESPONSE_OK);
    Serial.write(current_command);
    Serial.write(TAIL);

}

void LedsSerial::reset(){
  
  memset(_last_response,'0', MSG_SIZE);
  firstCommand = false;
  clearLeds = false;

}


 void LedsSerial::runSerial(int current_command){

  int state = -1;
  uint8_t res[MSG_SIZE] = {};
 
  state = readResponse(res);

  if(state == RESPONSE_OK){
  
    if(memcmp(_last_response, res, MSG_SIZE) != 0){
  
      firstCommand = true;
      memcpy(_last_response, res, MSG_SIZE);  // _last_response = res 

      if (res[1] == WRITE){
          updateLedProperties(res); 
          writeResponse(current_command);
      }
      if (res[1] == READ){
          sendCurrentCommand(current_command);              
      }
    }
    else{
      writeResponse(current_command);
    }   
    
  }
  else if (state == RESPONSE_ERROR || state == UNKNOWN_COMMAND){
    Serial.write(state);
  }

 }
        
