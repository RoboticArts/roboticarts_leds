#include <serial/serial.h>
#include <iostream>
#include <unistd.h>

// Comands
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
#define TAIL 0x23;

serial::Serial ser;

struct LedProperties{

	uint8_t  command;
	uint8_t  init_led;
	uint8_t  end_led;
	uint8_t  color_R;
	uint8_t  color_G;
	uint8_t  color_B;
	uint16_t data_time;
	uint8_t  direction;

};

struct LedProperties led_properties;


void buildMessage(uint8_t *message, struct LedProperties led_properties){

	message[0] = led_properties.command;
	message[1] = led_properties.init_led;             // Start led 
	message[2] = led_properties.end_led;              // End led
	message[3] = led_properties.color_R;              // Color R
	message[4] = led_properties.color_G;              // Color G
	message[5] = led_properties.color_B;              // Color B
	message[6] = led_properties.data_time >> 8;      // Time MSB
	message[7] = led_properties.data_time & 0x00FF;  // Time LSB
	message[8] = led_properties.direction;            // Direction
	message[9] = TAIL; // EOL

}


int main (int argc, char** argv){

	ser.setPort("/dev/ttyACM0");
	ser.setBaudrate(115200);

	serial::Timeout to = serial::Timeout::simpleTimeout(1000);
	ser.setTimeout(to);
	
	ser.open();

	usleep(1000000);
	uint8_t message[10] = {};

	led_properties.command = EMERGENCY;
	led_properties.init_led = 0;
	led_properties.end_led = 20;
	led_properties.color_R = 50;
	led_properties.color_G = 25;
	led_properties.color_B = 0;
	led_properties.data_time = 100;
	led_properties.direction = 0;

	buildMessage(message, led_properties);
	//buildMessage(message, CUSTOM_LED_B);
	//buildMessage(message, CUSTOM_SHIFT);

	ser.write(message, 10);

	
	usleep(100000);

	if(ser.available()){
		
		uint8_t response[1] = {};

		ser.read(response, 1);
		std::cout << response[0] << std::endl;
		//std::cout << response[1] << std::endl;
		
		/*
		std::cout << std::hex << +response[0];
		std::cout << std::hex << +response[1];
		std::cout << std::hex << +response[2];
		std::cout << std::hex << +response[3];
		std::cout << std::hex << +response[4];
		*/
	}

	while(1){
		usleep(200000);
	}
	
}