#include "Fuzzy_32_Channel_Servo_Board.h"
#include <Wire.h>


#define SERVO_POWER_PIN 26
#define MANUAL_POT_PIN A0
#define POWER_MOSFET_PIN 5


FuzzyServoBoard ServoController;


void setup()
{
	Serial.begin(115200);
	Serial.println("Fuzzy 32 Channel Servo Board");
	Serial.println("*** Control Servo Via Potentiometer *** example started.");
	pinMode(9, OUTPUT);
	analogWrite(9, 127);

	pinMode(POWER_MOSFET_PIN, OUTPUT);
	digitalWrite(POWER_MOSFET_PIN, HIGH);

	Wire.begin();
	Wire.setClock(400000); //set i2c frequency to 400k hz

	ServoController.init(); //required
	ServoController.setPrescale(121);
	//ServoController.setPrescale(0x79,0x79);//may be required, measured value. Default value is 0x1A for both chips.
	//ServoController.setPulseWidthCorrection(4, -53);//required, values need to be measured by an oscilloscope. It differs from chip to chip. 


	//ServoController.setEnvironmentTemperature(25); //optional, used for temperature correction.

}

void loop()
{
	uint16_t readValue = analogRead(MANUAL_POT_PIN);
	uint16_t commandedPWMValue = map(readValue, 0, 1023, 700, 2300);
	//ServoController.setPWMAll(commandedPWMValue);
	ServoController.setPWM(1, commandedPWMValue);
	ServoController.setPWM(2, commandedPWMValue);
	ServoController.setPWM(17, commandedPWMValue);
	delay(100);
}


