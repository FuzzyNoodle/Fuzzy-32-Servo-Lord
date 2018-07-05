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
	
	//For oscilloscope signal testing
	pinMode(9, OUTPUT);
	analogWrite(9, 127);

	pinMode(POWER_MOSFET_PIN, OUTPUT);
	digitalWrite(POWER_MOSFET_PIN, HIGH);

	Wire.begin();
	Wire.setClock(400000); //Set i2c frequency to 400 kHz.


	ServoController.begin(); //required
	ServoController.setClockFrequency(27102244, 27380892);
	ServoController.setUpdateFrequency(100);

	ServoController.setEnvironmentTemperatureCelsius(25);
	ServoController.setPWM(1, 1500);
	while (1);


	//ServoController.setEnvironmentTemperature(25); //optional, used for temperature correction.
	Serial.println("Setup completed, starting POT reading.");
}

void loop()
{
	uint32_t readValue = 0;
	//Several samples are read to average out the fluctuations in analog reading.
	//Each analogRead() takes about 116 microseconds on 16 MHz Arduino UNO.
	for (uint8_t i = 0; i < 4 ; i++)
	{
		readValue += analogRead(MANUAL_POT_PIN); 
	}
	readValue >>= 2;


	uint16_t targetPWMValue = map(readValue, 0, 1023, 2300, 700);
	//ServoController.setPWMAll(targetPWMValue);
	ServoController.setPWM(1, targetPWMValue);
	ServoController.setPWM(2, targetPWMValue);
	ServoController.setPWM(17, targetPWMValue);
	//delay(5);
}


