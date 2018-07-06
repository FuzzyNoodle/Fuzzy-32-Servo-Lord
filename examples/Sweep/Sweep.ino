#include "Fuzzy_32_Channel_Servo_Board.h"
#include <Wire.h>

/*
This example sketch sweeps all servo channels between 700 and 2300 microseconds.
*/

//The power MOSFET control pin (labeled "PIN") is controlled by this arduino pin
#define POWER_MOSFET_PIN  5

//Create the servo controller object to control up to 32 servos.
FuzzyServoBoard ServoController;

int servoPosition;

void setup()
{
	Serial.begin(115200);
	Serial.println("Fuzzy 32 Channel Servo Board");
	Serial.println("[Sweep] example started.");

	Wire.begin();
	Wire.setClock(400000); //Set i2c frequency to 400 kHz.

	//The begin() method is required to initialize the servo controller object.
	ServoController.begin();

	//Please refer to Chip_Frequency_Calibrator sketch for more advanced settings.

	//Enable the power MOSFET on the servo board.
	pinMode(POWER_MOSFET_PIN, OUTPUT);
	digitalWrite(POWER_MOSFET_PIN, HIGH);

}

void loop()
{
	
	for (servoPosition = 700; servoPosition <= 2300; servoPosition += 25) 
	{ 
		ServoController.setPWMAll(servoPosition);
		delay(10);
	}
	for (servoPosition = 2300; servoPosition >= 700; servoPosition -= 25) 
	{ 
		ServoController.setPWMAll(servoPosition);
		delay(10);
	}
}
