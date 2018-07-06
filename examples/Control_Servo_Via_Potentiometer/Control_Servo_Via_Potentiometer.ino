#include "Fuzzy_32_Channel_Servo_Board.h"
#include <Wire.h>



#define MANUAL_POT_PIN A0
#define POWER_MOSFET_PIN 5


FuzzyServoBoard ServoController;


void setup()
{
	Serial.begin(115200);
	Serial.println("Fuzzy 32 Channel Servo Board");
	Serial.println("[Control Servo Via Potentiometer] example started.");
	
	

	Wire.begin();
	Wire.setClock(400000); //Set i2c frequency to 400 kHz.

	//The begin() method is required to initialize the servo controller object.
	ServoController.begin(); 
	
	/*
	The following methods are optional. 
	setClockFrequency(uint32_t clock1, uint32_t clock2)
	setUpdateFrequency(float targetUpdateFrequency)
	setEnvironmentTemperatureCelsius(int8_t degreesInCelsius)

	PCA9685 chip clock frequency value are obtained from the 
	Chip_Frequency_Calibrator sketch.
	Supplying the calculated clock using setClockFrequency() 
	method greatly improves the accuracy of the output signal.

	Users may/are recommended to set higher update frequency (update rate) 
	than standard 50Hz. A classic servo runs at 50 Hz, and the resolution is 
	around 4.88us at this update frequency.

	Increase the update frequency reduce both the response time and resolution, 
	however, if the update frequency goes too high, the servo might shake or 
	stop working.

	Approximate values for some update frequencies:
	Update frequency        Period      Resolution
	50     Hz.               20    ms.      4.88 us.
	60     Hz.               16.67 ms.      4.07 us.
	100    Hz.               10    ms.      2.71 us.
	122.07 Hz.               8.19  ms.      2.00 us.
	150    Hz.               6.67  ms.      1.63 us.
	200    Hz.               5     ms.      1.22 us.
	244.14 Hz.               4.1   ms.      1.00 us.

	User may set the current temperature of the PCA9685 chip if it is available.
	This method modifies the temperature correction array, which are applied on
	each PWM command call. The float calculations are only done during the 
	setEnvironmentTemperatureCelsius() call. So it may be called as required.

	The RC oscillator is affected by temperature, normally the clock frequency increases
	when the temperature increases. The measured coefficient is about 0.02% increase
	per degree C increase. An int array is used to reduce calculation time during run time.
	This temperature correction method is not very accurate, as the temperature control 
	method is inaccurate during measurement.
	*/
	ServoController.setClockFrequency(27102244, 27380892); //Optional, recommended. Value differs for each chip, obtained from the calibrator sketch.
	ServoController.setUpdateFrequency(100); //Optional, recommended to be set to a value 
	ServoController.setEnvironmentTemperatureCelsius(25); //Optional.

	//Enable the power MOSFET on the servo board.
	pinMode(POWER_MOSFET_PIN, OUTPUT);
	digitalWrite(POWER_MOSFET_PIN, HIGH);

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

	ServoController.setPWMAll(targetPWMValue);

	//Alternatively, users may command individual servo.
	//ServoController.setPWM(1, targetPWMValue);
	//ServoController.setPWM(2, targetPWMValue);
	
}


