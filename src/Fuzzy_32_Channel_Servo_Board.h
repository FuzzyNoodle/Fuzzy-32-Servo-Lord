/*
 Name:		Fuzzy_32_Channel_Servo_Board.h
 Created:	6/12/2018 4:39:59 PM
 Author:	georgychen
 Editor:	http://www.visualmicro.com
*/

#ifndef _Fuzzy_32_Channel_Servo_Board_h
#define _Fuzzy_32_Channel_Servo_Board_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

/*Debug switch. */
#define SERIAL_DEBUG //Enable the SERIAL_DEBUG to output debug messages other than servo position setting.
//#define VERBOSE_SERIAL_DEBUG //This debug switch enables debug messages for servo position setting

//chip registers and default values
#define PCA9685_CHIP_1 1
#define PCA9685_CHIP_2 2
#define PCA9685_MODE1 0x00
#define PCA9685_MODE2 0x01
#define PCA9685_SWRST_ADDR 0X00
#define PCA9685_SWRST_ACK 0x06
#define PCA9685_SWRST_NOACK 0x00
#define PCA9685_AI 0x20
#define PCA9685_ALLCALL 0x01
#define PCA9685_PRESCALE 0xFE
#define PCA9685_SLEEP 0x10
#define PCA9685_ALLCALLADR 0x70
#define PCA9685_ALL_LED_ON_L 0xFA
#define PCA9685_ALL_LED_ON_H 0xFB
#define PCA9685_ALL_LED_OFF_L 0xFC
#define PCA9685_ALL_LED_OFF_H 0xFD
#define PCA9685_DEFAULT_PRESCALE_VALUE 0x1A
#define LED0_ON_L 0x06
#define LED0_ON_H 0x07
#define LED0_OFF_L 0x08
#define LED0_OFF_H 0x09
#define LED_FULL_OFF_L 0x00
#define LED_FULL_OFF_H 0x10

#define POPULATED_CHANNEL_NUMBER 32
#define MIN_PWM_LENGTH 500
#define MAX_PWM_LENGTH 2500
#define DEFAULT_CHANNEL_STAGGERING false
#define CHANNEL_OFFSET_STEP 50 //in us. This value should not extend the last channel off timing to exceed the 4095 limit.
#define NOMINATED_ROOM_TEMPERATURE 25
#define DEFAULT_TEMPERATURE_CORRECTION 40 //per 30 degrees change in temperature
#define DEFAULT_SERVO_PWM_LENGTH 1500 
#define NOMINAL_CLOCK_FREQUENCY 25000000
#define DEFAULT_UPDATE_FREQUENCY 50
#define TEMPERATURE_CORRECTION_COEFFICIENT 0.00020 
#define TEMPERATURE_CORRECTION_STEP 128
#define TEMPERATURE_CORRECTION_POINTS ((MAX_PWM_LENGTH-MIN_PWM_LENGTH)/TEMPERATURE_CORRECTION_STEP)

/*Notes:
Servo channel number starts from 1, available channels are 1~POPULATED_CHANNEL_NUMBER.
PWM output on/off setting is from 0 to 4095.
Array storing data is using index from 0 ~ POPULATED_CHANNEL_NUMBER-1.
*/

class FuzzyServoBoard
{
	public:
		FuzzyServoBoard();
		void begin();  //required function call to initialize the board
		void begin(uint32_t clock1, uint32_t clock2, uint32_t targetUpdateFrequency); //Use clock and update frequency as input parameter. Clock frequency requires measurement, from calibration sketch.
		void setPWM(uint8_t servoChannel, uint16_t targetLength); //Command a servo position.
		void setOutputAll(bool value);//Set all output signals to on or off. Any PWM input command will turn the signal on. If signal are set on after an off command, output will set to last position.
		void setOutput(uint8_t servoChannel, bool value); //Set output signal to on or off. Any PWM input command will turn the signal on. 
		void setPWMAll(uint16_t length); //Set all channels to the command target PWM position.
		void setPrescale(uint8_t value1, uint8_t value2); //Write to both chips
		void setPrescale(uint8_t value); //Write to both chips using the same value
		uint8_t getPrescale(uint8_t chipNumber); //Manually set the prescale.
		void setEnvironmentTemperatureCelsius(int8_t degreesInCelsius); //set current temperature for temperature correction
		void setChannelStaggering(bool value); //if set true, output channels will stagger (phase shift) by the amout of time (us) defined in CHANNEL_OFFSET_STEP
		
		
		float getNominalUpdateFrequency(uint32_t clockFrequency, uint8_t prescale);
		void setClockFrequency(uint32_t clock1, uint32_t clock2);
		void setUpdateFrequency(float targetUpdateFrequency1, float targetUpdateFrequency2);
		void setUpdateFrequency(float targetUpdateFrequency);
		uint8_t getCalculatedPrescale(uint32_t calculatedClockFrequency, float targetUpdateFrequency);
		float getCalculatedUpdateFrequency(uint8_t chipNumber, uint8_t prescale);
		float getCalculatedUpdateFrequency(uint8_t chipNumber);
		uint16_t getCalbratedSteps(uint8_t chipNumber,uint16_t targetLength);
		float getCalculatedResolution(uint8_t chipNumber);

	private:
		uint8_t _i2cAddress1 = 0x40;
		uint8_t _i2cAddress2 = 0x41;
		float _calculatedUpdateFrequency1, _calculatedUpdateFrequency2;
		uint32_t _clock1, _clock2;
		float _targetUpdateFrequency1, _targetUpdateFrequency2;
		float _calculatedResolution1, _calculatedResolution2;
		
		void reset();//reset both chips
		uint8_t readRegister(uint8_t chipNumber, uint8_t registerAddress);
		void writeRegister(uint8_t chipNumber, uint8_t registerAddress, uint8_t value);
		void writeRegister(uint8_t registerAddress, uint8_t value); //write to both chips one by one
		void writeRegisterAll(uint8_t registerAddress, uint8_t value); //write to both chips using all call address
		void _setPWM(uint8_t servoChannel, uint16_t on, uint16_t off);
		void _setPWMAll(uint16_t on, uint16_t off);
		uint16_t channelOffset[POPULATED_CHANNEL_NUMBER]; //array starts from index 0, servo channel starts from index 1

		/* 
		The RC oscillator is affected by temperature, normally the clock frequency increases
		when the temperature increases. The measured coefficient is about 0.02% increase
		per degree C increase. An int array is used to reduce calculation time during run time.
		This temperature correction method is not very accurate, as the temperature control method is inaccurate during measurement.
		However, the coefficient measured on the PCA9685 chip is somewhat similar to ATMEGA internal RC oscillator datasheet value.
		*/
		int8_t deltaTemperature = 0;
		int8_t temperatureCorrection = DEFAULT_TEMPERATURE_CORRECTION;
		int8_t temperatureCorrectionArray[TEMPERATURE_CORRECTION_POINTS];

		uint16_t lastLength[POPULATED_CHANNEL_NUMBER];
};

#endif

