/*
 Name:		Fuzzy_32_Channel_Servo_Board.h
 Created:	6/12/2018 4:39:59 PM
 Author:	georgychen
 Editor:	http://www.visualmicro.com
*/

#ifndef _Fuzzy_32_Channel_Servo_Board_h
#define _Fuzzy_32_Channel_Servo_Board_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

/*Debug switch. Enable the SERIAL_DEBUG to output debug messages.*/
//#define SERIAL_DEBUG


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
#define CHANNEL_OFFSET_STEP 100 //in us. This value should not extend the last channel off timing to exceed the 4095 limit.
#define NOMINATED_ROOM_TEMPERATURE 25
#define DEFAULT_TEMPERATURE_CORRECTION 40 //per 30 degrees change in temperature
#define DEFAULT_SERVO_PWM_LENGTH 1500 
#define NOMINAL_CLOCK_FREQUENCY 25000000
#define DEFAULT_UPDATE_FREQUENCY 50

/*Notes:
Servo channel number starts from 1, available channels are 1~POPULATED_CHANNEL_NUMBER.
PWM output on/off setting is from 0 to 4095.
Array storing data is using index from 0 ~ POPULATED_CHANNEL_NUMBER-1.
*/

class FuzzyServoBoard
{
	public:
		FuzzyServoBoard();
		void init();  //required function call to initialize the board
		void init(uint32_t clock1, uint32_t clock2, uint32_t targetUpdateFrequency); //use clock frequency as input parameter. clock frequency requires measurement, from calibration sketch.
		void setPWM(uint8_t servoChannel, uint16_t length);
		void setOutputAll(bool value);
		void setOutput(uint8_t servoChannel, bool value); //set output to on or off
		void setPWMAll(uint16_t length);
		void setPrescale(uint8_t value1, uint8_t value2); //write to both chips
		void setPrescale(uint8_t value); //write to both chips using the same value
		uint8_t getPrescale(uint8_t chipNumber);
		void setPulseWidthCorrection(int8_t value1, int8_t value2); //fine tune of output length
		void setEnvironmentTemperature(uint8_t degreesInCelsius); //set current temperature for temperature correction
		void setChannelStaggering(bool value); //if set true, output channels will stagger (phase shift) by the amout of time (us) defined in CHANNEL_OFFSET_STEP
		float getNominalUpdateFrequency(uint32_t clockFrequency, uint8_t prescale);
		void setClockFrequency(float clock1, float clock2);
		void setUpdateFrequency(float updateFrequency1, float updateFrequency2);
		void setUpdateFrequency(float updateFrequency);
		uint8_t getCalculatedPrescale(float calculatedClockFrequency, float targetUpdateFrequency);
		

	private:
		uint8_t _i2cAddress1 = 0x40;
		uint8_t _i2cAddress2 = 0x41;
		uint32_t _chip1CalbratedClockFrequency = NOMINAL_CLOCK_FREQUENCY; //calibrated to 25 degree C
		uint32_t _chip2CalbratedClockFrequency = NOMINAL_CLOCK_FREQUENCY;//calibrated to 25 degree C
		uint16_t _calculatedUpdateFrequency;
		uint32_t _clock1, _clock2;
		uint32_t _targetUpdateFrequency;
		
		void reset();//reset both chips
		uint8_t readRegister(uint8_t chipNumber, uint8_t registerAddress);
		void writeRegister(uint8_t chipNumber, uint8_t registerAddress, uint8_t value);
		void writeRegister(uint8_t registerAddress, uint8_t value); //write to both chips one by one
		void writeRegisterAll(uint8_t registerAddress, uint8_t value); //write to both chips using all call address
		void _setPWM(uint8_t servoChannel, uint16_t on, uint16_t off);
		void _setPWMAll(uint16_t on, uint16_t off);
		

		uint16_t channelOffset[POPULATED_CHANNEL_NUMBER]; //array starts from index 0, servo channel starts from index 1

		/*pulseWidthCorrection1 and pulseWidthCorrection2 are used to make frequency adjustments on chip 1 and 2.
		It is expressed in percentage * 100. For example, pulseWidthCorrection1 = -50 means the
		measured period (i.e. 4116.48us) is longer than nominated (should be 4096us) by 0.50%
		and needs to be shortened by -0.50%.
		As result, pulse length needs to be shortened by pulse length/4096 * -0.50%.
		The correction value is expressed in int8_t, from -128~+127, represents -1.28% to +1.27% adjustment range.

		Correction value = (1- measured period/4096)*10000
		*/
		int8_t pulseWidthCorrection1 = 0;
		int8_t pulseWidthCorrection2 = 0;
		/* The measured temperate variation is about 6us reduction when chip
		temperature increase by 30 degrees C at 1500us. So this correction is applied if
		setEnvironmentTemperature(uint8_t degreesInCelsius) is called and different from
		NOMINATED_ROOM_TEMPERATURE
		This correction is a product-wise coarse correction, not expected to be modified chip by chip,
		due to the measurement of temperature itself is not so accurate.
		*/
		int8_t deltaTemperature = 0;
		int8_t temperatureCorrection = DEFAULT_TEMPERATURE_CORRECTION;
		int8_t getAdjustValue(uint8_t chipNumber, uint16_t length);
		uint16_t lastLength[POPULATED_CHANNEL_NUMBER];
};

#endif

