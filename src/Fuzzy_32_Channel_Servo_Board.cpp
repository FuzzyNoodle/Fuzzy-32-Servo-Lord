/*
 Name:		Fuzzy_32_Channel_Servo_Board.cpp
 Created:	6/12/2018 4:39:59 PM
 Author:	georgychen
 Editor:	http://www.visualmicro.com
*/

#include "Fuzzy_32_Channel_Servo_Board.h"
#include <Wire.h>

FuzzyServoBoard::FuzzyServoBoard()
{

}

void FuzzyServoBoard::init()
{
	init(NOMINAL_CLOCK_FREQUENCY, NOMINAL_CLOCK_FREQUENCY, DEFAULT_UPDATE_FREQUENCY);
}

void FuzzyServoBoard::init(uint32_t clock1, uint32_t clock2, uint32_t targetUpdateFrequency)
{
	reset();

	//Enable Auto-Increment and all call.
	writeRegister(PCA9685_CHIP_1, PCA9685_MODE1, PCA9685_AI | PCA9685_ALLCALL);
	writeRegister(PCA9685_CHIP_2, PCA9685_MODE1, PCA9685_AI | PCA9685_ALLCALL);

	//set default prescale value. may be different from chip to chip.
	//setPrescale(PCA9685_DEFAULT_PRESCALE_VALUE);

	setChannelStaggering(DEFAULT_CHANNEL_STAGGERING);
	for (uint8_t servoChannel = 1; servoChannel <= POPULATED_CHANNEL_NUMBER; servoChannel++)
	{
		lastLength[servoChannel - 1] = DEFAULT_SERVO_PWM_LENGTH;
	}

	setOutputAll(false);

	#ifdef SERIAL_DEBUG
	Serial.println("PCA9685 initialization completed.");
	#endif //SERIAL_DEBUG

}

void FuzzyServoBoard::setClockFrequency(float clock1, float clock2)
{
	_clock1 = clock1;
	_clock2 = clock2;

}

void FuzzyServoBoard::setUpdateFrequency(float updateFrequency1, float updateFrequency2)
{
	uint8_t prescale1 = getCalculatedPrescale(_clock1, updateFrequency1);
	uint8_t prescale2 = getCalculatedPrescale(_clock2, updateFrequency2);

}

void FuzzyServoBoard::setUpdateFrequency(float updateFrequency)
{
	setUpdateFrequency(updateFrequency, updateFrequency);
}

uint8_t FuzzyServoBoard::getCalculatedPrescale(float calculatedClockFrequency, float targetUpdateFrequency)
{
	//rounding method from here: https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library/issues/40
	uint32_t calculatedPrescale;
	calculatedPrescale = calculatedClockFrequency * 100 / (4096 * targetUpdateFrequency);
	if ((calculatedPrescale % 100) >= 50) calculatedPrescale += 100;
	calculatedPrescale = (calculatedPrescale / 100) - 1;
	return (uint8_t)calculatedPrescale;
}

void FuzzyServoBoard::setPWM(uint8_t servoChannel, uint16_t commandedLength)
{
	#ifdef SERIAL_DEBUG
	Serial.print("Commanded ");
	Serial.print(commandedLength);
	Serial.print(" ");
	#endif //SERIAL_DEBUG
	uint16_t adjustedLength = commandedLength;

	//perform frequency adjustment

	if (adjustedLength < MIN_PWM_LENGTH) adjustedLength = MIN_PWM_LENGTH;
	if (adjustedLength > MAX_PWM_LENGTH) adjustedLength = MAX_PWM_LENGTH;
	lastLength[servoChannel - 1] = adjustedLength;

	if (servoChannel >= 1 && servoChannel <= 16)
	{
		adjustedLength += getAdjustValue(1, adjustedLength);
	}
	else if (servoChannel <= POPULATED_CHANNEL_NUMBER)
	{
		adjustedLength += getAdjustValue(2, adjustedLength);
	}

	_setPWM(servoChannel, 0, adjustedLength);
}

void FuzzyServoBoard::_setPWM(uint8_t servoChannel, uint16_t on, uint16_t off)
{
	on += channelOffset[servoChannel - 1];
	off += channelOffset[servoChannel - 1];

	if (servoChannel >= 1 && servoChannel <= 16)
	{
		uint8_t channel = servoChannel - 1;
		Wire.beginTransmission(_i2cAddress1);
		Wire.write(LED0_ON_L + 4 * channel);
		Wire.write(on);
		Wire.write(on >> 8);
		Wire.write(off);
		Wire.write(off >> 8);
		Wire.endTransmission();
	}
	else if (servoChannel <= POPULATED_CHANNEL_NUMBER)
	{
		uint8_t channel = servoChannel - 17;
		Wire.beginTransmission(_i2cAddress2);
		Wire.write(LED0_ON_L + 4 * channel);
		Wire.write(on);
		Wire.write(on >> 8);
		Wire.write(off);
		Wire.write(off >> 8);
		Wire.endTransmission();
	}
	#ifdef SERIAL_DEBUG
	Serial.print("Channel ");
	Serial.print(servoChannel);
	Serial.print(" on ");
	Serial.print(on);
	Serial.print(" off ");
	Serial.println(off);
	#endif //SERIAL_DEBUG
}

void FuzzyServoBoard::reset()
{
	//perform a software reset to both chips with reserved address PCA9685_SWRST_ADDR
	Wire.beginTransmission(PCA9685_SWRST_ADDR);
	Wire.write(PCA9685_SWRST_ACK);
	Wire.endTransmission();
}

uint8_t FuzzyServoBoard::readRegister(uint8_t chipNumber, uint8_t registerAddress)
{
	if (chipNumber == 1)
	{
		Wire.beginTransmission(_i2cAddress1);
		Wire.write(registerAddress);
		Wire.endTransmission();
		Wire.requestFrom(_i2cAddress1, (uint8_t)1);
	}
	else if (chipNumber == 2)
	{
		Wire.beginTransmission(_i2cAddress2);
		Wire.write(registerAddress);
		Wire.endTransmission();
		Wire.requestFrom(_i2cAddress2, (uint8_t)1);
	}
	return Wire.read();
}

void FuzzyServoBoard::writeRegister(uint8_t chipNumber, uint8_t registerAddress, uint8_t value)
{
	if (chipNumber == 1)
	{
		Wire.beginTransmission(_i2cAddress1);
		Wire.write(registerAddress);
		Wire.write(value);
		Wire.endTransmission();
	}
	else if (chipNumber == 2)
	{
		Wire.beginTransmission(_i2cAddress2);
		Wire.write(registerAddress);
		Wire.write(value);
		Wire.endTransmission();
	}
}

void FuzzyServoBoard::writeRegister(uint8_t registerAddress, uint8_t value)
{
	writeRegister(PCA9685_CHIP_1, registerAddress, value);
	writeRegister(PCA9685_CHIP_2, registerAddress, value);
}

void FuzzyServoBoard::writeRegisterAll(uint8_t registerAddress, uint8_t value)
{
	Wire.beginTransmission(PCA9685_ALLCALLADR);
	Wire.write(registerAddress);
	Wire.write(value);
	Wire.endTransmission();
}

void FuzzyServoBoard::setOutput(uint8_t servoChannel, bool value)
{
	if (servoChannel > 0 && servoChannel <= POPULATED_CHANNEL_NUMBER)
	{
		if (value == true)
		{
			setPWM(servoChannel, lastLength[servoChannel - 1]);
		}
		else // (value == false)
		{
			#ifdef SERIAL_DEBUG
			Serial.print("Set servo channel ");
			Serial.print(servoChannel);
			Serial.println(" output off");
			#endif //SERIAL_DEBUG

			if (servoChannel >= 1 && servoChannel <= 16)
			{
				uint8_t channel = servoChannel - 1;
				Wire.beginTransmission(_i2cAddress1);
				Wire.write(LED0_OFF_L + 4 * channel);
				Wire.write(LED_FULL_OFF_L);
				Wire.write(LED_FULL_OFF_H);
				Wire.endTransmission();
			}
			else if (servoChannel <= POPULATED_CHANNEL_NUMBER)
			{
				uint8_t channel = servoChannel - 17;
				Wire.beginTransmission(_i2cAddress2);
				Wire.write(LED0_OFF_L + 4 * channel);
				Wire.write(LED_FULL_OFF_L);
				Wire.write(LED_FULL_OFF_H);
				Wire.endTransmission();
			}
		}
	}
}

void FuzzyServoBoard::setOutputAll(bool value)
{
	if (value == true)
	{
		for (uint8_t servoChannel = 1; servoChannel <= POPULATED_CHANNEL_NUMBER; servoChannel++)
		{
			setPWM(servoChannel, lastLength[servoChannel - 1]);
		}
		#ifdef SERIAL_DEBUG
		Serial.println("Set all channels on");
		#endif //SERIAL_DEBUG
	}
	else // (value == false)
	{
		Wire.beginTransmission(PCA9685_ALLCALLADR);
		Wire.write(PCA9685_ALL_LED_OFF_L);
		Wire.write(LED_FULL_OFF_L);
		Wire.write(LED_FULL_OFF_H);
		Wire.endTransmission();
		#ifdef SERIAL_DEBUG
		Serial.println("Set all channels off");
		#endif //SERIAL_DEBUG
	}
}

void FuzzyServoBoard::setPWMAll(uint16_t length)
{
	if (length < MIN_PWM_LENGTH) length = MIN_PWM_LENGTH;
	if (length > MAX_PWM_LENGTH) length = MAX_PWM_LENGTH;
	_setPWMAll(0, length);
}

void FuzzyServoBoard::_setPWMAll(uint16_t on, uint16_t off)
{
	/*sequenced writing method*/
	#ifdef SERIAL_DEBUG
	Serial.print("Set PWM all ");
	Serial.print(on);
	Serial.print(" ");
	Serial.println(off);
	#endif //SERIAL_DEBUG
	for (uint8_t servoChannel = 1; servoChannel < 17; servoChannel++)
	{
		uint16_t channelOn = on + channelOffset[servoChannel - 1];
		uint16_t channelOff = off + channelOffset[servoChannel - 1] + getAdjustValue(1, off - on);
		Wire.beginTransmission(_i2cAddress1);
		Wire.write(LED0_ON_L + 4 * (servoChannel - 1));
		Wire.write(channelOn);
		Wire.write(channelOn >> 8);
		Wire.write(channelOff);
		Wire.write(channelOff >> 8);
		Wire.endTransmission();
		lastLength[servoChannel - 1] = channelOff - channelOn;
		#ifdef SERIAL_DEBUG
		Serial.print("Set channel ");
		Serial.print(servoChannel);
		Serial.print(" ");
		Serial.print(channelOn);
		Serial.print(" ");
		Serial.println(channelOff);
		#endif //SERIAL_DEBUG
	}
	for (uint8_t servoChannel = 17; servoChannel < POPULATED_CHANNEL_NUMBER + 1; servoChannel++)
	{
		uint16_t channelOn = on + channelOffset[servoChannel - 1];
		uint16_t channelOff = off + channelOffset[servoChannel - 1] + getAdjustValue(2, off - on);
		Wire.beginTransmission(_i2cAddress2);
		Wire.write(LED0_ON_L + 4 * (servoChannel - 17));
		Wire.write(channelOn);
		Wire.write(channelOn >> 8);
		Wire.write(channelOff);
		Wire.write(channelOff >> 8);
		Wire.endTransmission();
		lastLength[servoChannel - 1] = channelOff - channelOn;
		#ifdef SERIAL_DEBUG
		Serial.print("Set channel ");
		Serial.print(servoChannel);
		Serial.print(" ");
		Serial.print(channelOn);
		Serial.print(" ");
		Serial.println(channelOff); 
		#endif //SERIAL_DEBUG
	}

	/* all call method. not used because of output staggering.
	Wire.beginTransmission(PCA9685_ALLCALLADR);
	Wire.write(PCA9685_ALL_LED_ON_L);
	Wire.write(on);
	Wire.write(on >> 8);
	Wire.write(off);
	Wire.write(off >> 8);
	Wire.endTransmission();
	*/

}

void FuzzyServoBoard::setChannelStaggering(bool value)
{
	for (uint8_t index = 0; index < POPULATED_CHANNEL_NUMBER; index++)
	{
		if (value == true)
		{
			channelOffset[index] = CHANNEL_OFFSET_STEP * (index % 16);
		}
		else
		{
			channelOffset[index] = 0;
		}
	}
}

int8_t FuzzyServoBoard::getAdjustValue(uint8_t chipNumber, uint16_t length)
{
	int8_t returnValue;
	int32_t frequencyAdjustValue;

	if (chipNumber == 1)
	{
		frequencyAdjustValue = (int32_t)((int32_t)(length)* pulseWidthCorrection1) / 10000;
	}
	else if (chipNumber == 2)
	{
		frequencyAdjustValue = (int32_t)((int32_t)(length)* pulseWidthCorrection2) / 10000;
	}


	int32_t temperatureAdjustValue = (int32_t)((int32_t)(deltaTemperature)* length* temperatureCorrection / 30) / 10000;

	returnValue = frequencyAdjustValue + temperatureAdjustValue;

	return returnValue;
}

void FuzzyServoBoard::setPrescale(uint8_t value1, uint8_t value2)
{
	//The PRE_SCALE register can only be set when the SPEEP bit of MODE1 register is set to logic 1.
	/*
	prescale value = round (  (osc_clock)/(4096*update_rate) ) -1
	For 1us resolution, prescale value = 0x18
	*/
	uint8_t currentMode;
	uint8_t sleepMode;

	currentMode = readRegister(PCA9685_CHIP_1, PCA9685_MODE1); //read current MODE1 register
	sleepMode = (currentMode & 0x7F) | PCA9685_SLEEP;
	writeRegister(PCA9685_CHIP_1, PCA9685_MODE1, sleepMode);
	writeRegister(PCA9685_CHIP_1, PCA9685_PRESCALE, value1); //change the prescaler register
	writeRegister(PCA9685_CHIP_1, PCA9685_MODE1, currentMode);

	currentMode = readRegister(PCA9685_CHIP_2, PCA9685_MODE1); //read current MODE1 register
	sleepMode = (currentMode & 0x7F) | PCA9685_SLEEP;
	writeRegister(PCA9685_CHIP_2, PCA9685_MODE1, sleepMode);
	writeRegister(PCA9685_CHIP_2, PCA9685_PRESCALE, value2); //change the prescaler register
	writeRegister(PCA9685_CHIP_2, PCA9685_MODE1, currentMode);

	/*
	It takes 500 us max. for the oscillator to be up and running once SLEEP bit
	has been set to logic 0. Timings on LEDn outputs are not guaranteed if PWM
	control registers are accessed within the 500 us window.
	*/
	delay(1);

	#ifdef SERIAL_DEBUG
	Serial.print("Set prescale ");
	Serial.print(value1);
	Serial.print(" ");
	Serial.println(value2);
	#endif //SERIAL_DEBUG
}

void FuzzyServoBoard::setPrescale(uint8_t value)
{
	setPrescale(value, value);
}

uint8_t FuzzyServoBoard::getPrescale(uint8_t chipNumber)
{
	uint8_t prescale;
	prescale = readRegister(chipNumber, PCA9685_PRESCALE);
	return prescale;
}

float FuzzyServoBoard::getNominalUpdateFrequency(uint32_t clockFrequency, uint8_t prescale)
{
	float nominalUpdateFrequency;
	nominalUpdateFrequency = (float)clockFrequency / (float)(((uint32_t)(prescale+1)) << 12);
	return nominalUpdateFrequency;
}

void FuzzyServoBoard::setPulseWidthCorrection(int8_t value1, int8_t value2)
{
	pulseWidthCorrection1 = value1;
	pulseWidthCorrection2 = value2;
}

void FuzzyServoBoard::setEnvironmentTemperature(uint8_t degreesInCelsius)
{
	deltaTemperature = degreesInCelsius - NOMINATED_ROOM_TEMPERATURE;
}

