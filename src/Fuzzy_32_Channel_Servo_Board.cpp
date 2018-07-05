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

void FuzzyServoBoard::begin()
{
	begin(NOMINAL_CLOCK_FREQUENCY, NOMINAL_CLOCK_FREQUENCY, DEFAULT_UPDATE_FREQUENCY);
}

void FuzzyServoBoard::begin(uint32_t clock1, uint32_t clock2, uint32_t targetUpdateFrequency)
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

void FuzzyServoBoard::setClockFrequency(uint32_t clock1, uint32_t clock2)
{
	#ifdef SERIAL_DEBUG
	Serial.print("Set calibrated clock value  ");
	Serial.print(clock1);
	Serial.print(" ");
	Serial.print(clock2);
	Serial.println();
	#endif //SERIAL_DEBUG
	_clock1 = clock1;
	_clock2 = clock2;
}

void FuzzyServoBoard::setUpdateFrequency(float targetUpdateFrequency1, float targetUpdateFrequency2)
{
	#ifdef SERIAL_DEBUG
	Serial.print("Set target update frequency  ");
	Serial.print(targetUpdateFrequency1);
	Serial.print(" ");
	Serial.print(targetUpdateFrequency2);
	Serial.println();
	#endif //SERIAL_DEBUG

	_targetUpdateFrequency1 = targetUpdateFrequency1;
	_targetUpdateFrequency2 = targetUpdateFrequency2;
	uint8_t prescale1 = getCalculatedPrescale(_clock1, _targetUpdateFrequency1);
	uint8_t prescale2 = getCalculatedPrescale(_clock2, _targetUpdateFrequency2);
	setPrescale(prescale1, prescale2);

	_calculatedUpdateFrequency1 = getCalculatedUpdateFrequency(PCA9685_CHIP_1);
	_calculatedUpdateFrequency2 = getCalculatedUpdateFrequency(PCA9685_CHIP_2);

	_calculatedResolution1 = 1000000.0 / (_calculatedUpdateFrequency1 * 4096);
	_calculatedResolution2 = 1000000.0 / (_calculatedUpdateFrequency2 * 4096);

	#ifdef SERIAL_DEBUG
	Serial.print("Calculated update frequency = ");
	Serial.print(_calculatedUpdateFrequency1);
	Serial.print(" ");
	Serial.println(_calculatedUpdateFrequency2);

	Serial.print("Calculated resolution = ");
	Serial.print(_calculatedResolution1);
	Serial.print(" ");
	Serial.println(_calculatedResolution2);
	#endif //SERIAL_DEBUG
}

void FuzzyServoBoard::setUpdateFrequency(float updateFrequency)
{
	setUpdateFrequency(updateFrequency, updateFrequency);
}

uint8_t FuzzyServoBoard::getCalculatedPrescale(uint32_t calculatedClockFrequency, float targetUpdateFrequency)
{
	//rounding method from here: https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library/issues/40
	uint32_t calculatedPrescale;
	calculatedPrescale = (float)calculatedClockFrequency * 100.0 / (4096.0 * targetUpdateFrequency);
	if ((calculatedPrescale % 100) >= 50) calculatedPrescale += 100;
	calculatedPrescale = (calculatedPrescale / 100) - 1;
	return (uint8_t)calculatedPrescale;
}

float FuzzyServoBoard::getCalculatedUpdateFrequency(uint8_t chipNumber, uint8_t prescale)
{
	float calculatedUpdateFrequency;
	float calculatedClockFrequency;
	if (chipNumber == PCA9685_CHIP_1) calculatedClockFrequency = _clock1;
	if (chipNumber == PCA9685_CHIP_2) calculatedClockFrequency = _clock2;
	calculatedUpdateFrequency = calculatedClockFrequency / (float)(((uint32_t)prescale + 1) << 12);
	return calculatedUpdateFrequency;
}

float FuzzyServoBoard::getCalculatedUpdateFrequency(uint8_t chipNumber)
{
	float calculatedUpdateFrequency;
	uint8_t prescale;
	prescale = getPrescale(chipNumber);
	calculatedUpdateFrequency = getCalculatedUpdateFrequency(chipNumber, prescale);
	return calculatedUpdateFrequency;
}

uint16_t FuzzyServoBoard::getCalbratedSteps(uint8_t chipNumber,uint16_t targetLength)
{
	uint16_t steps;
	if (chipNumber == PCA9685_CHIP_1)
	{
		steps = (targetLength / _calculatedResolution1)+ 0.5 ;
	}
	else if (chipNumber == PCA9685_CHIP_2)
	{
		steps = (targetLength / _calculatedResolution2)+ 0.5;
	}
	return steps;
}

float FuzzyServoBoard::getCalculatedResolution(uint8_t chipNumber)
{
	if (chipNumber == PCA9685_CHIP_1)
	{
		return _calculatedResolution1;
	}
	else if (chipNumber == PCA9685_CHIP_2)
	{
		return _calculatedResolution2;
	}
}

void FuzzyServoBoard::setPWM(uint8_t servoChannel, uint16_t targetLength)
{
	#ifdef VERBOSE_SERIAL_DEBUG
	Serial.print("Target ");
	Serial.print(targetLength);
	Serial.print(" ");
	#endif //VERBOSE_SERIAL_DEBUG
	//uint16_t adjustedLength = commandedLength;

	//perform frequency adjustment
	uint16_t calbratedSteps;
	uint16_t calculatedLength;

	if (targetLength < MIN_PWM_LENGTH) targetLength = MIN_PWM_LENGTH;
	if (targetLength > MAX_PWM_LENGTH) targetLength = MAX_PWM_LENGTH;
	
	lastLength[servoChannel - 1] = targetLength;

	if (servoChannel >= 1 && servoChannel <= 16)
	{
		//adjustedLength += getAdjustValue(1, adjustedLength);
		calbratedSteps = getCalbratedSteps(PCA9685_CHIP_1, targetLength);
		calculatedLength = (float)calbratedSteps * _calculatedResolution1 + 0.5;
	}
	else if (servoChannel <= POPULATED_CHANNEL_NUMBER)
	{
		//adjustedLength += getAdjustValue(2, adjustedLength);
		calbratedSteps = getCalbratedSteps(PCA9685_CHIP_2, targetLength);
		calculatedLength = (float)calbratedSteps * _calculatedResolution2 + 0.5;
	}

	#ifdef VERBOSE_SERIAL_DEBUG
	Serial.print("Calculated = ");
	Serial.print(calculatedLength);
	Serial.print(" ");
	#endif //VERBOSE_SERIAL_DEBUG

	_setPWM(servoChannel, 0, calbratedSteps);
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
	#ifdef VERBOSE_SERIAL_DEBUG
	Serial.print("Channel ");
	Serial.print(servoChannel);
	Serial.print(" on ");
	Serial.print(on);
	Serial.print(" off ");
	Serial.println(off);
	#endif //VERBOSE_SERIAL_DEBUG
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

void FuzzyServoBoard::setPWMAll(uint16_t targetLength)
{
	if (targetLength < MIN_PWM_LENGTH) targetLength = MIN_PWM_LENGTH;
	if (targetLength > MAX_PWM_LENGTH) targetLength = MAX_PWM_LENGTH;
	_setPWMAll(0, targetLength);
}

void FuzzyServoBoard::_setPWMAll(uint16_t on, uint16_t off)
{
	/*sequenced writing method*/
	#ifdef SERIAL_VERBOSE_SERIAL_DEBUGDEBUG
	Serial.print("Set PWM all ");
	Serial.print(on);
	Serial.print(" ");
	Serial.println(off);
	#endif //VERBOSE_SERIAL_DEBUG
	for (uint8_t servoChannel = 1; servoChannel < 17; servoChannel++)
	{
		uint16_t channelOn = on + channelOffset[servoChannel - 1];
		uint16_t channelOff = channelOffset[servoChannel - 1] + getCalbratedSteps(PCA9685_CHIP_1, off);
		Wire.beginTransmission(_i2cAddress1);
		Wire.write(LED0_ON_L + 4 * (servoChannel - 1));
		Wire.write(channelOn);
		Wire.write(channelOn >> 8);
		Wire.write(channelOff);
		Wire.write(channelOff >> 8);
		Wire.endTransmission();
		lastLength[servoChannel - 1] = channelOff - channelOn;
		#ifdef VERBOSE_SERIAL_DEBUG
		Serial.print("Set channel ");
		Serial.print(servoChannel);
		Serial.print(" ");
		Serial.print(channelOn);
		Serial.print(" ");
		Serial.println(channelOff);
		#endif //VERBOSE_SERIAL_DEBUG
	}
	for (uint8_t servoChannel = 17; servoChannel < POPULATED_CHANNEL_NUMBER + 1; servoChannel++)
	{
		uint16_t channelOn = on + channelOffset[servoChannel - 1];
		uint16_t channelOff = channelOffset[servoChannel - 1] + getCalbratedSteps(PCA9685_CHIP_2, off);
		Wire.beginTransmission(_i2cAddress2);
		Wire.write(LED0_ON_L + 4 * (servoChannel - 17));
		Wire.write(channelOn);
		Wire.write(channelOn >> 8);
		Wire.write(channelOff);
		Wire.write(channelOff >> 8);
		Wire.endTransmission();
		lastLength[servoChannel - 1] = channelOff - channelOn;
		#ifdef VERBOSE_SERIAL_DEBUG
		Serial.print("Set channel ");
		Serial.print(servoChannel);
		Serial.print(" ");
		Serial.print(channelOn);
		Serial.print(" ");
		Serial.println(channelOff); 
		#endif //VERBOSE_SERIAL_DEBUG
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

void FuzzyServoBoard::setEnvironmentTemperature(uint8_t degreesInCelsius)
{
	deltaTemperature = degreesInCelsius - NOMINATED_ROOM_TEMPERATURE;
}

