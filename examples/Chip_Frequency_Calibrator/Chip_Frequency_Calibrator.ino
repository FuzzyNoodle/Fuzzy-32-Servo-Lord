#include "Fuzzy_32_Channel_Servo_Board.h"
#include <Wire.h>

/*
This sketch uses the arduino controller to measure PCA9685 output signal, and 
calculate the internal oscillator frequency of the PCA9685. Due to the inaccurate 
nature of the internal oscillator, the PCA9685 core frequency may be as much as 5%~10% 
off and differs chip by chip. So each chip needs to be calibrated if higher 
degree of accuracy is desired for robotic projects.

The calculated clock frequency is then used as the calibrator value for future usage. 

This sketch used the clock source from user's arduino control board 
(ex. UNO, NANO, MEGA2560, ZERO, etc...). Therefore the accuracy is limited by 
the on-board crystal oscillator which has a typical value of 0.2% ~ 0.5%.


The measured difference of core frequency using different prescale (thus different 
update frequency) is very small, around 0.03% maximum.  Thus one-time calibration 
for each chip should be good enough for different servos.

*/


/*
Wiring

Arduino PIN                         Servo Board Pin
5V                                  5V
GND                                 GND
SDA                                 SDA (A4 for UNO)
SCL                                 SCL (A5 for UNO)
PIN                                 POWER_MOSFET_PIN (5 in example)
CHIP_1_INTERRUPT_INPUT_PIN (2)      CHIP_1_FEEDBACK_CHANNEL (16)

*/

#define POWER_MOSFET_PIN  5
#define CHIP_1_FEEDBACK_CHANNEL 16
#define CHIP_2_FEEDBACK_CHANNEL 32
#define CHIP_1_INTERRUPT_INPUT_PIN 2
#define CHIP_2_INTERRUPT_INPUT_PIN 3


//For oscilloscope usage
#define CHIP_1_OSCILLOSCOPE_CHANNEL 2
#define CHIP_2_OSCILLOSCOPE_CHANNEL 18
#define NUMBER_OF_SAMPLES 500

//Create the servo controller object to control up to 32 servos.
FuzzyServoBoard ServoController;

volatile uint32_t chipPulseCounter = 0;
volatile uint32_t numberOfSamples = NUMBER_OF_SAMPLES;
volatile bool printProgress = false;
volatile uint32_t progressStep = 0;
volatile uint32_t startMicros;
volatile uint32_t endMicros;

void setup()
{
	Serial.begin(115200);
	Serial.println("Fuzzy 32 Channel Servo Board");
	Serial.println("[Chip Frequency Calibrator] example started.");


	Wire.begin();
	Wire.setClock(400000); //set i2c frequency to 400kHz

	//The begin() method is required to initialize the servo controller object.
	ServoController.begin();

	//Enable the power MOSFET on the servo board.
	pinMode(POWER_MOSFET_PIN, OUTPUT);
	digitalWrite(POWER_MOSFET_PIN, HIGH);


	/*
	Modify this value (targetUpdateFrequency) to desired update frequency.
	Achievable value range is 24 to 1526 Hz. according to the datasheet.
	A classic servo runs at 50 Hz, and the resolution is around 4.88us at this update frequency.
	Increase the update frequency reduce both the response time and resolution, however, if 
	the update frequency goes too high, the servo might shake or stop working. 
	
	Approximate values for some update frequencies:
	Update frequency        Period      Resolution
	 50    Hz.                20    ms.      4.88 us.
	 60    Hz.                16.67 ms.      4.07 us.
	100    Hz.                10    ms.      2.71 us.
	122.07 Hz.                8.19  ms.      2.00 us.
	150    Hz.                6.67  ms.      1.63 us.
	200    Hz.                5     ms.      1.22 us.
	244.14 Hz.                4.1   ms.      1.00 us.
	*/
	float targetUpdateFrequency = 50;
	
	
	/*
	Alternatively, user may set the target resolution for calibration.
	Uncomment the two lines below to set the desired frequency using desired resolution.
	*/
	//float targetResolution = 1.0;
	//targetUpdateFrequency = getUpdateFrequencyFromResolution(targetResolution);
	

	ServoController.setPrescale(PCA9685_DEFAULT_PRESCALE_VALUE);
	ServoController.setPWM(CHIP_1_FEEDBACK_CHANNEL, 1500);
	Serial.println("Running first pass using default prescale...");
	float clock1 = getCalculatedClockFrequency(PCA9685_CHIP_1, NUMBER_OF_SAMPLES);
	uint8_t prescale1 = getCalculatedPrescale(clock1, targetUpdateFrequency);
	Serial.println("Now running second pass...");
	Serial.print("Using target update frequency of ");
	Serial.print(targetUpdateFrequency);
	Serial.println(" Hz");
	Serial.print("Resetting prescale = ");
	Serial.println(prescale1);
	ServoController.setPrescale(prescale1);
	Serial.println("Verifying result...");
	clock1 = getCalculatedClockFrequency(PCA9685_CHIP_1, NUMBER_OF_SAMPLES);
	Serial.println("Now running third pass...");
	prescale1 = getCalculatedPrescale(clock1, targetUpdateFrequency);
	ServoController.setPrescale(prescale1);
	clock1 = getCalculatedClockFrequency(PCA9685_CHIP_1, NUMBER_OF_SAMPLES);


	ServoController.setPrescale(PCA9685_DEFAULT_PRESCALE_VALUE);
	ServoController.setPWM(CHIP_2_FEEDBACK_CHANNEL, 1500);
	Serial.println("Running first pass using default prescale...");
	float clock2 = getCalculatedClockFrequency(PCA9685_CHIP_2, NUMBER_OF_SAMPLES);
	uint8_t prescale2 = getCalculatedPrescale(clock2, targetUpdateFrequency);
	Serial.println("Now running second pass...");
	Serial.print("Using target update frequency of ");
	Serial.print(targetUpdateFrequency);
	Serial.println(" Hz");
	Serial.print("Resetting prescale = ");
	Serial.println(prescale2);
	ServoController.setPrescale(prescale2);
	Serial.println("Verifying result...");
	clock2 = getCalculatedClockFrequency(PCA9685_CHIP_2, NUMBER_OF_SAMPLES);
	Serial.println("Now running third pass...");
	prescale2 = getCalculatedPrescale(clock2, targetUpdateFrequency);
	ServoController.setPrescale(prescale2);
	clock2 = getCalculatedClockFrequency(PCA9685_CHIP_2, NUMBER_OF_SAMPLES);

	ServoController.setOutputAll(false);

	// Output final results for two PCA9685 chips
	Serial.println("Now printing final results...");
	Serial.println("*******************************************");
	Serial.println("Use the following values for calibration:");
	Serial.print("Chip 1 clock frequency = ");
	Serial.println((uint32_t)clock1);
	Serial.print("Chip 2 clock frequency = ");
	Serial.println((uint32_t)clock2);
	Serial.println("*******************************************");
	Serial.println("Sketch ended.");

}

void loop()
{



}


float getUpdateFrequencyFromResolution(float targetResolution)
{
	float updateFrequency;
	updateFrequency = 1000000.0 / (4096 * targetResolution);
	return updateFrequency;
}

float getResolutionMicroseconds(float calculatedClockFrequency, uint8_t prescale)
{
	float resolution;
	resolution = 1000000.0 / (4096.0 * getCalculatedUpdateFrequency(calculatedClockFrequency, prescale));
	return resolution;
}

float getCalculatedUpdateFrequency(float calculatedClockFrequency, uint8_t prescale)
{
	float calculatedUpdateFrequency;
	calculatedUpdateFrequency = calculatedClockFrequency / (float)(((uint32_t)prescale + 1) << 12);
	return calculatedUpdateFrequency;
}

uint8_t getCalculatedPrescale(float calculatedClockFrequency, float targetUpdateFrequency)
{
	//rounding method from here: https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library/issues/40
	uint32_t calculatedPrescale;
	calculatedPrescale = calculatedClockFrequency * 100 / (4096 * targetUpdateFrequency);
	if ((calculatedPrescale % 100) >= 50) calculatedPrescale += 100;
	calculatedPrescale = (calculatedPrescale / 100) - 1;
	return (uint8_t)calculatedPrescale;
}


float getCalculatedClockFrequency(uint8_t chipID,uint32_t samples)
{
	ServoController.setOutputAll(false);
	chipPulseCounter = 0;
	numberOfSamples = samples;

	uint8_t pin;
	if (chipID == PCA9685_CHIP_1)pin = CHIP_1_INTERRUPT_INPUT_PIN;
	if (chipID == PCA9685_CHIP_2)pin = CHIP_2_INTERRUPT_INPUT_PIN;



	uint8_t prescale = ServoController.getPrescale(chipID);
	progressStep = (numberOfSamples / 10);


	Serial.print("Getting calculated clock frequency for chip ");
	Serial.println(chipID);
	Serial.print("Prescale = ");
	Serial.println(prescale);
	Serial.print("Samples = ");
	Serial.println(numberOfSamples);

	Serial.print("Progress ");
	attachInterrupt(digitalPinToInterrupt(pin), chipPulseCount, RISING);
	ServoController.setPWMAll(1500);
	
	
	while (chipPulseCounter <= numberOfSamples)
	{
		if (printProgress == true)
		{
			Serial.print(".");
			printProgress = false;
		}
	}
	Serial.println(" Done");

	ServoController.setOutputAll(false);
	detachInterrupt(digitalPinToInterrupt(pin));


	uint32_t duration = endMicros - startMicros;
	float period = (float)duration / (float)numberOfSamples;
	float measuredUpdateFrequency = (float)1000000 / period;
	
	float nominalUpdateFrequency = ServoController.getNominalUpdateFrequency(NOMINAL_CLOCK_FREQUENCY, prescale);
	uint32_t calculatedClockFrequency = (uint32_t)(((float)NOMINAL_CLOCK_FREQUENCY) * measuredUpdateFrequency / nominalUpdateFrequency);
	
	float error = 100*(calculatedClockFrequency - (float)NOMINAL_CLOCK_FREQUENCY) / (float)NOMINAL_CLOCK_FREQUENCY;

	Serial.print("Period Count = ");
	Serial.println(chipPulseCounter-1);
	Serial.print("Duration(us) = ");
	Serial.println(duration);
	Serial.print("Period(us) = ");
	Serial.println(period);
	Serial.print("Measured update frequency = ");
	Serial.println(measuredUpdateFrequency);
	Serial.print("Calculated clock frequency = ");
	Serial.println(calculatedClockFrequency);
	Serial.print("Frequency error = ");
	Serial.print(error);
	Serial.println("%");
	Serial.print("Resolution = ");
	Serial.print(getResolutionMicroseconds(calculatedClockFrequency, prescale));
	Serial.println(" microseconds");

	Serial.println();

	return calculatedClockFrequency;
}

void chipPulseCount()
{
	if (chipPulseCounter == 0)startMicros = micros();
	if (chipPulseCounter == numberOfSamples) endMicros = micros();
	if ((chipPulseCounter%progressStep) == 0) printProgress = true;
	chipPulseCounter++;
}