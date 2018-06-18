#include "Fuzzy_32_Channel_Servo_Board.h"
#include <Wire.h>


#define POWER_MOSFET_PIN  5

#define CHIP_1_FEEDBACK_CHANNEL 16
#define CHIP_2_FEEDBACK_CHANNEL 32
#define CHIP_1_INTERRUPT_INPUT_PIN 2
#define CHIP_2_INTERRUPT_INPUT_PIN 3


//For oscilloscope usage
#define CHIP_1_OSCILLOSCOPE_CHANNEL 2
#define CHIP_2_OSCILLOSCOPE_CHANNEL 18
#define NUMBER_OF_SAMPLES 500
FuzzyServoBoard ServoController;

volatile uint32_t chipPulseCounter = 0;
volatile uint32_t startMillis;
volatile uint32_t endMillis;

volatile uint32_t startMacros;
volatile uint32_t endMacros;

void setup()
{
	Serial.begin(115200);
	Serial.println("Fuzzy 32 Channel Servo Board");
	Serial.println("*** Chip Frequency Calibrator *** example started.");



	Wire.begin();
	Wire.setClock(400000); //set i2c frequency to 400k hz

	ServoController.init(); //required

	pinMode(POWER_MOSFET_PIN, OUTPUT);
	digitalWrite(POWER_MOSFET_PIN, HIGH);

	//Trigger oscilloscope
	pinMode(4, OUTPUT);
	digitalWrite(4, HIGH);

	//For oscilloscope usage
	pinMode(9, OUTPUT);
	analogWrite(9, 127);

	//ServoController.setPWM(2, 1500);
	//ServoController.setPWMAll(1500);

	ServoController.setPrescale(25);
	getUpdateFrequency(PCA9685_CHIP_1);

	getUpdateFrequency(PCA9685_CHIP_2);
	//Trigger oscilloscope
	delay(50);
	digitalWrite(4, LOW);
}

void loop()
{



}

float getUpdateFrequency(uint8_t chipID)
{
	ServoController.setOutputAll(false);
	chipPulseCounter = 0;

	uint8_t pin;
	if (chipID == PCA9685_CHIP_1)pin = CHIP_1_INTERRUPT_INPUT_PIN;
	if (chipID == PCA9685_CHIP_2)pin = CHIP_2_INTERRUPT_INPUT_PIN;

	attachInterrupt(digitalPinToInterrupt(pin), chipPulseCount, RISING);
	ServoController.setPWMAll(1500);
	while (chipPulseCounter <= NUMBER_OF_SAMPLES);
	ServoController.setOutputAll(false);
	detachInterrupt(digitalPinToInterrupt(pin));


	uint32_t duration = endMillis - startMillis;
	float period = (float)duration / (float)NUMBER_OF_SAMPLES;
	float measuredUpdateFrequency = (float)1000 / period;
	uint8_t prescale = ServoController.getPrescale(chipID);
	float nominalUpdateFrequency = ServoController.getNominalUpdateFrequency(NOMINAL_CLOCK_FREQUENCY, prescale);
	uint32_t calculatedClockFrequency = (uint32_t)(((float)NOMINAL_CLOCK_FREQUENCY) * measuredUpdateFrequency / nominalUpdateFrequency);
	Serial.print("Chip = ");
	Serial.println(chipID);
	Serial.print("Prescale = ");
	Serial.println(prescale);
	Serial.print("Count = ");
	Serial.println(chipPulseCounter);
	Serial.print("Duration = ");
	Serial.println(duration);
	Serial.print("Period = ");
	Serial.println(period);
	Serial.print("Measured updateFrequency = ");
	Serial.println(measuredUpdateFrequency);
	Serial.print("Nominal updateFrequency = ");
	Serial.println(nominalUpdateFrequency);
	Serial.print("Nominal clock frequency = ");
	Serial.println(NOMINAL_CLOCK_FREQUENCY);
	Serial.print("Calculated clock frequency = ");
	Serial.println(calculatedClockFrequency);
	Serial.println();
}

void chipPulseCount()
{
	if (chipPulseCounter == 0)startMillis = millis();
	if (chipPulseCounter == NUMBER_OF_SAMPLES) endMillis = millis();
	chipPulseCounter++;
}