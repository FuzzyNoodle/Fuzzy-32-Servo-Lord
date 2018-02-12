/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 16 servos, one after the other

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to  
  interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// you can also call it with a different address you want
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);//address for 1st PCA9685 servo 1 to 16 
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);//address for 2nd PCA9685 servo 17 to 32

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  100 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  750 // this is the 'maximum' pulse length count (out of 4096)

// our servo # counter
uint8_t servonum = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("16 channel Servo test!");
  
  pinMode(10, OUTPUT);//in this example pin 10 is to turn servo main power on or off, can use any pin
  digitalWrite(10, HIGH);//turn servo main power on, HIGH = power on, LOW = power off
  delay(500);
  
  pwm1.begin();
  pwm1.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  pwm2.begin();
  pwm2.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  yield();
}

// you can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. its not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 60;   // 60 Hz
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000;
  pulse /= pulselength;
  Serial.println(pulse);
  pwm1.setPWM(n, 0, pulse);
  pwm2.setPWM(n, 0, pulse);
}


void loop() {
  // Drive each servo one at a time
 
  for(servonum = 0; servonum <2; servonum++)//pwm1 starts with servo 1 on PCB
  {
    Serial.println(servonum+1);//display servo number currently moving
    for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) 
    {
    pwm1.setPWM(servonum, 0, pulselen);
    }
    delay(500);
    for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) 
    {
    pwm1.setPWM(servonum, 0, pulselen);
    }
    delay(500);
    }
  
  for(servonum = 0; servonum <2; servonum++)//pwm2 starts with servo 17 on PCB
  {
    Serial.println(servonum +16);//display servo numer currently moving
    for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) 
    {
    pwm2.setPWM(servonum, 0, pulselen);
    }
    delay(500);
    for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--)
    {
      pwm2.setPWM(servonum, 0, pulselen);
    }
    delay(500);
  }
}
