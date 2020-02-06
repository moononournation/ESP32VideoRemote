/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 8 pins of the PCA9685

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815
  
  These drivers use I2C to communicate, 2 pins are required to  
  interface.

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVO_MIN 1
#define SERVO_MAX 16
#define USMIN  670 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 60 // Analog servos run at ~60 Hz updates

// our servo # counter
uint8_t servo_id = 1;

void setup() {
  Serial.begin(115200);
  Serial.printf("Serial Servo Tester for Adafruit_PWMServoDriver\n");
  Serial.printf("===============================================\n");
  Serial.printf("Input %d-%d for select servo;\n", SERVO_MIN, SERVO_MAX);
  Serial.printf("Input %d-%d for servo pulse width:\n", USMIN, USMAX);

  Wire.begin(12, 13);
  pwm.begin();
  // In theory the internal oscillator is 25MHz but it really isn't
  // that precise. You can 'calibrate' by tweaking this number till
  // you get the frequency you're expecting!
  pwm.setOscillatorFrequency(27000000);  // The int.osc. is closer to 27MHz  
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~60 Hz updates

  delay(10);
}

void loop() {
  if (Serial.available() > 0) {
    int i = Serial.parseInt();
    if ((i >= SERVO_MIN) && (i <= SERVO_MAX)) {
      Serial.printf("Set servo to %d.\n", i);
      servo_id = i;
    } else if ((i >= USMIN) && (i <= USMAX)) {
      Serial.printf("Set servo %d pulsewidth to %d.\n", servo_id, i);
      // Drive each servo one at a time using writeMicroseconds(), it's not precise due to calculation rounding!
      // The writeMicroseconds() function is used to mimic the Arduino Servo library writeMicroseconds() behavior.
      pwm.writeMicroseconds(servo_id - 1, i);
    }
  }
}
