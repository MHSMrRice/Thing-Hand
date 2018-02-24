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
/*
 * This version of the control software was edited and rearranged
 * by Corey Rice on July 24, 2017. Main features include simple sweep
 * every 10 seconds and a battery monitoring feature. 
 * (Battery monitor routine will place the device in a sleep mode for 
 * 10 minute spans if the battery dips to below a safe voltage.)
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define servonum 3
#define SERVOMIN 150    // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 600    // this is the 'maximum' pulse length count (out of 4096)

#define VBATPIN A9              //for battery monitoring
#define lowBatteryVoltage 3.3   //for battery monitoring

void setup() {
  Serial.begin(9600);
  Serial.println("16 channel Servo test!");
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  yield(); 
}

void loop() {
  //==================================================================================
  //routine for measuring battery voltage (for LiPo safety)
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage

  //==================================================================================
  //if the battery voltage dips below a safe level, enter sleep mode for 10 mins 
  if(measuredvbat <= (lowBatteryVoltage*1024)) {
    voltagePrint(measuredvbat);
    Serial.println(" <--below safe level: entering 10 min standby...");
    delay(600000);
    return; 
  }
  
  //==================================================================================
  //serial print battery status, in case you are watching...
  voltagePrint(measuredvbat);
  Serial.println(""); // just a newline for serial print
  
  //==================================================================================
  //steps to make the servo move in a sweep, every 10 seconds
  Serial.println("Moving now");
  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
    pwm.setPWM(servonum, 0, pulselen);
    //delay(15); //uncomment for the halloween version
  }
  delay(250);   // a slight pause at the direction change
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
    pwm.setPWM(servonum, 0, pulselen);
    //delay(15); //uncomment for the halloween version
  }
  Serial.println("10 second delay");
  delay(10000); // ten seconds between motion
}

void voltagePrint(float measuredvbat){  
  measuredvbat /= 1024; // convert to voltage
  Serial.print("Battery Voltage: ");
  Serial.print(measuredvbat);
  Serial.print("V");
}

