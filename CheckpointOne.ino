#include <Adafruit_MCP23X17.h>
#include <PS4Controller.h>

#define GPB0   8     //This is what nSLPL is connected to       
#define GPB1   9     //This is what nSLPR is connected to

//Associate the GPB0 and GPB1 pins with the nSLP signal for the motors
#define nSLPLPin GPB0
#define nSLPRPin GPB1
#define rightMotorDIRPin 15
#define rightMotorPWMPin 13
#define leftMotorDIRPin 14
#define leftMotorPWMPin 12

#define motorFreq 500       // set motor PWM frequency to 100 Hz
#define motorPwmBits 8      // use 8 bit counter for motor PWM control
#define FWD LOW               // Set DIR bit to 0 to move the robot in direction opposite claw (forward)
#define REV HIGH               // Set DIR bit to 1 to move the robot in direction of claw (reverse)
boolean swap = false;
const int motorPwmMaxCount = pow(2, motorPwmBits);

int pwmGoLowCount;                    //Count at which the motor PWM pulse goes low
int pwmDutyCycle = 0;                 //Duty cycle as percentage from 0-100

Adafruit_MCP23X17 gpioExt;

void setup() {
  Serial.begin(115200);
  Serial.println("Enter the motor duty cycle as a percentage from 0-100:");

  gpioExt.begin_I2C(0x20);                 //0x20 is the I2C address for the GPIO Extender

  gpioExt.pinMode(nSLPLPin, OUTPUT);    //all the usual pinModes exist: INPUT, OUTPUT, INPUT_PULLUP...
  gpioExt.pinMode(nSLPRPin, OUTPUT);

  gpioExt.digitalWrite(nSLPLPin, LOW);  //starts the robot with motors disabled (in sleep mode)
  gpioExt.digitalWrite(nSLPRPin, LOW);  //starts the robot with motors disabled (in sleep mode)

  pinMode(rightMotorDIRPin, OUTPUT);
  pinMode(leftMotorDIRPin, OUTPUT);
  pinMode(rightMotorPWMPin, OUTPUT);
  pinMode(leftMotorPWMPin, OUTPUT);

  ledcAttach(rightMotorPWMPin, motorFreq, motorPwmBits);
  ledcAttach(leftMotorPWMPin, motorFreq, motorPwmBits);

  PS4.begin();
}

void loop() {
  
  if(PS4.R1()){
    swap= (!swap);
    delay(500);
  }
  if(swap){
    driveBothWheels();
    if(PS4.LStickY()>3 || PS4.LStickY()<-3){
      driveLeftWheel();
      Serial.println("Left in both");
    }
    Serial.println("In both");
  }
  else{
  driveRightWheel();
  driveLeftWheel();
  Serial.println("In seperate");
  }
  



 
  

}

void driveRightWheel(){
     Serial.printf("Right Stick y at %d\n", PS4.RStickY());
    pwmDutyCycle = map(PS4.RStickY(), -127, 127, -100, 100);
    if(pwmDutyCycle<0){
      digitalWrite(rightMotorDIRPin, REV);
      pwmDutyCycle*=-1;
    }
    else{
      digitalWrite(rightMotorDIRPin, FWD);
    }
    gpioExt.digitalWrite(nSLPRPin, HIGH);
    pwmGoLowCount = 0.01 * pwmDutyCycle * motorPwmMaxCount; 
    ledcWrite(rightMotorPWMPin, pwmGoLowCount); 

  }

  void driveLeftWheel(){
     Serial.printf("Left Stick y at %d\n", PS4.LStickY());
    pwmDutyCycle = map(PS4.LStickY(), -127, 127, -100, 100);
    if(pwmDutyCycle<0){
      digitalWrite(leftMotorDIRPin, REV);
      pwmDutyCycle*=-1;
    }
    else{
      digitalWrite(leftMotorDIRPin, FWD);
    }
    gpioExt.digitalWrite(nSLPLPin, HIGH);
    pwmGoLowCount = 0.01 * pwmDutyCycle * motorPwmMaxCount; 
    ledcWrite(leftMotorPWMPin, pwmGoLowCount); 

  }

  void driveBothWheels(){
    pwmDutyCycle = map(PS4.RStickY(), -127, 127, -100, 100);
    if(pwmDutyCycle<0){
      digitalWrite(rightMotorDIRPin, REV);
      digitalWrite(leftMotorDIRPin, REV);
      pwmDutyCycle*=-1;
    }
    else{
      digitalWrite(rightMotorDIRPin, FWD);
      digitalWrite(leftMotorDIRPin, FWD);
    }
    gpioExt.digitalWrite(nSLPRPin, HIGH);
    gpioExt.digitalWrite(nSLPLPin, HIGH);
    pwmGoLowCount = 0.01 * pwmDutyCycle * motorPwmMaxCount; 
    ledcWrite(rightMotorPWMPin, pwmGoLowCount); 
    ledcWrite(leftMotorPWMPin, pwmGoLowCount);
  }





