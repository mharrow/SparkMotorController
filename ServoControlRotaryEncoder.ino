/* Speed Control of Spark Motor Controller
 * Tested on Teensy 3.2 and 4.0
 * Rotary Encoder - position value sets speed.
 * Integrated push button - start and stop motor.
 * Spark  - Full Speed Reverse 1000 usec pulse width
 *        - Full Speed Forward 2000 usec pulse width
 *        - Stop or Neutral 1500 usec pulse  + or - 40 usec (1460 to 1540)
 * 74HCT245 is used to convert 3.3V PWM to 5V PWM required by Spark Motor Controller
 * TODO:
 * Implement Reverse - Program only supports Forward movement of Motor
 */

#include <Encoder.h>
#include <Bounce.h>
#include <Servo.h>

#define PWM_OUT 7         //PWM Output pin
#define PWM_ENABLE 5      //Pin to Enable External PWM Output of 74HCT245
                          //Convert Teensy 3.3 voltage to 5 volt logic
                         
#define PWM_BUTTON 1      //Rotary Encoder integrated Momentary Push Button
#define LED 13            //Status LED - Motor Neutral on/off

#define MAX_DUTY 2000       //Max Duty Cycle of PWM
#define MIN_DUTY 1500       //Min Duty Cycle of PWM

int speedLevel;
boolean onOffState = LOW;   // Flag for Motor On/Off
boolean speedChangeValue = HIGH;   // Flag indicates speed change requested
int ledValue = LOW;         // Motor On/Off indication
 
Bounce bouncer = Bounce(PWM_BUTTON, 10);  //init input button Object 10 msec settle time
Encoder myEnc(3, 2);                      // init Rotary Encoder Object pins 2 and 3
Servo myServo;                            // init Servo Object

void setup()
{
 // Configure pins
  pinMode(PWM_OUT, OUTPUT);
  pinMode(PWM_ENABLE, OUTPUT);
  digitalWrite(PWM_ENABLE, LOW);
  pinMode(PWM_BUTTON,INPUT_PULLUP);
  pinMode(LED,OUTPUT);
  
  myServo.attach(PWM_OUT);                //Servo output pin for PWM

  Serial.begin(9600);                     //Open Serial Port for Status Messages
  digitalWrite(PWM_ENABLE,LOW);           //This pin conneted to Enable 74HCT244/74F245 Output

  myEnc.write(1500);                      //Init encoder to Neutral value
  myServo.writeMicroseconds(MIN_DUTY);    //set Motor to Neutral
  
}
// variables for myEnc rotary encoder
 long oldPosition = 0;
 long maxPosition = 2000;
 long minPosition = 1500;


void loop()
{ 
  // Check for On/Off Button Press and handle state change
  if (bouncer.update() && bouncer.fallingEdge()) {
    Serial.println("Button Pressed Teensy Controller");
      // Button press Toggle onOff State to Motor On
      if (onOffState == LOW) {
        onOffState = HIGH;
        digitalWrite(LED,HIGH);
        
      } else {
        // Button press toggle onOffState to Motor Off
        onOffState = LOW;
        digitalWrite(LED,LOW);
        myServo.writeMicroseconds(MIN_DUTY);  // Set Servo to Motor Off immediately
        myEnc.write(minPosition);
      }
    }
  
// Rotary Encoder Control of Speed - Test for Encoder change position 
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    if (newPosition > maxPosition)
    {
      newPosition = maxPosition;
      myEnc.write(newPosition);
    }
    if (newPosition < minPosition){
      newPosition = minPosition;
      myEnc.write(newPosition);
    }
    oldPosition = newPosition;
    speedChangeValue = HIGH;  // Flag to indicate Speed request change
  }
  
  //Check Flags update speedLevel and update Servo 
  if ((speedChangeValue && onOffState)) {
    speedLevel = newPosition;
    Serial.print (" Speed Value = ");
    Serial.println (speedLevel);
    Serial.println();
    myServo.writeMicroseconds(speedLevel);
    speedChangeValue = LOW;     // Speed Change has been completed so reset flag
  }
  
 }
