/*
* _4.2_continuous_servo
*
* The board  will control the continuous rotation servo by writing a speed to it instead of a position.
*
* (c) 2013-2016 Arduino LLC.
*/
#include <Servo.h>
#include <ArduinoBLE.h>
#include "ble_functions.h"
#include "IMU.h"
#include "buzzer_functions.h"

long oldTime = 0.0f;
long currentTime = 0.0f;
long deltaTime = 0.0f;
#include "paddle_controller.h"

//Name your controller!
const char* deviceName = "CometMice";

// Pin definitions buzzer/LED
const int BUZZER_PIN = 11;       // Pin for haptic feedback buzzer
const int LED_PIN = LED_BUILTIN; // Status LED pin

// Movement state tracking
int currentMovement = 0;         // Current movement value (0=none, 1=up, 2=down, 3=handshake)


Servo myservo;
Servo myservo2;
long timeLeft = 0.0;
int speed = 15;
//mice state
//0 = onward
//1 = left turning
//2 = right turning
//3 = backward
//4 = stop
int miceState = 0;


void setup() {
  //servo defination
  myservo.attach(2);
  myservo2.attach(3);

  //initalized servo
  myservo.write(90);
  myservo2.write(90);
  Serial.begin(9600);
  
  //initializeIMU
  initializeImu();
  // Initialize Bluetooth Low Energy with device name and status LED
  setupBLE(deviceName, LED_PIN);


  
  // Initialize buzzer for feedback
  //setupBuzzer(BUZZER_PIN);

}

void loop() {
  //setup time calcilation for 
  oldTime = currentTime;
  currentTime = millis();
  deltaTime = currentTime - oldTime;
  //make the correct noise

  roam();


  readImu();
  // Update BLE connection status and handle incoming data
  updateBLE();
  
  //read the inputs te determine the current state
  //results in changing the value of currentMovement
  handleInput();
  sendMovement(currentMovement);
  updateState(floor(map(yaw, 0, 360, 0, 70)) * 10);

  //Serial.println("s1ss");
}

void roam()
{
  timeLeft -= deltaTime;
  Serial.println(timeLeft);
  if(timeLeft <= 0.0)
  {
    //new round of direction
    timeLeft += 1000 + random(-1500, 15500);
    int randomState = floor(random(0, 6));
    switch (randomState) 
    {
      case 0:
        miceState = 0;
        myservo.write(90 - speed);
        myservo2.write(90 + speed);
        break;
      case 1:
        miceState = 1;
        myservo.write(90 - speed*1.9);
        myservo2.write(90);
        break;
      case 2:
        miceState = 2;
        myservo.write(90);
        myservo2.write(90 + speed);
        break;
      case 3:
        miceState = 3;
        myservo.write(90 + speed*1.9);
        myservo2.write(90 - speed);
        break;
      case 4:
        miceState = 4;
        myservo.write(90);
        myservo2.write(90 - speed);
        break;
      case 5:
        miceState = 5;
        myservo.write(90 + speed*1.9);
        myservo2.write(90);
        break;
      default:
        miceState = 4;
        myservo.write(90);
        myservo2.write(90);
        break;
    }
  }
}
void handleInput() 
{
//flipped read method because of INPUT_PULLUP 
  //bool upPressed = !digitalRead(BUTTON_UP_PIN);
  //bool downPressed = !digitalRead(BUTTON_DOWN_PIN);
  
  if (FinalOutput == 1) 
  {
    currentMovement = 1;         // UP movement
  } 
  else if (FinalOutput  == 2) 
  {
    currentMovement = 2;         // DOWN movement
  } 
  else 
  {
    currentMovement = 0;         // No movement
  }
}
