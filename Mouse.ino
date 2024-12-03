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

//initialize servos
Servo myservo;
Servo myservo2;

//timer variable, don't edit manually as it will be overwritten.
long timeLeft = 0.0;

//mice speed, do NOT go over 15, the servo will take too much power, causing a arduino shut down
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


  //function that sets both servo's speed and direction and keep track of the time
  roam();

  //get IMU value
  readImu();
  
  // Update BLE connection status and handle incoming data
  updateBLE();
  
  //read the inputs te determine the current state
  //results in changing the value of currentMovement
  handleInput();
  sendMovement(currentMovement);

  //map the yaw value of the arduino's IMU to a value between 0 - 700, which is the reacheable range of the paddle, 
  // the value is mapped to 70 and multiplied by 10 is to make sure the value goes up or down by 10. 
  updateState(floor(map(yaw, 0, 360, 0, 70)) * 10);

  //Serial.println("s1ss");
}

void roam()
{
  //tick the time
  timeLeft -= deltaTime;
  Serial.println(timeLeft);

  //if time is smaller then 0, means time to do something
  if(timeLeft <= 0.0)
  {
    //new round of direction
    //get a new time, for a new direcion, this is how long this around of servo speed and direction will last.
    timeLeft += 1000 + random(-1500, 15500);

    //a state machine here to decide from 7 states.
    //0: forward
    //1: turn left
    //2: turn right
    //3: turn left backwards
    //4: turn right backwards
    //5: turn right backwards with an offset
    //6: stop fully
    //usually a random float will give a value between 0 - 5.99, only a very rare chance to have a 6, which results a full stop
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
