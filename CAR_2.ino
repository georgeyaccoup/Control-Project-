#include <Servo.h>
#include <NewPing.h>

// Pin Definitions
#define BUTTON_PIN 2
#define FRONT_SENSOR_TRIGGER_PIN 3
#define FRONT_SENSOR_ECHO_PIN 4
#define LEFT_SENSOR_TRIGGER_PIN 5
#define LEFT_SENSOR_ECHO_PIN 6
#define RIGHT_SENSOR_TRIGGER_PIN 7
#define RIGHT_SENSOR_ECHO_PIN 8
#define MOTOR1_PIN 9
#define MOTOR2_PIN 10
#define MOTOR3_PIN 11
#define MOTOR4_PIN 12
#define SERVO_PIN 13
#define BUZZER_PIN A0

// Constants
#define SENSOR_THRESHOLD 20 // Distance in centimeters
#define TURN_DURATION 1000
#define BEEP_DURATION 200


// Define velocity levels
const int LOW_VELOCITY = 60;  // 60 RPM
const int MEDIUM_VELOCITY = 80;  // 80 RPM
const int HIGH_VELOCITY = 120; // 120 RPM

// Variables
Servo servo;
bool isStarted = false;
NewPing frontSensor(FRONT_SENSOR_TRIGGER_PIN, FRONT_SENSOR_ECHO_PIN);
NewPing leftSensor(LEFT_SENSOR_TRIGGER_PIN, LEFT_SENSOR_ECHO_PIN);
NewPing rightSensor(RIGHT_SENSOR_TRIGGER_PIN, RIGHT_SENSOR_ECHO_PIN);

// Function declarations
void rotateServo(int angle);
void moveForward();
void turnLeft();
void turnRight();
void stopCar();
void beepContinuously();
void setMotorVelocity(int velocity);

void setup() {
  // Set the pin modes
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(MOTOR1_PIN, OUTPUT);
  pinMode(MOTOR2_PIN, OUTPUT);
  pinMode(MOTOR3_PIN, OUTPUT);
  pinMode(MOTOR4_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Attach the servo to the appropriate pin
  servo.attach(SERVO_PIN);
}

void loop() {
  int frontDistance = frontSensor.ping_cm();
  delay(90);  
  rotateServo(60);
  int RightDistance = frontSensor.ping_cm();
  delay(90);  
  rotateServo(-120);
  int LifttDistance = frontSensor.ping_cm();
  delay(100);
  if (frontDistance>RightDistance&&frontDistance>LifttDistance){
    if (RightDistance>LifttDistance){
      turnLeft();
      delay(100); 
      if (LifttDistance > 60 ){
        if (LifttDistance > MEDIUM_VELOCITY){
          SetMotorSpeed(HIGH_VELOCITY);          
        }else if (LifttDistance<30){
          SetMotorSpeed(MEDIUM_VELOCITY);
        }
      }else {        
        SetMotorSpeed(LOW_VELOCITY); 
     }
      moveForward();
      delay(2000);     
    }else {
      turnRight();
      delay(100);
      if (LifttDistance > LOW_VELOCITY ){
        if (LifttDistance > MEDIUM_VELOCITY){
          SetMotorSpeed(HIGH_VELOCITY);          
        }else{
          SetMotorSpeed(MEDIUM_VELOCITY);
        }
      }else {        
        SetMotorSpeed(HIGH_VELOCITY); 
     }
      moveForward();
      delay(2000);    
    }
  }else {
      moveForward();
      delay(1000);
           
  } 
  if (LifttDistance<RightDistance&&LifttDistance<frontDistance){
    turnLeft();
    delay(1000);
    moveForward();
    delay(1500);
  }
  rotateServo(60);
  delay(2000);
  STOP();
  delay(1200);
  }

  


  /****before Editing****


  rotateServo(15);
  digitalWrite(MOTOR1_PIN, HIGH);
  digitalWrite(MOTOR2_PIN, LOW);
  delay(5000);  
  // Check if the car is started
  if (true) {
    // Read sensor values
    int frontDistance = frontSensor.ping_cm();
  
    // Check the sensor conditions
    if (frontDistance < SENSOR_THRESHOLD) {
        frontDistance = frontSensor.ping_cm();
        if (frontDistance < SENSOR_THRESHOLD) {
          rotateServo(-105);
          frontDistance = frontSensor.ping_cm();
          if (frontDistance > SENSOR_THRESHOLD) {
          turnLeft();
          delay(50);
          if (frontDistance<20){
           setMotorVelocity(LOW_VELOCITY);
           delay(50); // 5 seconds delay
          }else if (frontDistance>20 && frontDistance<50){
          setMotorVelocity(MEDIUM_VELOCITY);
          delay(50); // 5 seconds delay
          }else{
            setMotorVelocity(HIGH_VELOCITY);
           delay(500); // 5 seconds delay
          } 
          rotateServo(15);
        }
        }else{
          rotateServo(15);
          turnRight();
          delay(50);  
          if (frontDistance<20){
           setMotorVelocity(LOW_VELOCITY);
           delay(50); // 5 seconds delay
          }else if (frontDistance>20 && frontDistance<50){
          setMotorVelocity(MEDIUM_VELOCITY);
          delay(50); // 5 seconds delay
          }else{
            setMotorVelocity(HIGH_VELOCITY);
           delay(50); // 5 seconds delay
          }         
        }
      } else {
        stopCar();
        beepContinuously();
        if (frontDistance<20){
           setMotorVelocity(LOW_VELOCITY);
           delay(50); // 5 seconds delay
          }else if (frontDistance>20 && frontDistance<50){
          setMotorVelocity(MEDIUM_VELOCITY);
          delay(50); // 5 seconds delay
          }else{
            setMotorVelocity(HIGH_VELOCITY);
           delay(50); // 5 seconds delay
          } 
      }
    } else {
      moveForward();
    }
  } */
void SetMotorSpeed(int velocity){
  if (velocity == LOW_VELOCITY){
     analogWrite(MOTOR1_PIN, LOW_VELOCITY);
     analogWrite(MOTOR3_PIN, LOW_VELOCITY);
  }else if (velocity === MEDIUM_VELOCITY){
     analogWrite(MOTOR1_PIN, MEDIUM_VELOCITY);
     analogWrite(MOTOR3_PIN, MEDIUM_VELOCITY);
  }else{
     analogWrite(MOTOR1_PIN, HIGH_VELOCITY);
     analogWrite(MOTOR3_PIN, HIGH_VELOCITY);
  }

}
void moveForward() {
  digitalWrite(MOTOR1_PIN, HIGH);
  digitalWrite(MOTOR2_PIN, LOW);
  digitalWrite(MOTOR3_PIN, HIGH);
  digitalWrite(MOTOR4_PIN, LOW);
}
void STOP() {
  digitalWrite(MOTOR1_PIN, LOW);
  digitalWrite(MOTOR2_PIN, LOW);
  digitalWrite(MOTOR3_PIN, LOW);
  digitalWrite(MOTOR4_PIN, LOW);
}
void setMotorVelocity(int velocity) {
  // Map velocity to servo angle
  int angle = map(velocity, LOW_VELOCITY, HIGH_VELOCITY, 0, 180);

  // Set servo position
 // myservo.write(angle);

  // Set motor speed
  analogWrite(MOTOR1_PIN, velocity);
  analogWrite(MOTOR2_PIN, velocity);
  analogWrite(MOTOR3_PIN, velocity);
  analogWrite(MOTOR4_PIN, velocity);
}

void rotateServo(int angle) {
  int currentAngle = servo.read();
  int targetAngle = currentAngle + angle;
  
  if (targetAngle < 0) {
    targetAngle = 0;
  } else if (targetAngle > 180) {
    targetAngle = 180;
  }
  
  servo.write(targetAngle);
  delay(TURN_DURATION);
}

void turnLeft() {
  digitalWrite(MOTOR1_PIN, LOW);
  digitalWrite(MOTOR2_PIN, LOW);
  digitalWrite(MOTOR3_PIN, HIGH);
  digitalWrite(MOTOR4_PIN, LOW);
  //servo.write(90); // Turn the servo to the left
  //delay(TURN_DURATION);
}

void turnRight() {
  digitalWrite(MOTOR1_PIN, HIGH);
  digitalWrite(MOTOR2_PIN, LOW);
  digitalWrite(MOTOR3_PIN, LOW);
  digitalWrite(MOTOR4_PIN, LOW);
  servo.write(0); // Turn the servo to the right
  delay(TURN_DURATION);
}

void stopCar() {
  digitalWrite(MOTOR1_PIN, LOW);
  digitalWrite(MOTOR2_PIN, LOW);
  digitalWrite(MOTOR3_PIN, LOW);
  digitalWrite(MOTOR4_PIN, LOW);
  //servo.write(45); // Reset the servo to the center position
}

void beepContinuously() {
  while (true) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(BEEP_DURATION);
    digitalWrite(BUZZER_PIN, LOW);
    delay(BEEP_DURATION);
  }
}
