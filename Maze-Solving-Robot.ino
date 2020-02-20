// Nov 27, 2019
// left turn fix

// Include AccelStepper and Servo library
#include <AccelStepper.h>
#include <VarSpeedServo.h>

//Define pin connections to the stepper motors and motor interface type (must be 1)
#define dirPinLeft 2
#define stepPinLeft 3
#define dirPinRight 4
#define stepPinRight 5
#define motorInterfaceType 1

// Define pin connections to the ultrasonic sensors
#define trigPinFront1 22
#define echoPinFront1 23
#define trigPinFront2 24
#define echoPinFront2 25
#define trigPinLeft1 26
#define echoPinLeft1 27
#define trigPinLeft2 28
#define echoPinLeft2 29
#define trigPinRight1 30
#define echoPinRight1 31
#define trigPinRight2 32
#define echoPinRight2 33
#define trigPinBack1 34
#define echoPinBack1 35
#define trigPinCube1 36
#define echoPinCube1 37
#define trigPinCube2 38
#define echoPinCube2 39
#define led 13

// Constants for reading ultrasonics
long durationFront1;
long durationFront2;
long durationLeft1;
long durationLeft2;
long durationRight1;
long durationRight2;
long durationBack1;
long durationCube1;
long durationCube2;

// Distance distance variables for ultrasonics
int distanceFront1;
int distanceFront2;
int distanceLeft1;
int distanceLeft2;
int distanceRight1;
int distanceRight2;
int distanceBack1;
int distanceCube1;
int distanceCube2;

// Tuning Variables for Wall Detection
const int limitDistanceFront = 6;
const int limitDistanceSide = 10;

// Tuning Variables for Stepper Motors
const int turnSteps = 125;
const int fineTurnSteps = 30;
const int normalStepForward = 20;

// Servo Positions
const int downPos = 120;
const int liftPos = 20;
const int grabPos = 120;
const int letGoPos = 55;
const int servoSpeed = 75;

// PID Variables
double error = 0;
double pastError = 0;
double currentValue = 0;
double integral = 0;
double derivative = 0;
double Kp = 10;
double Ki = 0;
double Kd = 10;
double PIDvalue = 0;
double motorSpeedLeft = 0;
double motorSpeedRight = 0;
double baseMotorSpeedLeft = 73;
double baseMotorSpeedRight = 70;

int four = 10;
int three = 10;
int two = 10;
int current = 10;

const int distanceWall = 20;
int wallCounter;
int path;
boolean localized = false;
const int decidePathLimit = 50;
boolean orientatedForHome = false;
int homeLocation = 1;
int home1Counter = 0;
int home3Counter = 0;
int home4Counter = 0;
boolean finalSequence = false;
int lastWallCounter = 10;
int runCounter = 0;
int threeCounter = 0;

// Block Pick Up
int pass = 0;
int done = 0;

// Create instance of the AccelStepper class
AccelStepper stepperLeft = AccelStepper(motorInterfaceType, stepPinLeft, dirPinLeft);
AccelStepper stepperRight = AccelStepper(motorInterfaceType, stepPinRight, dirPinRight);

//Create instance of the Servo class
VarSpeedServo gripper;
VarSpeedServo lifter;

void setup() {
  // Pin Setup
  pinMode(trigPinFront1, OUTPUT);
  pinMode(echoPinFront1, INPUT);
  pinMode(trigPinFront2, OUTPUT);
  pinMode(echoPinFront2, INPUT);
  pinMode(trigPinLeft1, OUTPUT);
  pinMode(echoPinLeft1, INPUT);
  pinMode(trigPinLeft2, OUTPUT);
  pinMode(echoPinLeft2, INPUT);
  pinMode(trigPinRight1, OUTPUT);
  pinMode(echoPinRight1, INPUT);
  pinMode(trigPinRight2, OUTPUT);
  pinMode(echoPinRight2, INPUT);
  pinMode(trigPinBack1, OUTPUT);
  pinMode(echoPinBack1, INPUT);
  pinMode(trigPinCube1, OUTPUT);
  pinMode(echoPinCube1, INPUT);
  pinMode(trigPinCube2, OUTPUT);
  pinMode(echoPinCube2, INPUT);
  pinMode(led, OUTPUT);

  // Attaches the servos on pins (digital PWM)
  gripper.write(grabPos);
  gripper.attach(11);
  delay(2000);
  lifter.write(liftPos);
  lifter.attach(10);
  delay(2000);

  //Set the max speed in terms of steps per second
  stepperLeft.setMaxSpeed(300);
  stepperRight.setMaxSpeed(300);

  //Set the max acceleration
  stepperLeft.setAcceleration(1);
  stepperRight.setAcceleration(1);

  Serial.begin(9600);

  // Align To Wall at Start using Left Sensors
  readAll();
  do {
    readAll();
    if (distanceLeft1 - distanceLeft2 > 0)
    {
      turnLeft(3);
      delay(500);
    }
    if (distanceLeft1 - distanceLeft2 < 0)
    {
      turnRight(3);
      delay(500);
    }
    readAll();
  } while (abs(distanceLeft1 - distanceLeft2) > 2);
}

void loop() {
  readAll();
  if (localized == false) {
    readWalls();
    if (wallCounter != current && (wallCounter == 0 || wallCounter == 1 || wallCounter == 2 || wallCounter == 3 || wallCounter == 5)) {
      four = three;
      three = two;
      two = current;
      current = wallCounter;
      Serial.print("Current Path:");
      Serial.print(current);
      Serial.print("  ");
      Serial.print(two);
      Serial.print("  ");
      Serial.print(three);
      Serial.print("  ");
      Serial.println(four);
      delay(500);
    }
    if (current == 0) {
      Serial.println("Checking Sequence");
      checkSequence();
    }
    readAll();
    movement();
  } else if (localized == true && finalSequence == false) {
    movement();
    readWalls();
    if (wallCounter == 2 && distanceFront1 >= decidePathLimit && distanceFront2 >= decidePathLimit) {
      do {
        readAll();
        movement();
      } while (distanceLeft1 < limitDistanceSide);
      blinkLZ();
      delay(5000);
      extractionHardCode();
      //dynamicExtraction();
    }
  } else if (finalSequence == true) {
    readAll();
    if (orientatedForHome == false) {
      Serial.println("180");
      turnLeft(turnSteps * 2);
      align();
      runCounter++;
      if (runCounter == 1) {
        while (distanceFront1 > limitDistanceFront && distanceFront2 > limitDistanceFront && distanceLeft1 > limitDistanceSide) {
          Serial.println("align");
          delay(1000);
          turnLeft(5);
          forward(5);
          align();
          readAll();
        }

        Serial.println("exit while loop");
      }
      orientatedForHome = true;
    }
    switch (homeLocation) {
      case 1:
        Serial.println("Home 1");
        Serial.println("Normal Movement");
        movement();
        readWalls();
        if (wallCounter == 3) {
          Serial.println("Block drop off");
          blinkFinal();
          dropOff();
        }
        if (wallCounter == 0 || wallCounter == 1 || wallCounter == 2 || wallCounter == 3 || wallCounter == 5) {
          Serial.print("last wall counter:");
          Serial.println(lastWallCounter);
          Serial.print("current counter");
          Serial.println(wallCounter);
          lastWallCounter = wallCounter;
        }
        break;
      case 2:
        Serial.println("Home 2");
        Serial.println("Normal Movement");
        movement();
        readWalls();
        if (wallCounter == 3) {
          Serial.println("Block drop off");
          blinkFinal();
          dropOff();
        }
        break;
      case 3:
        Serial.println("Home 3");
        Serial.println("Normal Movement");
        movement();
        readWalls();
        if (wallCounter == 0) {
          home3Counter++;
          if (home3Counter == 1) {
            readAll();
            movement();
            readAll();
            movement();
            Serial.println("Turn Left");
            turnLeft(turnSteps);
          }
        }
        if (wallCounter == 3 && lastWallCounter == 0) {
          Serial.println("Block drop off");
          blinkFinal();
          dropOff();
        }
        if (wallCounter != lastWallCounter && (wallCounter == 0 || wallCounter == 1 || wallCounter == 2 || wallCounter == 3 || wallCounter == 5)) {
          Serial.print("last wall counter:");
          Serial.println(lastWallCounter);
          Serial.print("current counter");
          Serial.println(wallCounter);
          lastWallCounter = wallCounter;
        }
        break;
      case 4:
        Serial.println("home 4");
        if (orientatedForHome == false) {
          Serial.println("180");
          turnLeft(turnSteps * 2);
          align();
          orientatedForHome = true;
        }
        Serial.println("Normal Movement");
        movement();
        readWalls();
        if (wallCounter == 0) {
          home4Counter++;
          if (home4Counter == 1) {
            readAll();
            movement();
            readAll();
            movement();
            Serial.println("Turn Right");
            turnRight(turnSteps);
          }
        }
        if (wallCounter == 1 && lastWallCounter == 2) {
          readAll();
          movement();
          readAll();
          movement();
          Serial.println("Turn Right");
          turnRight(turnSteps);
        }
        if (wallCounter == 3) {
          Serial.println("Block drop off");
          blinkFinal();
          dropOff();
        }
        if (wallCounter != lastWallCounter && (wallCounter == 0 || wallCounter == 1 || wallCounter == 2 || wallCounter == 3 || wallCounter == 5)) {
          Serial.print("last wall counter:");
          Serial.println(lastWallCounter);
          Serial.print("current counter");
          Serial.println(wallCounter);
          lastWallCounter = wallCounter;
        }
        break;
    }
  }
}

int align() {
  do {
    readAll();
    if (distanceLeft1 - distanceLeft2 > 0)
    {
      turnLeft(3);
      delay(100);
    }
    if (distanceLeft1 - distanceLeft2 < 0)
    {
      turnRight(3);
      delay(100);
    }
    readAll();
  } while (abs(distanceLeft1 - distanceLeft2) > 2);
}

int blinkLZ() {
  Serial.println("AT LZ");
  for (int i = 0; i < 5; i++) {
    digitalWrite(led, HIGH);
    delay(500);
    digitalWrite(led, LOW);
    delay(300);
  }
}

int blinkFinal() {
  Serial.println("AT HOME");
  digitalWrite(led, HIGH);
  delay(5000);
}

int blinkLocal() {
  Serial.println("LOCALIZED");
  for (int i = 0; i < 3; i++) {
    digitalWrite(led, HIGH);
    delay(500);
    digitalWrite(led, LOW);
    delay(300);
  }
}

int movement() {
  // Move Forward using PID Control (left or right alignment)
  if (distanceFront1 > limitDistanceFront && distanceFront2 > limitDistanceFront)
  {

    // Turn left if it senses opening

    if (distanceLeft1 > 25 && distanceLeft2 > 25 && distanceRight1 < 10 && distanceRight2 < 10)
    {

      Serial.println("entering pocket");
      delay (3000);
      readAll();
      readWalls();
      if (wallCounter != current && (wallCounter == 0 || wallCounter == 1 || wallCounter == 2 || wallCounter == 3 || wallCounter == 5)) {
        four = three;
        three = two;
        two = current;
        current = wallCounter;
        Serial.print("Current Path:");
        Serial.print(current);
        Serial.print("  ");
        Serial.print(two);
        Serial.print("  ");
        Serial.print(three);
        Serial.print("  ");
        Serial.println(four);
        delay(500);
      }
      // Adjust motor speed
      motorSpeedLeft = baseMotorSpeedLeft;
      motorSpeedRight = baseMotorSpeedRight;
      // Move FORWARD
      forward(normalStepForward * 3);

      // Adjust motor speed
      motorSpeedLeft = baseMotorSpeedLeft;
      motorSpeedRight = baseMotorSpeedRight;
      // Turn Left
      turnLeft(turnSteps);

      // Adjust motor speed
      motorSpeedLeft = baseMotorSpeedLeft;
      motorSpeedRight = baseMotorSpeedRight;
      // Move FORWARD
      forward(normalStepForward * 8);
    }


    // Choose left sensor for PID Control
    if (distanceLeft1 < limitDistanceSide) {
      pidLeft();
      forward(normalStepForward);
      //Serial.print("PIDVALUE: ");
      //Serial.println(PIDvalue);
    } // End PID left Control

    // Choose right sensor for PID Control
    else if (distanceRight1 < limitDistanceSide) {
      pidRight();
      forward(normalStepForward);
      //Serial.println("PID RIGHT");
    } // End PID right Control

    // Adjust motor speed to base values
    else {
      motorSpeedLeft = baseMotorSpeedLeft;
      motorSpeedRight = baseMotorSpeedRight;
      // Move FORWARD without PID
      forward(normalStepForward);
    }
  }
  // Turn right due to obstacle detected
  if ((distanceFront2 <= limitDistanceFront || distanceFront1 <= limitDistanceFront))
  {
    if (distanceRight1 <= limitDistanceSide || distanceRight2 <= limitDistanceSide)
    {
      turnLeft(turnSteps);
      Serial.println("Turn left because of wall");
      delay(1000);
    }
    else {
      turnRight(turnSteps);
    }
  } // End Obstacle Detection
}

int checkSequence() {
  if (two == 2) {
    if (three == 5) {
      readAll();
      if (distanceFront1 > decidePathLimit) {
        path = 2;
        Serial.println("Path 2");
        localized = true;
        blinkLocal();
        for (int i = 0; i < 3; i++) {
          readAll();
          movement();
        }
        turnRight(turnSteps * 2);
      } else {
        path = 3;
        Serial.println("Path 3");
        localized = true;
        blinkLocal();
        for (int i = 0; i < 3; i++) {
          readAll();
          movement();
        }
        turnLeft(turnSteps);
      }
    } else if (three == 1) {
      path = 4;
      Serial.println("Path 4");
      localized = true;
      blinkLocal();
    } else {
      Serial.println("sequence not right");
    }
  } else if (two == 3) {
    path = 1 ;
    for (int i = 0; i < 3; i++) {
      readAll();
      movement();
    }
    turnRight(turnSteps);
    Serial.println("Path 1");
    localized = true;
    blinkLocal();
  } else {
    Serial.println("sequence not right");
  }
}

int readWalls() {
  wallCounter = 0;
  boolean frontBlocked = false;
  boolean leftBlocked = false;
  boolean rightBlocked = false;
  boolean backBlocked = false;
  int left  = abs(distanceLeft1 - distanceLeft2);
  int right = abs(distanceRight1 - distanceRight2);

  //1 means 1 wall
  //2 means walls on left and right
  //3 means 3 walls
  //4 means 4 walls
  //5 means walls on front and 1 side or back and 1 side
  if ((left < 5 && right < 5) || (distanceLeft1 > 30 && distanceLeft2 > 30  && right < 5)) {
    if (distanceFront1 < distanceWall && distanceFront2 < distanceWall) {
      wallCounter++;
      frontBlocked = true;
      Serial.println("Front BLOCKED");
    }
    if (distanceLeft1 < distanceWall) {
      wallCounter++;
      leftBlocked = true;
      Serial.println("Left BLOCKED");
    }
    if (distanceRight1 < distanceWall) {
      wallCounter++;
      rightBlocked = true;
      Serial.println("Right BLOCKED");
    }
    if (distanceBack1 < distanceWall) {
      wallCounter++;
      backBlocked = true;
      Serial.println("Back BLOCKED");
    }
    if ((frontBlocked && leftBlocked && !rightBlocked && !backBlocked) || (frontBlocked && !leftBlocked && rightBlocked && !backBlocked) || (!frontBlocked && leftBlocked && !rightBlocked && backBlocked) || (!frontBlocked && !leftBlocked && rightBlocked && backBlocked)) {
      wallCounter = 5;
    }
  } else if (distanceFront1 > distanceWall && distanceFront2 > distanceWall && distanceLeft1 > distanceWall && distanceLeft2 > distanceWall && distanceRight1 > distanceWall && distanceRight2 > distanceWall && distanceBack1 > distanceWall) {
    wallCounter = 0;
  } else {
    wallCounter = 10;
    Serial.println("Not reading");
  }
  Serial.print("Wall Counter:");
  Serial.println(wallCounter);
}

int readFront1() {
  digitalWrite(trigPinFront1, LOW);
  delay(2);
  digitalWrite(trigPinFront1, HIGH);
  delay(10);
  digitalWrite(trigPinFront1, LOW);
  durationFront1 = pulseIn(echoPinFront1, HIGH);
  distanceFront1 = durationFront1 * 0.034 / 2;
  //  Serial.print("Front1:");
  //  Serial.println(distanceFront1);
}

int readFront2() {
  digitalWrite(trigPinFront2, LOW);
  delay(2);
  digitalWrite(trigPinFront2, HIGH);
  delay(10);
  digitalWrite(trigPinFront2, LOW);
  durationFront2 = pulseIn(echoPinFront2, HIGH);
  distanceFront2 = durationFront2 * 0.034 / 2;
  //  Serial.print("Front2:");
  //  Serial.println(distanceFront2);
}

int readLeft1() {
  digitalWrite(trigPinLeft1, LOW);
  delay(2);
  digitalWrite(trigPinLeft1, HIGH);
  delay(10);
  digitalWrite(trigPinLeft1, LOW);
  durationLeft1 = pulseIn(echoPinLeft1, HIGH);
  distanceLeft1 = durationLeft1 * 0.034 / 2;
  //Serial.print("Left1: ");
  //Serial.println(distanceLeft1);
}

int readLeft2() {
  digitalWrite(trigPinLeft2, LOW);
  delay(2);
  digitalWrite(trigPinLeft2, HIGH);
  delay(10);
  digitalWrite(trigPinLeft2, LOW);
  durationLeft2 = pulseIn(echoPinLeft2, HIGH);
  distanceLeft2 = durationLeft2 * 0.034 / 2;
  //Serial.print("Left2: ");
  //Serial.println(distanceLeft2);
}

int readRight1() {
  digitalWrite(trigPinRight1, LOW);
  delay(2);
  digitalWrite(trigPinRight1, HIGH);
  delay(10);
  digitalWrite(trigPinRight1, LOW);
  durationRight1 = pulseIn(echoPinRight1, HIGH);
  distanceRight1 = durationRight1 * 0.034 / 2;
  Serial.print("Right1: ");
  Serial.println(distanceRight1);
}

int readRight2() {
  digitalWrite(trigPinRight2, LOW);
  delay(2);
  digitalWrite(trigPinRight2, HIGH);
  delay(10);
  digitalWrite(trigPinRight2, LOW);
  durationRight2 = pulseIn(echoPinRight2, HIGH);
  distanceRight2 = durationRight2 * 0.034 / 2;
  Serial.print("Right2: ");
  Serial.println(distanceRight2);
}

int readBack1() {
  digitalWrite(trigPinBack1, LOW);
  delay(2);
  digitalWrite(trigPinBack1, HIGH);
  delay(10);
  digitalWrite(trigPinBack1, LOW);
  durationBack1 = pulseIn(echoPinBack1, HIGH);
  distanceBack1 = durationBack1 * 0.034 / 2;
  //Serial.print("Back1: ");
  //Serial.println(distanceBack1);
}

int readCube1() {
  digitalWrite(trigPinCube1, LOW);
  delay(2);
  digitalWrite(trigPinCube1, HIGH);
  delay(10);
  digitalWrite(trigPinCube1, LOW);
  durationCube1 = pulseIn(echoPinCube1, HIGH);
  distanceCube1 = durationCube1 * 0.034 / 2;
}

int readCube2() {
  digitalWrite(trigPinCube2, LOW);
  delay(2);
  digitalWrite(trigPinCube2, HIGH);
  delay(10);
  digitalWrite(trigPinCube2, LOW);
  durationCube2 = pulseIn(echoPinCube2, HIGH);
  distanceCube2 = durationCube2 * 0.034 / 2;
}

int readAll()
{
  readFront1();
  readFront2();
  readLeft1();
  readLeft2();
  readRight1();
  readRight2();
  readBack1();
}

int readCube() {
  readCube1();
  readCube2();
}

int forward(int steps) {
  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);
  while (stepperLeft.currentPosition() != (steps) && stepperRight.currentPosition() != (steps)) {
    stepperLeft.setSpeed(motorSpeedLeft);
    stepperRight.setSpeed(motorSpeedRight);
    stepperLeft.runSpeed();
    stepperRight.runSpeed();
  }
}

int turnRight(int steps) {
  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(steps);
  while (stepperLeft.currentPosition() != steps && stepperRight.currentPosition() != 0) {
    stepperLeft.setSpeed(70);
    stepperRight.setSpeed(-70);
    stepperLeft.runSpeed();
    stepperRight.runSpeed();
  }
}

int turnLeft(int steps) {
  stepperLeft.setCurrentPosition(steps);
  stepperRight.setCurrentPosition(0);
  while (stepperLeft.currentPosition() != 0 && stepperRight.currentPosition() != steps) {
    stepperLeft.setSpeed(-70);
    stepperRight.setSpeed(70);
    stepperLeft.runSpeed();
    stepperRight.runSpeed();
  }
}

/*
     LINK 1: https://www.instructables.com/id/Line-Follower-Robot-PID-Control-Android-Setup/
     LINK 2: http://robotsforroboticists.com/pid-control/
*/

int pidLeft() {
  // PID loop
  currentValue = distanceLeft1;    // calculate how centered robot is using left sensor
  error = 7.5 - currentValue;                         // error of 0 means robot is in the middle
  integral = integral + error;
  derivative = error - pastError;
  pastError = error;
  PIDvalue = (Kp * error) + (Ki * integral) + (Kd * derivative);

  // Adjust motor speed
  motorSpeedLeft = baseMotorSpeedLeft + PIDvalue;
  motorSpeedRight = baseMotorSpeedRight - PIDvalue;
  //Serial.print("Speed Left: ");
  //Serial.println(motorSpeedLeft);
  //Serial.print("Speed Right: ");
  //Serial.println(motorSpeedRight);
}

int pidRight() {
  // PID loop
  currentValue = distanceRight1;    // calculate how centered robot is using right sensor
  error = 7.5 - currentValue;                         // error of 0 means robot is in the middle
  integral = integral + error;
  derivative = error - pastError;
  pastError = error;
  PIDvalue = (Kp * error) + (Ki * integral) + (Kd * derivative);

  // Adjust motor speed
  motorSpeedLeft = baseMotorSpeedLeft - PIDvalue;
  motorSpeedRight = baseMotorSpeedRight + PIDvalue;
}

int down() {
  //Lowers the gripper
  lifter.slowmove(downPos, servoSpeed);
  delay(1000);
}

int lift() {
  //Only lifts the gripper if it is actuating for safety purposes
  if (gripper.read() == grabPos) {
    lifter.slowmove(liftPos, servoSpeed);
    delay(1000);
  } else {}
}

int grab() {
  //Actuates gripper
  gripper.slowmove(grabPos, servoSpeed);
  delay(1000);
}

int letGo() {
  //Unactuates gripper
  gripper.slowmove(letGoPos, servoSpeed);
  delay(1000);
}

void extractionHardCode()
{
  Serial.println("extraction started");
  down();
  delay(1000);
  letGo();
  delay(1000);
  motorSpeedLeft = baseMotorSpeedLeft;
  motorSpeedRight = baseMotorSpeedRight;
  forward(130);
  grab();
  delay(1000);
  lift();
  Serial.println("block has been lifted");
  delay(1000);
  readAll();
  while (distanceFront1 > limitDistanceFront && distanceFront2 > limitDistanceFront)
  {
    Serial.println("going forward");

    Serial.println("PID");
    // Choose right sensor for PID Control
    if (distanceRight1 < limitDistanceSide) {
      pidRightBlock();
      forward(normalStepForward);
      //stepsTracker = stepsTracker + normalStepsForward;
      //Serial.println("PID RIGHT");
    } // End PID right Control

    // Adjust motor speed to base values
    else {
      motorSpeedLeft = baseMotorSpeedLeft;
      motorSpeedRight = baseMotorSpeedRight;
      //stepsTracker = stepsTracker + normalStepsForward;
      // Move FORWARD without PID
      forward(normalStepForward);
    }
    readAll();
  }

  finalSequence = true;
}

void extraction()
{
  do {
    Serial.println ("moving closer");
    delay(2000);
    motorSpeedLeft = baseMotorSpeedLeft;
    motorSpeedRight = baseMotorSpeedRight;
    forward(25);
    readCube();
    Serial.println (distanceCube1);
    Serial.println (distanceCube2);
  } while ( (distanceCube1 > 21 && distanceCube2 > 21) && (distanceCube1 < 100 || distanceCube2 < 100) );


  Serial.println("Push Block");
  motorSpeedLeft = baseMotorSpeedLeft;
  motorSpeedRight = baseMotorSpeedRight;
  forward(200);
  backward(220);
  Serial.println("Grabbing");
  down();
  delay(1000);
  letGo();
  delay(1000);
  motorSpeedLeft = baseMotorSpeedLeft;
  motorSpeedRight = baseMotorSpeedRight;
  forward(130);
  grab();
  delay(1000);
  lift();
  delay(1000);

  // Move back to the wall
  readAll();
  while (distanceBack1 > limitDistanceFront)
  {
    Serial.println("Backwards");
    backward(20);
    readAll();
  }

  // Turn Right
  turnRight(turnSteps);

  // Move Forward

  // Move Forward using PID Control (left or right alignment)
  while (distanceFront1 > limitDistanceFront && distanceFront2 > limitDistanceFront)
  {
    Serial.println("PID");
    // Choose right sensor for PID Control
    if (distanceRight1 < limitDistanceSide) {
      pidRightBlock();
      forward(normalStepForward);
      //stepsTracker = stepsTracker + normalStepsForward;
      //Serial.println("PID RIGHT");
    } // End PID right Control

    // Adjust motor speed to base values
    else {
      motorSpeedLeft = baseMotorSpeedLeft;
      motorSpeedRight = baseMotorSpeedRight;
      //stepsTracker = stepsTracker + normalStepsForward;
      // Move FORWARD without PID
      forward(normalStepForward);
    }
    readAll();
  }
  done = 1;
  finalSequence = true;
  Serial.println("Finished");
  delay(3000);
}


int backward(int steps) {
  stepperLeft.setCurrentPosition(steps);
  stepperRight.setCurrentPosition(steps);
  while (stepperLeft.currentPosition() != (0) && stepperRight.currentPosition() != (0)) {
    stepperLeft.setSpeed(-baseMotorSpeedLeft);
    stepperRight.setSpeed(-baseMotorSpeedRight);
    stepperLeft.runSpeed();
    stepperRight.runSpeed();
  }
}

void dropOff()
{
  motorSpeedLeft = baseMotorSpeedLeft;
  motorSpeedRight = baseMotorSpeedRight;
  backward(130);
  down();
  delay(1000);
  letGo();
  delay(1000);
  motorSpeedLeft = baseMotorSpeedLeft;
  motorSpeedRight = baseMotorSpeedRight;
  backward(100);
  grab();
  delay(1000);
  lift();
  delay(1000);
}


void dynamicExtraction ()
{

  while (pass == 0 && done == 0)
  {
    sweep();

    // Move Forward using PID Control (left or right alignment)
    if (distanceFront1 > limitDistanceFront && distanceFront2 > limitDistanceFront)
    {
      Serial.println("PID");
      // Choose right sensor for PID Control
      if (distanceRight1 < limitDistanceSide) {
        pidRightBlock();
        forward(normalStepForward);
        //stepsTracker = stepsTracker + normalStepsForward;
        //Serial.println("PID RIGHT");
      } // End PID right Control

      // Adjust motor speed to base values
      else {
        motorSpeedLeft = baseMotorSpeedLeft;
        motorSpeedRight = baseMotorSpeedRight;
        //stepsTracker = stepsTracker + normalStepsForward;
        // Move FORWARD without PID
        forward(normalStepForward);
      }
    }

    // Turn left due to wall detected in front
    if ((distanceFront2 <= limitDistanceFront || distanceFront1 <= limitDistanceFront))
    {
      pass = 1;
      turnLeft(turnSteps);
    }
  }

  // Pass 2

  if (done == 0)
  {
    sweep();
  }

  if (done == 0)
  {
    for (int g = 0; g < 4; g++)
    {
      if (done == 0)
      {
        turnLeft(turnSteps);
        motorSpeedLeft = baseMotorSpeedLeft;
        motorSpeedRight = baseMotorSpeedRight;

        forward(80);
        do {
          readAll();

          // Alignment
          if (distanceLeft1 - distanceLeft2 > 0)
          {
            turnLeft(3);
            delay(500);
          }
          if (distanceLeft1 - distanceLeft2 < 0)
          {
            turnRight(3);
            delay(500);
          } // End of Alignment
          readAll();

        } while (abs(distanceLeft1 - distanceLeft2) > 1);

        turnRight(turnSteps);
        if (g != 1)
        {
          sweep();
        }
        else
        {
          Serial.println("SKIP");
          //delay(2000);
        }
      }
    }
  }
  Serial.println("No more Sweeping");
  //delay(3000);
}


int pidRightBlock() {
  // PID loop
  currentValue = distanceRight1;    // calculate how centered robot is using right sensor
  error = 6 - currentValue;                         // error of 0 means robot is in the middle
  integral = integral + error;
  derivative = error - pastError;
  pastError = error;
  PIDvalue = (Kp * error) + (Ki * integral) + (Kd * derivative);

  // Adjust motor speed
  motorSpeedLeft = baseMotorSpeedLeft - PIDvalue;
  motorSpeedRight = baseMotorSpeedRight + PIDvalue;
}

void sweep()
{
  readCube();
  readAll();
  if (((distanceFront1 - distanceCube1) > 7 || (distanceFront2 - distanceCube2) > 7) && ((distanceFront1 - distanceCube1) < 90 || (distanceFront2 - distanceCube2) < 90))
  {
    Serial.println("Block Detected");
    extraction();
  }
}
