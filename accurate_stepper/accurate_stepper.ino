
#include <AccelStepper.h>   //include the stepper motor library

// define pin numbers for stepper
#define stepXPin 2        //right stepper motor step pin
#define dirXPin 5         //right stepper motor direction pin
#define stepYPin 5
#define dirYPin 7
#define Pul 10
#define Dir 9
#define stepperEnable 8    //stepper enable pin on stepStick 
#define stepperEnTrue false //variable for enabling stepper motor
#define stepperEnFalse true //variable for disabling stepper motor


// Stepper Library Default Speeds
#define speedD 1000         // default speed
#define accelD 300          // default acceleration

#define timer_rate 10                    // sensor update calls per second
#define timer_int 1000000/timer_rate    // timer interrupt interval in microseconds

boolean pulse = 1;
boolean accurate = 0;
boolean moving = 0;

byte state = 0;

unsigned int count = 0;
unsigned int maxCount = 500;


// Stepper Setup
AccelStepper stepperX(AccelStepper::DRIVER, stepXPin, dirXPin);    //create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperY(AccelStepper::DRIVER, stepYPin, dirYPin);


void setup() {
  // put your setup code here, to run once:
  // Added to improve debugging capability
  Serial.begin(9600); // open the serial port at 9600 baud

  // Pin initialization
  pinMode(stepXPin, OUTPUT);                     //sets pin as output
  pinMode(dirXPin, OUTPUT);                      //sets pin as output
  pinMode(stepYPin, OUTPUT);                     //sets pin as output
  pinMode(dirYPin, OUTPUT);                      //sets pin as output
  pinMode(stepperEnable, OUTPUT);                 //sets pin as output
  pinMode(Pul, OUTPUT);
  pinMode(Dir, OUTPUT);
  digitalWrite(stepperEnable, stepperEnFalse);    //turns off the stepper motor

  // Stepper initialization
  stepperX.setMaxSpeed(speedD);               //set the maximum speed for the right stepper
  stepperX.setAcceleration(accelD);           //set the initial acceleration for the right stepper
  stepperY.setMaxSpeed(speedD);               //set the maximum speed for the right stepper
  stepperY.setAcceleration(accelD);           //set the initial acceleration for the right stepper

  digitalWrite(stepperEnable, stepperEnTrue);     //turns on the stepper motor driver

  //  Timer1.initialize(timer_int);
  //  Timer1.attachInterrupt(nextStep);
  //  Timer1.stop();


}

void loop() {
  // put your main code here, to run repeatedly:{

  if (Serial.available() > 0) {
    //Serial.println("2");        //debugging
    if (Serial.peek() == 'c') {
      Serial.read();
      state = Serial.parseInt();
      //digitalWrite(LED_BUILTIN, state);
    }
    while (Serial.available() > 0) {
      Serial.read();
    }
  }

  if (~(moving)) {
    //Serial.println("1");
    switch (state) {
      case 1:
        forward(1, 2000, speedD);
        break;
      case 2:
        forward(2, 2000, speedD);
        break;
      //case 3:
      //count = 0;
      //accurate = true;
      //moving = 1;
      //Timer1.start();
      //break;
      default:
        Serial.println("Catch-all");
        delay(500);
        break;
    }
  } else {
    //    if (accurate) {
    //      digitalWrite(Dir, HIGH);
    //      if (count > maxCount) {
    //        //Timer1.stop();
    //        accurate = false;
    //        count = 0;
    //        moving = 0;
    //}
    //}
  }
}

/*
  forward has two inputs: an integer for the distance to travel in inches, and an
  integer for the speed of travel.

  dist is a value from -1000 to 1000 in inches
  spd is a value from 100 to 1000 in steps/sec

  The function then sets the desired position using stepper.move(dist) on both inputs.
  It then runs a while loop that waits until both steppers complete their movement that
  calls left.run() and right.run() each loop. This allows us to improve precision by
  using the acceleration option in the stepper library to eliminate skipped steps and
  wheel skid during the start and end of movement.
*/
void forward(int motor, int dist, int spd) {
  switch (motor) {
    case 1:
      stepperX.move(dist);
      stepperX.setMaxSpeed(spd);
      runXToStop();                  //move to the desired position

      break;
    case 2:
      stepperY.move(dist);
      stepperY.setMaxSpeed(spd);
      runYToStop();                  //move to the desired position

      break;
    default:
      break;
  }
}


/*
  runToStop runs both the right and left stepper until they stop moving
*/
void runXToStop ( void ) {
  boolean runX = 1;   //state variabels
  moving = 1;
  while (runX) {        //until both stop
    if (!stepperX.run()) {  //step the right stepper, if it is done moving set runR = 0
      runX = 0;                 //right done moving
    }
  }
  moving = 0;
}
void runYToStop ( void ) {
  boolean runY = 1;   //state variables
  moving = 1;
  while (runY) {        //until both stop
    if (!stepperY.run()) {   //step the left stepper, if it is done moving set runL = 0
      runY = 0;                 //left done moving
    }
  }
  moving = 0;
}

void nextStep() {

  pulse = !(pulse);
  digitalWrite(Pul, pulse);
  count++;
  //delay(1);

  //pulse = 1;
  //digitalWrite(Pul, pulse);
  //count++;
  //delay(1);
  //pulse = 0;
  //degitalWrite(Pul, pulse);



}
