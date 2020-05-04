#include <AccelStepper.h>   //include the stepper motor library

#define stepXPin 2        //right stepper motor step pin
#define dirXPin 5         //right stepper motor direction pin
#define stepperEnable 8    //stepper enable pin on stepStick 
#define stepperEnTrue false //variable for enabling stepper motor
#define stepperEnFalse true //variable for disabling stepper motor

#define speedD 8000          //default speed
#define accelD 500          //default acceleration


AccelStepper stepperX(AccelStepper::DRIVER, stepXPin, dirXPin);    //create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)


void setup() {
  // put your setup code here, to run once:

  pinMode(stepXPin, OUTPUT);                     //sets pin as output
  pinMode(dirXPin, OUTPUT);                      //sets pin as output
  pinMode(stepperEnable, OUTPUT);                 //sets pin as output

  digitalWrite(stepperEnable, stepperEnFalse);    //turns off the stepper motor





  stepperX.setMaxSpeed(speedD);               //set the maximum speed for the right stepper
  stepperX.setAcceleration(accelD);           //set the initial acceleration for the right stepper

  digitalWrite(stepperEnable, stepperEnTrue);     //turns on the stepper motor driver

}

void loop() {
  // put your main code here, to run repeatedly:

  delay(500);
  forward(1, 5000, speedD);
  delay(500);

}

void forward(int motor, int dist, int spd) {

  stepperX.move(dist);
  stepperX.setMaxSpeed(spd);
  runXToStop();

}

void runXToStop ( void ) {
  int runX = 1;   //state variabels
  while (runX) {        //until both stop
    if (!stepperX.run()) {  //step the right stepper, if it is done moving set runR = 0
      runX = 0;                 //right done moving
    }
  }
}
