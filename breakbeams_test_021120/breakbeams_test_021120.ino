// Bounce.pde
// -*- mode: C++ -*-
//
// Make a single stepper bounce from one limit to another
//
// Copyright (C) 2012 Mike McCauley
// $Id: Random.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $

#include <TimerOne.h>       //include library for timers with interrupts

#include <AccelStepper.h>


// define pin numbers for stepper
#define stepPin 2        //right stepper motor step pin
#define dirPin 5         //right stepper motor direction pin
#define stepperEnable 8    //stepper enable pin on stepStick 
#define stepperEnTrue false //variable for enabling stepper motor
#define stepperEnFalse true //variable for disabling stepper motor

#define bottomBreakBeam 12
#define topBreakBeam 13


// Stepper Library Default Speeds
#define speedD 4000          //default speed
#define accelD 2000          //default acceleration

#define timer_rate 5                   // sensor update calls per second
#define timer_int 1000000/timer_rate   // timer interrupt interval in microseconds

long count = 0;
boolean top = 1;
boolean bottom = 1;

int forward = -1;

byte State = 1;

byte drive = 1;
byte reverse = 2;
byte slow = 3;


// Stepper Setup
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);    //create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)

void setup()
{

  // put your setup code here, to run once:
  // Added to improve debugging capability
  Serial.begin(9600); // open the serial port at 9600 baud

  // Pin initialization
  pinMode(stepPin, OUTPUT);                     //sets pin as output
  pinMode(dirPin, OUTPUT);                      //sets pin as output
  pinMode(stepperEnable, OUTPUT);                 //sets pin as output
  pinMode(topBreakBeam, INPUT_PULLUP);
  pinMode(bottomBreakBeam, INPUT_PULLUP);
  digitalWrite(stepperEnable, stepperEnTrue);    //turns on the stepper motor

  stepper.setMaxSpeed(speedD);
  stepper.setAcceleration(accelD);

  // Change these to suit your stepper if you want
  stepper.moveTo(10000);

  //Timer Interrupt Set Up
//  Timer1.initialize(timer_int);                   // initialize timer1, and set a timer_int second period
//  Timer1.attachInterrupt(updateState);           // attaches updateIR() as a timer overflow interrupt

}

void loop()
{
  
//  top = digitalRead(topBreakBeam);
//  bottom = digitalRead(bottomBreakBeam);
//  
//  while(top!=0 && bottom !=0){
//    runToBreakBeam();
//  }
  
  updateState();
}


void updateState() {

  top = digitalRead(topBreakBeam);
  bottom = digitalRead(bottomBreakBeam);

  //Serial.print("BreakBeams");
  //Serial.print(top);
  //Serial.print("  |  ");
  //Serial.print(bottom);
  //Serial.println();


//runToBreakBeam();

  if (State == drive) {
    if (forward == 1) {
      if (bottom == 0) {
       
        //stepper.stop();

        Serial.print(State);
        Serial.print("  |  ");
        Serial.println(count);

        forward = forward * (-1);
        State = reverse;

        stepper.setMaxSpeed(2000);
        stepper.setAcceleration(2000);
      }
    } else if (forward == -1) {
        if (top == 0) {
          
          //stepper.stop();
  
          Serial.print(State);
          Serial.print("  |  ");
          Serial.println(count);
  
          forward = forward * (-1);
          State = reverse;
  
          stepper.setMaxSpeed(2000);
          stepper.setAcceleration(2000);
        }
      }
  } else if (State == reverse) {
    if (top == 1 && bottom == 1) {

     // stepper.stop();


      Serial.print(State);
      Serial.print("  |  ");
      Serial.println(count);

      forward = forward * (-1);
      State = slow;

      stepper.setMaxSpeed(300);
      stepper.setAcceleration(500);
    }

  } else if (State == slow) {
    if (forward == 1) {
      if (bottom == 0) {

        //stepper.stop();

        Serial.print(State);
        Serial.print("  |  ");
        Serial.println(count);

        count = 0;
        forward = forward * (-1);
        State = drive;

        stepper.setMaxSpeed(speedD);
        stepper.setAcceleration(accelD);
      }

    } else if (forward == -1) {
      if (top == 0) {

        //stepper.stop();


        Serial.print(State);
        Serial.print("  |  ");
        Serial.println(count);

        count = 0;
        forward = forward * (-1);
        State = drive;

        stepper.setMaxSpeed(speedD);
        stepper.setAcceleration(accelD);
      }
    }


  } else {
//    Serial.println("error");
//      runToBreakBeam();
  }
  runToBreakBeam();
}


void runToBreakBeam () {

  if (!stepper.run());
  stepper.move(10000 * forward);
  count = count + forward;

}
