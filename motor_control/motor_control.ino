
#include <AccelStepper.h>  //include the stepper motor library
#include <avr/io.h>
#include <util/delay.h>

//40,000 step/rev

//The Timer/Counter Control Registers TCCRnA and TCCRnB hold the main control bits for the timer.
//(Note that TCCRnA and TCCRnB do not correspond to the outputs A and B.) These registers hold several groups of bits:

//Waveform Generation Mode bits (WGM): these control the overall mode of the timer. (These bits are split between TCCRnA and TCCRnB.)
//Clock Select bits (CS): these control the clock prescaler
//Compare Match Output A Mode bits (COMnA): these enable/disable/invert output A
//Compare Match Output B Mode bits (COMnB): these enable/disable/invert output B

//The Output Compare Registers OCRnA and OCRnB set the levels at which outputs A and B will be affected.
//When the timer value matches the register value, the corresponding output will be modified as specified by the mode

// The main PWM modes are "Fast PWM" and "Phase-correct PWM"
//      fast PWM is the simplest PWM mode, the timer repeatedly counts from 0 to 255.
//      The output turns on when the timer is at 0, and turns off when the timer matches the output compare register
//
//      Phase-correct PWM is when the timer counts from 0 to 255 and then back down to 0.
//      The output turns off as the timer hits the output compare register value on the way up, and turns back on as the timer hits the output compare register value on the way down
//      The output frequency will be approximately half of the value for fast PWM mode, because the timer runs both up and down.
//
//      Both fast PWM and phase correct PWM have an additional mode that gives control over the output frequency.
//      In this mode, the timer counts from 0 to OCRA (the value of output compare register A), rather than from 0 to 255
//      Note that in this mode, only output B can be used for PWM; OCRA cannot be used both as the top value and the PWM compare value

// timer 0 ---> TCCR0B ---> (controls pin 13, 4);        //timer 0 is the one on which rely all time functions in Arduino:
//i.e., if you change this timer, function like delay() or millis
//() will continue to work but at a different timescale (quicker or slower!!!)

// timer 1 ---> TCCR1B ---> (controls pin 12, 11);
// timer 2 ---> TCCR2B ---> (controls pin 10, 9);
// timer 3 ---> TCCR3B ---> (controls pin 5, 3, 2);
// timer 4 ---> TCCR4B ---> (controls pin 8, 7, 6);
// timer 5 ---> TCCR5B ---> (controls pin 44, 45, 46) (this one was not included on some websites)

//open timer pins 9, 10, 11, 12, 13 (3, 6) (44, 45, 46)
//going to hopfully use pins 9 (TCCR2B, OC2B) and 10 (TCCR2A, OC2A)

// prescaler = CS values
// prescaler values are good for all timers (TCCR1B, TCCR2B, TCCR3B, TCCR4B) except for timer 0 (TCCR0B).
// prescaler = 0 ---> OFF
// prescaler = 1 ---> 1 ---> PWM frequency is 31000 Hz
// prescaler = 2 ---> 8 ---> PWM frequency is 4000 Hz
// prescaler = 3 ---> 64 ---> PWM frequency is 490 Hz (default value)
// prescaler = 4 ---> 256---> PWM frequency is 120 Hz
// prescaler = 5 ---> 1024 ---> PWM frequency is 30 Hz
// prescaler = 6 ---> big ---> PWM frequency is <20 Hz

// for timer 0 (TCCR0B) the values are: because 8 bit instead of 128 instead of 16 bit 256 max
// prescaler = 0 ---> OFF
// prescaler = 1 ---> 1 ---> PWM frequency is 62000 Hz
// prescaler = 2 ---> 8 ---> PWM frequency is 7800 Hz
// prescaler = 3 ---> 64 ---> PWM frequency is 980 Hz (default value)
// prescaler = 4 ---> 254 ---> PWM frequency is 250 Hz
// prescaler = 5 ---> 1024 ---> PWM frequency is 60 Hz
// prescaler = 6 ---> big ---> PWM frequency is <20 Hz

// (prescalers equal to 0  or 7 are useless). (not really)

// define pin numbers for stepper
#define stepXPin 2  //right stepper motor step pin
#define dirXPin 5   //right stepper motor direction pin
#define stepZPin 4
#define dirZPin 7
#define Pul 9
// #define Dir 9
#define stepperEnable 8      //stepper enable pin on stepStick
#define stepperEnTrue false  //variable for enabling stepper motor
#define stepperEnFalse true  //variable for disabling stepper motor

// // Stepper Library Default Speeds
#define speedD 1000  // default speed
#define accelD 300   // default acceleration

// boolean pulse = 1;
// boolean accurate = 0;
boolean moving = 0;

uint8_t state = 0;

  uint8_t prescalerMode = 0x05;
  uint8_t sec_per_rev = 30;  //you pick this

// unsigned int count = 0;
// unsigned int maxCount = 500;

// Stepper Setup
AccelStepper stepperX(AccelStepper::DRIVER, stepXPin, dirXPin);  //create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperZ(AccelStepper::DRIVER, stepZPin, dirZPin);   //create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)

void setup() {
  // put your setup code here, to run once:
  // Added to improve debugging capability
  Serial.begin(9600);  // open the serial port at 9600 baud

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  uint32_t clockRate = 16000000;
  uint16_t prescaler = 64;  //determined from prescalerMode
  uint8_t gearRatio = 14;
  uint16_t step_per_rev = 40000;  // determined by SW1-4 on closed loop stepper driver

  //going to hopfully use pins 9 (TCCR2B, OC2B) and 10 (TCCR2A, OC2A)
  pinMode(Pul, OUTPUT);  //output pin for OCR2B, this is Arduino pin number
  pinMode(10, OUTPUT);   //output pin for OCR2A, controlling the top limit

  // In the next line of code, we:
  // 1. Set the compare output mode to clear OC2A and OC2B on compare match.
  //    To achieve this, we set bits COM2A1 and COM2B1 to high.
  // 2. Set the waveform generation mode to fast PWM (mode 3 in datasheet).
  //    To achieve this, we set bits WGM21 and WGM20 to high.
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);

  // In the next line of code, we:
  // 1. Set the waveform generation mode to fast PWM mode 7 —reset counter on
  //    OCR2A value instead of the default 255. To achieve this, we set bit
  //    WGM22 to high.
  // 2. Set the prescaler divisor to 1, so that our counter will be fed with
  //    the clock's full frequency (16MHz). To achieve this, we set CS20 to
  //    high (and keep CS21 and CS22 to low by not setting them).
  //
  //   CS22  | CS21  | CS20  |  DEC |Description
  //       0 |     0 |     0 |   0  |stop timer
  //       0 |     0 |     1 |   1  |prescaler = 1 (no prescaller)
  //       0 |     1 |     0 |   2  |prescaler = 8
  //       0 |     1 |     1 |   3  |prescaler = 64
  //       1 |     0 |     0 |   4  |prescaler = 256
  //       1 |     0 |     1 |   5  |prescaler = 1024

  TCCR2B &= ~0b111;  // this operation (AND plus NOT),  set the three bits in TCCR2B to 0 (clears prescaler)
                     //now that CS02, CS01, CS00  are clear, we write on them a new value:
  //TCCR2B = _BV(WGM22) | _BV(CS21) | _BV(CS20);
  TCCR2B = _BV(WGM22);
  TCCR2B = (TCCR2B & 0b11111000) | (prescalerMode);

  // OCR2A holds the top value of our counter, so it acts as a divisor to the
  // clock. When our counter reaches this, it resets. Counting starts from 0.
  // Thus 63 equals to 64 divs.
  //int top = (clockRate * sec_per_rev) / (step_per_rev * gearRatio * prescaler);
  //Serial.println(top,DEC);
  OCR2A = 3;
  // This is the duty cycle. Think of it as the last value of the counter our
  // output will remain high for. Can't be greater than OCR2A of course. A
  // value of 0 means a duty cycle of 1/64 in this case.
  OCR2B = 1;
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // Pin initialization
  pinMode(stepXPin, OUTPUT);  //sets pin as output
  pinMode(dirXPin, OUTPUT);   //sets pin as output
  pinMode(stepZPin, OUTPUT);       //sets pin as output
  pinMode(dirZPin, OUTPUT);        //sets pin as output
  pinMode(stepperEnable, OUTPUT);  //sets pin as output
  //pinMode(Pul, OUTPUT);
  //pinMode(Dir, OUTPUT);
  digitalWrite(stepperEnable, stepperEnFalse);  //turns off the stepper motor

  // Stepper initialization
     stepperX.setMaxSpeed(speedD);      //set the maximum speed for the right stepper
     stepperX.setAcceleration(accelD);  //set the initial acceleration for the right stepper
     stepperZ.setMaxSpeed(speedD);      //set the maximum speed for the right stepper
     stepperZ.setAcceleration(accelD);  //set the initial acceleration for the right stepper

     digitalWrite(stepperEnable, stepperEnTrue);  //turns on the stepper motor driver

  //  Timer1.initialize(timer_int);
  //  Timer1.attachInterrupt(nextStep);
  //  Timer1.stop();
}

void loop() {
  // put your main code here, to run repeatedly:{

  if (Serial.available() > 0) {
    if (Serial.peek() == 'c') {
      Serial.read();
      state = Serial.parseInt();
      moving = false;
      //digitalWrite(LED_BUILTIN, state);
    }
    while (Serial.available() > 0) {
      Serial.read();
    }
  }

  if (~(moving)) {
    switch (state) {
      case 1:
        forward(1, 2000, speedD);
        break;
       case 2:
         forward(2, 2000, speedD);
         break;
      case 3:
      TCCR2B = (TCCR2B & 0b11111000) & ~(prescalerMode);
      Serial.println(_BV(TCCR2B));
      moving = false;
      break;
            case 4:
      TCCR2B = (TCCR2B & 0b11111000) | prescalerMode;
      Serial.println(_BV(TCCR2B));
      moving = true;
      break;
      default:
        Serial.println("Catch-all");
        delay(500);
        break;
    }
  } else {
    // if (accurate) {
    //   digitalWrite(Dir, HIGH);
    //   if (count > maxCount) {
    //     //Timer1.stop();
    //     accurate = false;
    //     count = 0;
    //     moving = 0;
    //   }
    // }
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
      runXToStop();  //move to the desired position
      break;
     case 2:
       stepperZ.move(dist);
       stepperZ.setMaxSpeed(spd);
       runZToStop();  //move to the desired position
       break;
    default:
      break;
  }
}

/*
  runToStop runs both the right and left stepper until they stop moving
*/
void runXToStop(void) {
  boolean runX = 1;  //state variabels
  moving = 1;
  while (runX) {            //until both stop
    if (!stepperX.run()) {  //step the right stepper, if it is done moving set runR = 0
      runX = 0;             //right done moving
    }
  }
  moving = 0;
}

 void runZToStop(void) {
   boolean runZ = 1;  //state variables
   moving = 1;
   while (runZ) {            //until both stop
     if (!stepperZ.run()) {  //step the left stepper, if it is done moving set runL = 0
       runZ = 0;             //left done moving
     }
   }
   moving = 0;
 }

// void nextStep() {
//   pulse = !(pulse);
//   digitalWrite(Pul, pulse);
//   count++;
//delay(1);

//pulse = 1;
//digitalWrite(Pul, pulse);
//count++;
//delay(1);
//pulse = 0;
//degitalWrite(Pul, pulse);
//}
