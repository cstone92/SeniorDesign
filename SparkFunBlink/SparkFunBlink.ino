// Pins
const int led_pin = PB5;

// counter and compare values
const uint16_t t1_load = 0;
const uint16_t t1_comp = 8;
//   CS22  | CS21  | CS20  |  DEC |Description
//       0 |     0 |     0 |   0  |stop timer
//       0 |     0 |     1 |   1  |prescaler = 1 (no prescaller)
//       0 |     1 |     0 |   2  |prescaler = 8
//       0 |     1 |     1 |   3  |prescaler = 64
//       1 |     0 |     0 |   4  |prescaler = 256
//       1 |     0 |     1 |   5  |prescaler = 1024
uint8_t prescalerMode = 0x05;


boolean pulse = false;

void setup() {
  // Set LED pin to be output
  pinMode(45, OUTPUT); // output pin for OCR5B, this is Arduino pin number
  pinMode(30, OUTPUT);

  // Reset Timer1 Control Reg A
  TCCR5A = 0;

  // set waveform generation mode WGM
  TCCR5B &= ~(1 << WGM53); //clears
  TCCR5B &= ~(1 << WGM52); //clears
  TCCR5B &= ~(1 << WGM51); //clears
  TCCR5B &= ~(1 << WGM50); //clears
  TCCR5B |= (1 << WGM52); //sets

  // Set Prescaller of 256
  TCCR5B &= ~0b111;  // this operation (AND plus NOT),  set the three bits in TCCR2B to 0 (clears prescaler)
  TCCR5B = (TCCR5B & 0b11111000) | (prescalerMode);

  // Reset Timer1 and set compare values
  TCNT5 = t1_load;
  OCR5A = t1_comp;

  // Enable timer1 compare interrupt
  TIMSK5 = (1 << OCIE5A);

  // Enable global interrupts
  sei();
}

void loop() {
  delay(500);
}

ISR(TIMER5_COMPA_vect) {
  pulse = !(pulse);
  digitalWrite(45, pulse);
  //TCNT1 = t1_load;
}
