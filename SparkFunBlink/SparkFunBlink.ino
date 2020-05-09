// Pins
const int led_pin = PB5;

// counter and compare values
const uint16_t t1_load 0;
const uint16_t t1_comp 31250;

//   CS22  | CS21  | CS20  |  DEC |Description
//       0 |     0 |     0 |   0  |stop timer
//       0 |     0 |     1 |   1  |prescaler = 1 (no prescaller)
//       0 |     1 |     0 |   2  |prescaler = 8
//       0 |     1 |     1 |   3  |prescaler = 64
//       1 |     0 |     0 |   4  |prescaler = 256
//       1 |     0 |     1 |   5  |prescaler = 1024
uint8_t prescalerMode = 0x04;

void setup() {
  // Set LED pin to be output
  DDRB |= (1 << led_pin);

  // Reset Timer1 Control Reg A
  TCCR1A = 0;

  // set waveform generation mode WGM
  TCCR1B &= ~(1 << WGM13); //clears
  TCCR1B &= ~(1 << WGM12); //clears
  TCCR1B &= ~(1 << WGM11); //clears
  TCCR1B &= ~(1 << WGM10); //clears
  TCCR1B |= (1 << WGM12); //sets

  // Set Prescaller of 256
  TCCR1B &= ~0b111;  // this operation (AND plus NOT),  set the three bits in TCCR2B to 0 (clears prescaler)
  TCCR1B = (TCCR1B & 0b11111000) | (prescalerMode);

  // Reset Timer1 and set compare values
  TCNT1 = t1_load;
  OCR1A = t1_comp;

  // Enable timer1 compare interrupt
  TIMSK1 = (1 << OCIE1A);

  // Enable global interrupts
  sei();
}

loop() {
  delay(500);
}

ISR(TIMER1_COMPA_vect) {
  TCNT1 = t1_load;
}