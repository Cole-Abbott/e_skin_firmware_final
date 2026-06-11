// ===========================================
// Bit-banged SPI (no CS pin)
// DIN = MOSI (D7)
// CLK = SCK  (D3)
// LE  = optional latch (D6)
// Sends 16 bits MSB-first
// ===========================================

#define DIN  _BV(PD7)   // D7
#define CLK  _BV(PD3)   // D3
#define LE   _BV(PD6)   // D6
#define CLR  _BV(PB1)   // D9
#define TRIG _BV(PB3)   // D11
#define LED  _BV(PB5)   // D13 (onboard LED)
#define CLKP _BV(PD2)   // D2, Pulser Clock
#define Rxenable _BV(PD5)   // D5
#define FULL _BV(PB2)   // MUX cycle complete
#define TRIG2 _BV(PB0)

#define CLK_HALF_US  10   // half period
#define TRIG_PULSE   50   // the width of trigger signal in us

// switches 3, 5, 7
#define NUM_SWITCHES 6
uint8_t switch_arr[] = {3, 5, 7, 9, 11, 13};


uint8_t target_switch_index = 0;
// -------------------------------------------
// Setup
// -------------------------------------------
void setup() {
  DDRD |= DIN | CLK | LE | Rxenable | CLKP;   // Set PD3, PD5, PD6, PD7 as output
  DDRB |= CLR | TRIG | TRIG2 | FULL;       // Set PB1, PB3 as output

  PORTD &= ~(DIN | CLK | LE| CLKP);  // Start LOW
  PORTB &= ~(CLR | TRIG);

  // Turn LED ON
  PORTB |= LED;                 // LED HIGH   // <<< added
  // Serial.begin(115200); // Uncomment for debugging

  // PORTD |= Rxenable; // Set Rx enable high to always have ACD connected to signal

  close_switch();

  // PORTD &= ~Rxenable;     // Rx LOW
  PORTD |= Rxenable;      // Rx HIGH

}


void close_switch() {
  // Loop 32 times for a 32-bit value
  for (int i = 32; i >= 0; i--) {
    // Output MSB first (Bit 31 for a 32-bit value)
    if (i == switch_arr[target_switch_index]) // The new 32-bit MSB mask
      PORTD |= DIN;        // DIN HIGH
    else
      PORTD &= ~DIN;       // DIN LOW

    // Generate the clock pulse (Clock Low, then Clock High)
    // PORTD &= ~(CLK);         // CLK, CLKP LOW
    PORTD &= ~(CLK | CLKP);         // CLK, CLKP LOW
    _delay_us(CLK_HALF_US);
    PORTD |= (CLK | CLKP);          // CLK HIGH
    // PORTD |= (CLK);          // CLK HIGH
    _delay_us(CLK_HALF_US);
  }

  // Final cleanup and Latch Enable (LE) pulse
  PORTD &= ~(CLK | DIN | CLKP);   // CLK LOW, DIN LOW, CLKP LOW
  _delay_us(CLK_HALF_US * 2);

  // Latch the data (LE LOW then LE HIGH)
  PORTD &= ~LE;            // LE LOW
  _delay_us(CLK_HALF_US);
  PORTD |= LE;             // LE HIGH


}


void loop() {
  

  // ------------close switch-------------------
  // start timing
  unsigned long start = micros();

  close_switch();
  unsigned long switch_start = micros();
  // ------------close switch-------------------

  // --------------actuating and sensing ---------
  // trigger the HV pulse
  PORTB |= (TRIG2 | TRIG);      // Trigger HIGH
  // PORTB |= TRIG2;      // Trigger HIGH

  if (target_switch_index == 0) {
    PORTB |= FULL;
  }

  _delay_us(TRIG_PULSE);
  // PORTB &= ~TRIG2;     // Trigger LOW
  PORTB &= ~(TRIG2 | TRIG);     // Trigger LOW

  // actuate and sense
  // PORTD &= ~Rxenable;     // Rx LOW
  while ((micros() - switch_start) < 200) {// close switch for 200us
  }
  // PORTD |= Rxenable;      // Rx HIGH
  
  PORTB &= ~FULL;
  
  // // open all switches
  PORTB |= CLR;       // CLR HIGH
  _delay_us(CLK_HALF_US);
  PORTB &= ~CLR;      // CLR LOW
  // --------------actuating and sensing ---------

  // ------------prepare next channel-----------
  // update the switch
  target_switch_index++;
  if (target_switch_index >= NUM_SWITCHES) target_switch_index = 0;
  // ------------prepare next channel-----------

  // ------------wait the rest of time (1 ms total period)-----------
  while ((micros() - start) < 1000) {
    // do nothing — wait until 1 ms has passed
  }
  // ------------wait the rest of time (1 ms total period)-----------
}
