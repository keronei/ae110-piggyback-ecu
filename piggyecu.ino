#include <avr/pgmspace.h>
#include <EEPROM.h>
#include <avr/wdt.h>

#include <avr/sleep.h>

#define myubbr (16000000 / 16 / 9600 - 1)

volatile unsigned long timer2_overflow_count;

volatile float vbatt_filtered = 12.0;  // initial guess (safe)

volatile uint8_t batteryTick = 0;

unsigned long microSeconds(void) {
  unsigned long m;
  uint8_t t;
  uint8_t oldSREG = SREG;  // Save the current interrupt state

  cli();  // Disable interrupts
  m = timer2_overflow_count;
  t = TCNT2;

  // Check if Timer2 overflowed JUST before or during our read.
  // TIFR2 is the Timer Interrupt Flag Register.
  // TOV2 is the Timer Overflow Flag bit.
  if ((TIFR2 & _BV(TOV2)) && (t < 255)) {
    m++;
  }
  SREG = oldSREG;  // Restore interrupts to what they were (safely!)

  // Math: (Overflows * 256 + TimerTicks) * prescaler_factor
  // Assuming 16MHz and 128 prescaler (8us per tick)
  return ((m << 8) + t) * 4;
}


unsigned long elapsedMicroseconds(unsigned long startMicroSeconds, unsigned long currentMicroseconds) {
  if (currentMicroseconds >= startMicroSeconds) {
    return currentMicroseconds - startMicroSeconds;
  }
  return 0xFFFFFFFF - (startMicroSeconds - currentMicroseconds);
}

unsigned long elapsedMicroseconds(unsigned long startMicroSeconds) {
  return elapsedMicroseconds(startMicroSeconds, microSeconds());
}

char intBuffer[12];

int gtcount = 0;
int allgtcount = 0;

volatile bool g_coil_trigger_active = false;

volatile int igtcountmiss = 0;
volatile int synccount = 0;

const int IGF_PIN = 4;

const int INDICATOR = 5;

const int SYNC_STATE_IND = 2;

const int COIL_4_PIN = 34;  // Red cyl 1-3-4-2
const int COIL_3_PIN = 32;  // Purple cyl
const int COIL_2_PIN = 38;  // Blue cyl
const int COIL_1_PIN = 30;  // Orange cyl


// State variables
volatile int nextCoilToFire = 0;

volatile unsigned long last_igt_time = 0;

volatile bool edgeDetected = false;
volatile unsigned long lastEdgeTime = 0;
volatile byte edgeCount = 0;

enum SyncState { WAITING_FOR_EDGE,
                 VALIDATING,
                 SYNCED };

SyncState syncState = WAITING_FOR_EDGE;
unsigned long firstEdgeTime = 0;
unsigned long edgeInterval = 0;
byte validEdgeCount = 0;

const unsigned long DWELL_MIN_US = 1000UL;
const unsigned long DWELL_MAX_US = 4000UL;

// --- Volatile state for ISR-friendly operation ---
volatile unsigned long lastDwellRequestMicros = 0;
volatile unsigned long currentDwellUs = 2000;  // default in case

// IIR smoothing
const float VBATT_ALPHA = 0.08;  // smoothing factor (0..1). larger = faster response

// Dwell table (voltage ascending)
const uint8_t TABLE_LEN = 13;
const float dwell_v[TABLE_LEN] = { 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 12.5, 13.0, 13.5, 14.0, 14.5, 15.0, 16.0 };
const unsigned long dwell_us[TABLE_LEN] = {
  4000UL, 3600UL, 3200UL, 2800UL, 2400UL, 2000UL, 1850UL, 1700UL, 1550UL, 1400UL, 1300UL, 1200UL, 1000UL
};

void validateSync() {

  synccount++;

  if (edgeDetected) {

    edgeDetected = false;
    unsigned long t = lastEdgeTime;

    switch (syncState) {
      case WAITING_FOR_EDGE:
        firstEdgeTime = t;
        validEdgeCount = 1;
        syncState = VALIDATING;
        break;

      case VALIDATING:
        edgeInterval = t - firstEdgeTime;
        validEdgeCount++;

        if (validEdgeCount >= 2) {
          syncState = SYNCED;
          nextCoilToFire = COIL_2_PIN;
        }
        break;

      case SYNCED:
        break;
    }
  }
}

volatile unsigned long lastEdgeMicros = 0;
volatile unsigned int igtCountRejected = 0;
volatile unsigned int igtCountDeadTime = 0;
volatile unsigned int igtWhileUnsynced = 0;

static uint32_t lastIGF = 0;

volatile unsigned long ignore_igt_until = 0;
#define IGT_MIN_DEBOUNCE_US 500   // tune: 150..500
#define IGT_STABLE_VERIFY_US 120  // verify signal still high after N us


void onIGTRising(void) {
  allgtcount++;

  if (syncState != SYNCED) {

    igtWhileUnsynced++;
    return;
  }

  unsigned long t = microSeconds();
  // simple coarse debounce to avoid extremely high frequency interrupts
  if ((t - lastEdgeMicros) <= IGT_MIN_DEBOUNCE_US) {  // 500 µs dead-time; tune to your engine
    igtCountRejected++;
    return;  // too early
  }

  if (t <= ignore_igt_until) {
    igtCountDeadTime++;
    return;
  }

  lastDwellRequestMicros = microSeconds();

  lastEdgeMicros = t;

  if (!g_coil_trigger_active) {
    gtcount++;

    //Direct port manipulation for speed
    if (nextCoilToFire == COIL_1_PIN) {
      PORTC |= (1 << PC7);  // Pin 30 HIGH
    } else if (nextCoilToFire == COIL_3_PIN) {
      PORTC |= (1 << PC5);  // Pin 32 HIGH
    } else if (nextCoilToFire == COIL_4_PIN) {
      PORTC |= (1 << PC3);  // Pin 34 HIGH
    } else if (nextCoilToFire == COIL_2_PIN) {
      PORTD |= (1 << PD7);  // Pin 38 HIGH
    }

    last_igt_time = t;

    uint16_t ocr = (currentDwellUs * 2) - 1;

    TCCR1B = 0;
    TCNT1 = 0;  // Reset Timer1 count
    TIFR1 = (1 << OCF1A);
    OCR1A = ocr;

    g_coil_trigger_active = true;

    TIMSK1 |= (1 << OCIE1A);
    TCCR1B = (1 << WGM12) | (1 << CS11);  // Start Timer1 with prescaler 8

    ignore_igt_until = lastDwellRequestMicros + currentDwellUs + 300;

  } else {
    // Find a way to compensate for the miss here - spoils the whole flow if one is missed.
    igtcountmiss++;
    syncState = WAITING_FOR_EDGE;

    PORTC &= ~((1 << PC7) | (1 << PC5) | (1 << PC3));
    PORTD &= ~(1 << PD7);

    TCCR1B = 0;
    TCNT1 = 0;
    g_coil_trigger_active = false;
  }
}


//COP end
static uint8_t lastPINK = 0;

ISR(PCINT2_vect) {

  uint8_t current = PINK;
  uint8_t changed = current ^ lastPINK;

  // ----- A10 (PK2) -----
  if (changed & (1 << 2)) {
    if (current & (1 << 2)) {
      // A10 is now HIGH (rising edge)
      onIGTRising();
    } else {
      // A10 is now LOW (falling edge)
      if (syncState != SYNCED) {
        // launch fake IGF because firing hasn't begun
        startTimer3_us(500);
      }
    }
  }

  lastPINK = current;
}

#define PIN15_BIT (1 << 0)  // PJ0
//attach the vss/buttons interrupt
ISR(PCINT1_vect) {
  static uint8_t lastPINJ = 0;
  uint8_t current = PINJ;

  uint8_t changed = current ^ lastPINJ;
  lastPINJ = current;

  if (changed & PIN15_BIT) {
    if (current & PIN15_BIT) {
      // pin 15 is now HIGH
      onCPSChange();
    } else {
      // pin 15 is now LOW
      onCPSChange();
    }
  }

} /* ISR(PCINT1_vect) */


void onCPSChange() {
  unsigned long now = microSeconds();

  // Debounce: ignore very close edges (noise)
  if (now - lastEdgeTime > 500) {  // 500 µs filter
    lastEdgeTime = now;
    edgeDetected = true;

    validateSync();
  }
}


char myBuffer[20];

volatile long fireTime = 0;

void stopTimer3() {
  // TCCR3B = 0;
  TCCR3B = (1 << WGM32);

  TIMSK3 &= ~(1 << OCIE3A);

  TIFR3 |= (1 << OCF3A);
  //TCCR3B &= ~0b111;  // remove prescaler -> stops timer immediately
}

void startTimer3_us(uint16_t duration_us) {
  // Force IGF low first (safety)
  PORTG &= ~(1 << PG5);

  // Stop timer
  // TCCR3B = 0;
  TCCR3B = (1 << WGM32);
  TIMSK3 &= ~(1 << OCIE3A);

  // Clear stale flag
  TIFR3 |= (1 << OCF3A);

  // Example: prescaler = 8 -> 0.5us per tick
  // For “X microseconds”, OCR3A = X * 2 - 1
  uint16_t ocr = (duration_us * 2) - 1;

  TCNT3 = 0;  // reset counter
  OCR3A = ocr;

  PORTG |= (1 << PG5);  // Pin 4 HIGH

  // Enable interrupt
  TIMSK3 |= (1 << OCIE3A);

  //TCCR3B &= ~0b111;       // clear prescaler bits

  TCCR3B |= (1 << CS31);  // prescaler = 8 (no freezing!)

  lastIGF = microSeconds();
}

ISR(TIMER3_COMPA_vect) {
  stopTimer3();  // important: turn it off immediately

  PORTG &= ~(1 << PG5);  // Pin 4 LOW
}

ISR(TIMER2_OVF_vect) {
  timer2_overflow_count++;
}

ISR(TIMER1_COMPA_vect) {

  TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));

  // Disable compare interrupt
  TIMSK1 &= ~(1 << OCIE1A);

  // Clear compare flag
  TIFR1 |= (1 << OCF1A);

  if (!g_coil_trigger_active) {
    return;
  }

  if (nextCoilToFire == COIL_4_PIN) {
    PORTC &= ~(1 << PC3);         // Pin 34 LOW
    nextCoilToFire = COIL_2_PIN;  // Next is cylinder 2
  } else if (nextCoilToFire == COIL_3_PIN) {
    PORTC &= ~(1 << PC5);         // Pin 32 LOW
    nextCoilToFire = COIL_4_PIN;  // Next is cylinder 4
  } else if (nextCoilToFire == COIL_2_PIN) {
    PORTD &= ~(1 << PD7);         // Pin 38 LOW
    nextCoilToFire = COIL_1_PIN;  // Next is cylinder 1
  } else if (nextCoilToFire == COIL_1_PIN) {
    PORTC &= ~(1 << PC7);         // Pin 30 LOW
    nextCoilToFire = COIL_3_PIN;  // Next is cylinder 3
  }

  g_coil_trigger_active = false;  // Mark trigger as inactive

  startTimer3_us(500);
}

String getResetCause() {
  uint8_t mcusr_val = MCUSR;  // Read MCUSR as early as possible
  MCUSR = 0;                  // Clear MCUSR for the next reset detection

  if (mcusr_val & (1 << WDRF)) {
    return "Watchdog Reset";
  }
  if (mcusr_val & (1 << BORF)) {
    return "Brown-out Reset";
  }
  if (mcusr_val & (1 << EXTRF)) {
    return "External Reset";
  }
  if (mcusr_val & (1 << PORF)) {
    return "Power-on Reset";
  }
  return "Unknown Reset";  // Should ideally not happen if flags are caught
}

void setupTimer3() {

  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;

  // CTC mode
  TCCR3B |= (1 << WGM32);

  // DO NOT enable interrupt here
  TIMSK3 &= ~(1 << OCIE3A);

  // Clear stale flag
  TIFR3 |= (1 << OCF3A);
}

int startup = 0;

void setup(void) {
  wdt_disable();

  pinMode(15, INPUT_PULLUP);
  pinMode(7, OUTPUT);

  pinMode(SYNC_STATE_IND, OUTPUT);
  // --- A10 (PK2) and A11 (PK3) as inputs ---
  DDRK &= ~((1 << 2) | (1 << 3));  // inputs
  PORTK |= (1 << 3);               // optional pull-ups

  // --- Enable PCINT on PORTK ---
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);  // A10 + A11

  // --- A9 (PK1) as input with optional pull-up ---
  DDRK &= ~(1 << 1);  // input

  // --- Enable PCINT on PORTK ---
  PCICR |= (1 << PCIE2);     // enable PCINT for PORTK
  PCMSK2 |= (1 << PCINT17);  // enable PCINT for A9

  // A8 = PK0 = PCINT16
  DDRK &= ~(1 << 0);  // input

  PCMSK2 |= (1 << PCINT16);

  PCICR |= (1 << PCIE1);    // enable PCINT for PORTJ
  PCMSK1 |= (1 << PCINT9);  // enable PCINT on pin 15 (PJ0)


  String resetCauseString = getResetCause();  // Get the String object
  const char *resetCauseChar = resetCauseString.c_str();

  simpletx("Last Reset Cause: ");
  simpletx(resetCauseChar);
  simpletx("\n\n\n");

  pinMode(COIL_4_PIN, OUTPUT);
  digitalWrite(COIL_4_PIN, LOW);

  pinMode(COIL_1_PIN, OUTPUT);
  digitalWrite(COIL_1_PIN, LOW);

  pinMode(COIL_2_PIN, OUTPUT);
  digitalWrite(COIL_2_PIN, LOW);

  pinMode(COIL_3_PIN, OUTPUT);
  digitalWrite(COIL_3_PIN, LOW);

  pinMode(IGF_PIN, OUTPUT);

  pinMode(INDICATOR, OUTPUT);
  digitalWrite(INDICATOR, LOW);

  digitalWrite(IGF_PIN, LOW);

  init2();

  PCMSK1 |= (1 << (PCINT11 - 8)) | (1 << (PCINT12 - 8));

  // Software interrupt for dwell time
  // --- Configure Timer1 for Compare Match A interrupt ---
  // 1. Stop Timer1 (clear prescaler bits)
  TCCR1A = 0;  // Clear CS32, CS31, CS30 bits

  // // 2. Set Timer1 to CTC (Clear Timer on Compare Match) mode
  TCCR1B = (1 << WGM12) | (1 << CS11);

  // // 3. Set the OCR1A value for 300us (600 ticks for 0.5us/tick)
  OCR1A = 4499;  // (599 + 1) * 0.5us = 600 * 0.5us = 300 us 999==500us, 1499==750us, 1ms=1999, 1.25ms = 2499, 2.0ms = 3999, 2.5ms = 4999

  // // 4. Enable Timer1 Compare Match A Interrupt
  TIMSK1 |= (1 << OCIE1A);  // Enable interrupt for Timer1 Compare Match A

  setupTimer3();

  while(startup < 15){
      digitalWrite(7, HIGH);
      delay2(100);
      digitalWrite(7, LOW);
      delay2(100);
      startup++;
  }

  wdt_enable(WDTO_250MS);

} /* void setup (void) */

volatile bool syncIsLit = false;

volatile unsigned long lastBatCheck = 0;

volatile unsigned long lastHeartBeat = 0;

#define CFG_SERIAL_TX 1

void loop(void) {

  if (elapsedMicroseconds(lastHeartBeat) > 500000) {

    syncIsLit = !syncIsLit;

    if (syncState == SYNCED && syncIsLit) {
      digitalWrite(SYNC_STATE_IND, HIGH);
    } else {
      digitalWrite(SYNC_STATE_IND, LOW);
    }

    if (syncIsLit) {
      digitalWrite(7, HIGH);
    } else {
      digitalWrite(7, LOW);
    }

#if (CFG_SERIAL_TX == 1)

    simpletx(" volRef: ");
    simpletx(format(vbatt_filtered * 1000.0f));
    simpletx(", ");

    simpletx("SYNC: ");
    simpletx(inttochar(synccount, myBuffer, sizeof(myBuffer)));
    simpletx(", ");

    simpletx("igtWhileUnsynced: ");
    simpletx(inttochar(igtWhileUnsynced, myBuffer, sizeof(myBuffer)));
    simpletx(", ");

    simpletx("Total: ");
    simpletx(inttochar(allgtcount, myBuffer, sizeof(myBuffer)));
    simpletx("  ");
    simpletx("Handled: ");
    simpletx(inttochar(gtcount, myBuffer, sizeof(myBuffer)));
    simpletx("  ");
    simpletx("Debounced: ");
    simpletx(inttochar(igtCountRejected, myBuffer, sizeof(myBuffer)));
    simpletx("  ");
    simpletx("DeadTime: ");
    simpletx(inttochar(igtCountDeadTime, myBuffer, sizeof(myBuffer)));
    simpletx("  ");
    simpletx("StillF: ");
    simpletx(inttochar(igtcountmiss, myBuffer, sizeof(myBuffer)));
    simpletx("\n");

#endif

    lastHeartBeat = microSeconds();
  }

  if (PORTG & (1 << PG5)) {
    if (elapsedMicroseconds(lastIGF) > 20000) {  // >2ms HIGH
      PORTG &= ~(1 << PG5);                      // force low
      stopTimer3();
    }
  }

  if (elapsedMicroseconds(lastBatCheck) > 10000) {
    updateBatteryFiltered();

    cli();
    float dwell = lookupDwellUs(vbatt_filtered);  // microseconds
    currentDwellUs = dwell;
    sei();

    lastBatCheck = microSeconds();
  }

  if (syncState != WAITING_FOR_EDGE) {
    if ((elapsedMicroseconds(lastEdgeTime) > 500000) && (elapsedMicroseconds(last_igt_time) > 1000000)) {  // 0.5 sec && 1 sec
      cli();
      syncState = WAITING_FOR_EDGE;
      validEdgeCount = 0;
      igtcountmiss = 0;
      synccount = 0;
      igtCountRejected = 0;
      igtWhileUnsynced = 0;
      igtCountDeadTime = 0;
      g_coil_trigger_active = false;
      TCCR1B = 0;
      TCNT1 = 0;
      PORTC &= ~((1 << PC7) | (1 << PC5) | (1 << PC3));
      PORTD &= ~(1 << PD7);
      sei();
    }
  }

  if (syncState == WAITING_FOR_EDGE) {
    if (elapsedMicroseconds(lastEdgeTime) > 60000000) {
      simpletx("Entering sleep...\n");
      system_sleep();  //system PowerDown mode to save power
    }
  }
  
  wdt_reset();
} /* loop (void) */


//--------------------------------------------------------
char *format(unsigned long num) {
  static char fBuff[7];  //used by format
  unsigned char dp = 3;
  unsigned char x = 6;

  while (num > 999999) {
    num /= 10;
    dp++;
    if (dp == 5) break; /* We'll lose the top numbers like an odometer */
  }
  if (dp == 5) {
    dp = 99;
  } /* We don't need a decimal point here. */

  /* Round off the non-printed value. */
  if ((num % 10) > 4) {
    num += 10;
  }

  num /= 10;


  while (x > 0) {
    x--;
    if (x == dp) {
      /* time to poke in the decimal point? */
      fBuff[x] = '.';
    } else {
      /* poke the ascii character for the digit. */
      fBuff[x] = '0' + (num % 10);
      num /= 10;
    }
  }

  if (fBuff[0] == '0') {  //if leftmost char is 0 then replace it with space: '099.56' => ' 99.56'
    fBuff[0] = ' ';
    if (fBuff[1] == '0') fBuff[1] = ' ';
  }

  fBuff[6] = 0;
  return fBuff;
}


char *inttochar(int number, char *buffer, size_t bufferSize) {
  // Use snprintf for safety to prevent buffer overflows
  snprintf(buffer, bufferSize, "%d", number);
  return buffer;  // Return the pointer to the buffer passed in
}

// Call regularly (e.g., every 5-10 ms)
void updateBatteryFiltered() {
  float v = batteryVoltage() / 1000.0f;
  // simple first-order IIR
  vbatt_filtered = vbatt_filtered + VBATT_ALPHA * (v - vbatt_filtered);
}

unsigned long lookupDwellUs(float vbat) {
  // Clamp input
  if (vbat <= dwell_v[0]) return DWELL_MAX_US;
  if (vbat >= dwell_v[TABLE_LEN - 1]) return DWELL_MIN_US;

  // find bracket
  uint8_t i = 0;
  while (i < TABLE_LEN - 1 && vbat > dwell_v[i + 1]) i++;

  // linear interpolate between i and i+1
  float v0 = dwell_v[i];
  float v1 = dwell_v[i + 1];
  unsigned long d0 = dwell_us[i];
  unsigned long d1 = dwell_us[i + 1];

  float t = (vbat - v0) / (v1 - v0);  // 0..1
  float dwe = d0 + t * (d1 - d0);
  unsigned long d = (unsigned long)(dwe + 0.5f);

  // clamp just in case
  if (d < DWELL_MIN_US) d = DWELL_MIN_US;
  if (d > DWELL_MAX_US) d = DWELL_MAX_US;
  return d;
}

unsigned long batteryVoltage(void) {
  float vout = 0.0;
  float vin = 0.0;
  float R1 = 27300.0;  // resistance of R1 - 21500
  float R2 = 10000.0;  // resistance of R2 - 9500
  int value = 0;

  value = analogRead(A3);

  vout = (value * 5.0) / 1024.0;
  vin = vout / (R2 / (R1 + R2));

  vin = vin * 1000 - 300 + 1000ul;
  return ((unsigned long)vin / 100) * 100;  //make last digit to be 0
}

void simpletx(char *string) {
  if (UCSR0B != (1 << TXEN0)) {  //do we need to init the uart?
    UBRR0H = (unsigned char)(myubbr >> 8);
    UBRR0L = (unsigned char)myubbr;
    UCSR0B = (1 << TXEN0);   //Enable transmitter
    UCSR0C = (3 << UCSZ00);  //N81
  }
  while (*string) {
    while (!(UCSR0A & (1 << UDRE0)))
      ;
    UDR0 = *string++;  //send the data
  }
}

unsigned long millis2(){
	return timer2_overflow_count * 64UL * 2 / (16000000UL / 128000UL);
}

void delay2(unsigned long ms){
	unsigned long start = millis2();
	while (millis2() - start < ms);
}

void init2() {
  // this needs to be called before setup() or some functions won't
  // work there
  sei();

  // timer 0 is used for millis2() and delay2()
  timer2_overflow_count = 0;
  // on the ATmega168, timer 0 is also used for fast hardware pwm
  // (using phase-correct PWM would mean that timer 0 overflowed half as often
  // resulting in different millis2() behavior on the ATmega8 and ATmega168)
  TCCR2A = 1 << WGM20 | 1 << WGM21;
  // set timer 2 prescale factor to 64
  TCCR2B = 1 << CS22;

  // enable timer 2 overflow interrupt
  TIMSK2 |= 1 << TOIE2;
  // disable timer 0 overflow interrupt
  TIMSK0 &= ~(1 << TOIE0);
}

void system_sleep(void) {

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  cli();

  // Clear any pending pin change interrupts
  PCIFR |= (1 << PCIF1) | (1 << PCIF2);

  sleep_enable();
  ADCSRA &= ~(1 << ADEN);  // Optional power saving

  sei();
  sleep_cpu();  // <<< CPU sleeps here

  // ---- WOKE UP ----
  sleep_disable();
  ADCSRA |= (1 << ADEN);
}

