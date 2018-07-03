#include <eRCaGuy_Timer2_Counter.h>
#include "state.h"

// MARK: Properties

// Set our initial state to PreSync
// Allows us to wait for a valid sync pulse
State phase = PreSync;

// initialize our timing variables
volatile float riseTime = 0;
volatile float fallTime = 0;

float syncStart = 0; // set on rising edge of sync pulse
float syncEnd = 0; // set on falling edge of sync pulse

float sweepRise = 0; // set on rising edge of sweep pulse
float sweepFall = 0; // set on falling edge of sweep pulse

// set constants
const int MAX_SYNC = 138; // upper bound on sync pulse duration
const int MIN_SYNC = 50; // lower bound on sync pulse duration
const float pi = 3.141593;
const int SYNC_OFFSET = 10; // microsecond offset
const int RADIAL_OFFSET = 0; // microsecond offset

const float SWEEP_TIME = 16.6666666667 * 1000; // time (us) it takes for horizontal/vertical sweeps to complete (FIXME)
const float SWEEP_VEL = pi / SWEEP_TIME; // velocity (rad/us) of sweeps (FIXME)
const float DIODE_WIDTH_MM = 4.5; // the width (mm) of the photodiode
const float DIODE_WIDTH = DIODE_WIDTH_MM / 10; // the width (cm) of the photodiode

// initialize position variables
float theta = -1;
float phi = -1;
float radial_h = -1;
float radial_v = -1;

unsigned int syncBits = 0;

const byte interruptPin_a = 2;
const byte interruptPin_b = 3;
const byte outputPin = 13;

bool interrupted = false;
bool enabled = true;
volatile unsigned int interruptID = 0;
unsigned int expectedID = 1;

// initialize our state arrays (solely for proof of concept)
const int STORE = 3;
float times[STORE];
bool init_rising = false;

// MARK: Default Methods (setup and loop)

void setup() {
  // pinMode(outputPin, OUTPUT);
  pinMode(interruptPin_a, INPUT);
  pinMode(interruptPin_b, INPUT);
  
  // timer2.setup();
  Serial.begin(115200);
  
  memset(times,0,STORE);
  
  attachInterrupt(digitalPinToInterrupt(interruptPin_a), rising, RISING);
  attachInterrupt(digitalPinToInterrupt(interruptPin_b), falling, FALLING);
}

void loop() {
  if (interrupted && enabled) {
    float rise = riseTime; float fall = fallTime;
    interrupted = false;
    if (interruptID == expectedID) {
      if (rise > fall) {
        times[interruptID] = rise;
        if (interruptID == 0) {
          init_rising = true;
        }
      } else {
        times[interruptID] = fall;
      }
      expectedID++;
      
      if (expectedID >= STORE) {
        enabled = false;
        // noInterrupts();
      }
    } else {
      interruptID = 0;
      expectedID = 1;
      init_rising = false;
    }
  } else if (!enabled) {
    Serial.println(times[1] - times[0]);
    Serial.println(times[2] - times[1]);
    
    /*
    phase = PreSync;
    float rise; float fall; int i; int off = 0;
    if (init_rising) {
      rise = getTime(0); updateStateRising(rise);
      fall = getTime(1); updateStateFalling(fall - rise);
      i = 2;
    } else {
      rise = getTime(1); updateStateRising(rise);
      fall = getTime(2); updateStateFalling(fall - rise);
      i = 3; off = 1;
    }
    for (; i < STORE; i++) {
      t = times[i];
      if ((i+off) % 2 == 0) {
        rise = t;
        updateStateRising(rise - fall);
      } else {
        fall = t;
        updateStateFalling(fall - rise);
      }
      
      updatePhi(false);
      updateTheta(false);
      updateRadial(false);
    } */
      
    enabled = true;
    // interrupts();
  }
}

// MARK: Logic

void updateStateRising(float diff) {
  switch (phase) {
    case PreSync:
      Serial.println("Rising edge detected! Setting SYNC state.");
      phase = Sync;
      break;
    case PreHoriz: // Vertical hit
      Serial.println("Rising edge detected! Setting PREHORIZ state.");
      // Horizontal hit
      if (-diff > SWEEP_TIME) {
        phase = PreSync;
        break;
      }
      phi = calculateAngle(diff);
      phase = Horiz;
      break;
    case PreVert: // Vertical hit
      Serial.println("Rising edge detected! Setting PREVERT state.");
      if (diff > SWEEP_TIME) {
        phase = PreSync;
        break;
      }
      theta = calculateAngle(diff);
      phase = Vert;
      break;
    default:
      Serial.print(phase);
      Serial.println(": ERROR: Rising edge.");
      break;
  }
}

void updateStateFalling(float diff) {
  switch (phase) {
    case Sync:
      Serial.println("Falling edge detected! Checking sync pulse validity.");
      if (diff > MIN_SYNC && diff < MAX_SYNC) {
        Serial.println("Setting PRESWEEP state.");
        unsigned int pulse = syncPulse(diff);
        unsigned int sk = 0b100 & pulse;
        if (sk == 1) {
          phase = PreSync;
          // sweep skipped for cycle
          Serial.println("Skip bit enabled, skipping sweep.");
          break;
        }
        
        unsigned int axis = 0b1 & pulse;
        if (axis == 0) {
          phase = PreHoriz;
          // waiting for horizontal sweep
        } else if (axis == 1) {
          phase = PreVert;
          // waiting for vertical sweep
        }
      } else {
        phase = PreSync;
        // sync pulse length not within valid range
        Serial.println("Sync pulse length not within valid range.");
      }
      break;
    case Vert:
      Serial.println("Falling edge detected! Setting VERT state.");
      if (diff > MIN_SYNC) {
        phase = PreSync;
        break;
      }
      radial_v = calculateRadialDistance(diff);
      // vertical sweep happened
      phase = PreSync;
      break;
    case Horiz:
      Serial.println("Falling edge detected! Setting VERT state.");
      if (diff > MIN_SYNC) {
        phase = PreSync;
        break;
      }
      radial_h = calculateRadialDistance(diff);
      // horizontal sweep happened
      phase = PreSync;
      break;
    default:
      Serial.print(phase);
      Serial.println(": ERROR: Falling edge.");
      break;
  }
}

// MARK: Interrupts

/** Called when a rising edge appears. Handles lighthouse pulse logic. */
void rising() {
  if (enabled) {
    riseTime = microseconds();
    interrupted = true;
    interruptID++;
  }
}

/** Called when a falling edge appears. Handles lighthouse pulse logic. */
void falling() {
  if (enabled) {
    fallTime = microseconds();
    interrupted = true;
    interruptID++;
  }
}

// MARK: Math

/** Calculates the angle at which our receiver is at
  based on infrared laser sweep and pulse timing information.*/
float calculateAngle(float _diff) {
  return _diff * SWEEP_VEL;
}

/** Calculates the distance from the lighthouse our receiver is at
  based on infrared laser sweep and pulse timing information.*/
float calculateRadialDistance(float diff) {
  float vel = DIODE_WIDTH / (diff - RADIAL_OFFSET);
  return vel / SWEEP_VEL;
}

/** Returns a number defining our 3 information bits: skip, data, axis.
  Given by our pulse length in microseconds (us). */
int syncPulse(float duration) {
  return (unsigned int) (48*duration - 2501) / 500;
  // return (unsigned int) div_48_500 * duration - div_2501_500;
}

// MARK: Serial

void updatePhi(bool simple) {
  update("Phi", phi * 180/pi, "degrees", riseTime, simple);
}

void updateTheta(bool simple) {
  update("Theta", theta * 180/pi, "degrees", riseTime, simple);
}

void updateRadial(bool simple) {
  update("Distance (horiz)", radial_h, "meters", fallTime, simple);
  update("Distance (vert)", radial_v, "meters", fallTime, simple);
}

void update(String header, float value, String units, float time, bool simple) {
  if (!simple) {
    Serial.print(header);
    Serial.print(": ");
    Serial.print(value);
    Serial.print(" ");
    Serial.print(units);
    Serial.print(" at time ");
    Serial.println(time);
  } else {
    Serial.println(value);
  }
}

// MARK: OS

float microseconds() {
  // return micros();
  return timer2.get_micros();
}
