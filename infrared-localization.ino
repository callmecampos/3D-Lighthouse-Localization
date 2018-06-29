#include "state.h"

// Set our initial state to Waiting
// Allows us to wait for a sync pulse
State phase = Waiting;

// initialize our timing variables
float hitTime; // set on rising edge
float dropTime; // set on falling edge
float syncTime; // set on falling edge of sync pulse

float h_dist;
float v_dist;

// set constants
const int MAX_SYNC = 138; // upper bound on sync pulse duration
const int MIN_SYNC = 45; // lower bound on sync pulse duration
const float pi = 3.141592653589793;

const float SWEEP_TIME = 6.9; // time (s) it takes for horizontal/vertical sweeps to complete (FIXME)
const float SWEEP_VEL = pi / SWEEP_TIME; // velocity (cm/s) of sweeps (FIXME)
const float DIODE_WIDTH = 4.20; // the width (cm) of the photodiode (FIXME)

int cycleID = 0; // filler

// initialize position arrays
const int MEM = 200;
float thetas[MEM];
int th = 0;
float phis[MEM];
int ph = 0;
float radials[2*MEM];
int ra = 0;

const byte interruptPin_a = 2;
const byte interruptPin_b = 3;
const byte outputPin = 13;

void setup() {
  pinMode(outputPin, OUTPUT);
  pinMode(interruptPin_a, INPUT);
  pinMode(interruptPin_b, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(interruptPin_a), start, RISING);
  attachInterrupt(digitalPinToInterrupt(interruptPin_b), finish, FALLING);
}

void loop() {
  // do nothing
}

void setHigh() {
  // set output pin to HIGH
  digitalWrite(outputPin, HIGH);
}

void setLow() {
  // set output pin to LOW
  digitalWrite(outputPin, LOW);
}

/** Called when a rising edge appears. Handles lighthouse pulse logic. */
void start() {
  hitTime = microseconds();
  setHigh(); // TEMP
  
  float phi;
  float theta;
  
  float _diff = hitTime - dropTime;
  
  switch (phase) {
    case PreHoriz:
      // Horizontal hit
      phi = calculateAngle(_diff);
      updatePhi(phi);
    case PreVert:
      // Vertical hit
      theta = calculateAngle(_diff);
      updateTheta(theta);
    default:
      break;
  }
}

/** Called when a falling edge appears. Handles lighthouse pulse logic. */
void finish() {
  dropTime = microseconds();
  setLow(); // TEMP
  float diff = dropTime - hitTime;
  switch (phase) {
    case Sync || Hit:
      if (diff > MIN_SYNC && diff < MAX_SYNC) {
        int pulse = syncPulse(diff);
        int sk = skip(pulse);
        if (sk == 1) {
          phase = PreSync;
          // sweep skipped for cycle
          // TODO: MAYBE WRITE TO SERIAL PORT HERE? IDK
          cycleID++;
          break;
        }
        
        int ax = axis(pulse);
        if (ax == 0) {
          phase = PreHoriz;
          // waiting for horizontal sweep
        } else if (ax == 1) {
          phase = PreVert;
          // waiting for vertical sweep
        }
        syncTime = dropTime;
      } else {
        phase = Waiting;
        // sync pulse length not within valid range
      }
    case Horiz:
      h_dist = calculateRadialDistance(diff);
      updateRadial((h_dist + v_dist) / 2.0);
      // horizontal sweep happened
      cycleID++;
    case Vert:
      v_dist = calculateRadialDistance(diff);
      updateRadial((h_dist + v_dist) / 2.0);
      // vertical sweep happened
      cycleID++;
    default:
      break;
  }
}

/** Calculates the angle at which our receiver is at
  based on infrared laser sweep and pulse timing information.*/
float calculateAngle(float _diff) {
  return _diff * SWEEP_VEL;
}

/** Calculates the distance from the lighthouse our receiver is at
  based on infrared laser sweep and pulse timing information.*/
float calculateRadialDistance(float diff) {
  float vel = DIODE_WIDTH / diff;
  return vel / SWEEP_VEL;
}

/** Returns a number defining our 3 information bits: skip, data, axis.
  Given by our pulse length in microseconds (us). */
int syncPulse(float duration) {
  return (int) (48*duration - 2501) / 500;
}

/** Returns the axis bit based on pulse length. */
bool axis(int pulse) {
  return 0b1 & pulse;
}

/** Returns the axis bit based on pulse length. */
bool data(int pulse) {
  return 0b10 & pulse;
}

/** Returns the axis bit based on pulse length. */
bool skip(int pulse) {
  return 0b100 & pulse;
}

void updatePhi(float phi) {
  phis[ph] = phi; ph++;
  if (ph > MEM) {
    // write to serial
  }
}

void updateTheta(float theta) {
  thetas[th] = theta; th++;
  if (th > MEM) {
    // write to serial
  }
}

void updateRadial(float radial) {
  radials[ra] = radial; ra++;
  if (ra > MEM) {
    // write to serial
  }
}

float microseconds() {
  return micros(); // FIXME: this has 4 us resolution, we need 0.5 us resolution
}
