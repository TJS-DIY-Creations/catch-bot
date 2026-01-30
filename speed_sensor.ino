// -------- CONFIG --------
const byte sens1 = 2;   // MUST be interrupt-capable
const byte sens2 = 3;   // MUST be interrupt-capable

const float distanceMeters = 0.50; // distance between sensors (meters)
const unsigned long debounceTime = 5; // ms

// -------- STATE --------
volatile bool sens1Triggered = false;
volatile bool sens2Triggered = false;

volatile unsigned long startMillis = 0;
volatile unsigned long stopMillis = 0;

volatile unsigned long lastSens1Time = 0;
volatile unsigned long lastSens2Time = 0;

bool resultPrinted = false;

// -------- SETUP --------
void setup() {
  Serial.begin(9600);

  pinMode(sens1, INPUT);
  pinMode(sens2, INPUT);

  attachInterrupt(digitalPinToInterrupt(sens1), sens1ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(sens2), sens2ISR, RISING);

  Serial.println("System armed.");
}

// -------- LOOP --------
void loop() {
  // Print result once both sensors triggered
  if (sens1Triggered && sens2Triggered && !resultPrinted) {
    noInterrupts();
    unsigned long travelTime = stopMillis - startMillis;
    interrupts();

    float timeSeconds = travelTime / 1000.0;
    float speed = distanceMeters / timeSeconds;

    Serial.print("Travel time (ms): ");
    Serial.println(travelTime);

    Serial.print("Speed (m/s): ");
    Serial.println(speed, 3);

    Serial.print("Speed (km/h): ");
    Serial.println(speed * 3.6, 2);

    resultPrinted = true;

    delay(500); // small pause before re-arming
    resetSystem();
  }
}

// -------- INTERRUPTS --------
void sens1ISR() {
  unsigned long now = millis();
  if (now - lastSens1Time < debounceTime) return;

  if (!sens1Triggered) {
    sens1Triggered = true;
    startMillis = now;
  }

  lastSens1Time = now;
}

void sens2ISR() {
  unsigned long now = millis();
  if (now - lastSens2Time < debounceTime) return;

  if (sens1Triggered && !sens2Triggered) {
    sens2Triggered = true;
    stopMillis = now;
  }

  lastSens2Time = now;
}

// -------- RESET --------
void resetSystem() {
  noInterrupts();
  sens1Triggered = false;
  sens2Triggered = false;
  startMillis = 0;
  stopMillis = 0;
  lastSens1Time = 0;
  lastSens2Time = 0;
  interrupts();

  resultPrinted = false;
  Serial.println("System re-armed.");
}
