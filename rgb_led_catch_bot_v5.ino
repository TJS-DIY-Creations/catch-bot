#include <AccelStepper.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ============================================================
// OLED
// ============================================================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// define rgb led pins
int redPin = 11; 
int greenPin = 10;
int bluePin = 9;

struct Color {
  int r;
  int g;
  int b;
};

// Bigger color list
Color colors[] = {
  { 255, 0, 0 },   // Red
  { 255, 40, 0 },  // Red-Orange
  { 255, 80, 0 },
  { 255, 120, 0 },
  { 255, 165, 0 },  // Orange
  { 255, 200, 0 },
  { 255, 255, 0 },  // Yellow
  { 180, 255, 0 },
  { 120, 255, 0 },
  { 0, 255, 0 },  // Green
  { 0, 255, 120 },
  { 0, 255, 200 },
  { 0, 255, 255 },  // Cyan
  { 0, 120, 255 },
  { 0, 0, 255 },    // Blue
  { 75, 0, 130 },   // Indigo
  { 128, 0, 128 },  // Purple
  { 255, 0, 255 },  // Magenta
  { 255, 0, 120 },
  { 255, 0, 60 },
  { 255, 255, 255 },  // White
  { 255, 180, 120 },  // Warm white
  { 180, 200, 255 }   // Cool white
};

const int colorCount = sizeof(colors) / sizeof(colors[0]);

// Timing
unsigned long previousMillis = 0;
const long fadeInterval = 10;
const int fadeSteps = 100;

// State
int currentIndex = 0;
int nextIndex = 1;
int stepCount = 0;

float version = 1.1;

const byte sens1 = 2;//sensor pins
const byte sens2 = 3;

const float distanceMeters = 0.02815;  //distance between sensors, in cad 0.02815
const unsigned long debounceUS = 50; // debounce to prevent false trigger

//variables
const int stepsPerCM = 50; // steps to move 1 cm
float h = 0.825; //hight of desk
float g = 9.81; // gravity

// A4988 pins
#define STEP_PIN 4
#define DIR_PIN 5
#define ENABLE_PIN 6

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN); //defines stepper driver pins

// ISR RESULTS
volatile unsigned long tStart = 0;
volatile unsigned long tStop = 0;
volatile bool measurementReady = false;

// ============================================================
// CALCULATED VALUES
// ============================================================
float speed_mps = 0.0;
float speed_fps = 0.0;
long targetSteps = 0;

// ============================================================
// STATE MACHINE
// ============================================================
enum SystemState {
  ARMED,
  CALCULATING,
  MOVING,
  RETURNING,
  WAITING_FOR_CLEAR
};

SystemState currentState = ARMED;

unsigned long moveCompleteMillis = 0;
const unsigned long returnDelay = 2000;

// INTERRUPTS
void sens1ISR() {
  static unsigned long last = 0;
  unsigned long now = micros();

  if (now - last > debounceUS) {
    tStart = now;
  }
  last = now;
}

void sens2ISR() {
  static unsigned long last = 0;
  unsigned long now = micros();

  if (now - last > debounceUS && tStart != 0) {
    tStop = now;
    measurementReady = true;
  }
  last = now;
}


// SETUP

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    for (;;)
      ;
  }

  pinMode(sens1, INPUT);
  pinMode(sens2, INPUT);

  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  stepper.setMaxSpeed(250000);
  stepper.setAcceleration(250000);
  stepper.setCurrentPosition(0);

  attachInterrupt(digitalPinToInterrupt(sens1), sens1ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(sens2), sens2ISR, RISING);

  drawDisplay();  // initial screen
}

void loop() {
  stepper.run();

  switch (currentState) {

    case ARMED:
      if (measurementReady) currentState = CALCULATING;
      break;

    case CALCULATING:
      {
        noInterrupts();
        unsigned long start = tStart;
        unsigned long stop = tStop;
        tStart = 0;
        measurementReady = false;
        interrupts();

        unsigned long dt = stop - start;
        if (dt == 0) dt = 1;

        speed_mps = distanceMeters / (dt / 1000000.0);
        speed_fps = speed_mps * 3.28084;

        float X = speed_mps * sqrt((2.0 * h) / g);
        float X_cm = X * 100.0;
        targetSteps = (long)(X_cm * stepsPerCM);

        if (targetSteps > 0) {
          stepper.moveTo(targetSteps);
          currentState = MOVING;
        } else {
          currentState = WAITING_FOR_CLEAR;
        }

        drawDisplay();
        break;
      }

    case MOVING:
      if (stepper.distanceToGo() == 0) {
        moveCompleteMillis = millis();
        currentState = RETURNING;
      }
      break;

    case RETURNING:
      if (millis() - moveCompleteMillis >= returnDelay) {
        stepper.moveTo(0);
        currentState = WAITING_FOR_CLEAR;
      }
      break;

    case WAITING_FOR_CLEAR:
      if (stepper.distanceToGo() == 0 && digitalRead(sens1) == LOW && digitalRead(sens2) == LOW) {

        delay(300);
        currentState = ARMED;
        drawDisplay();
      }
      break;
  }
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= fadeInterval) { 
    previousMillis = currentMillis;

    float t = (float)stepCount / fadeSteps;

    // changes the color of the rgb leds
    int r = colors[currentIndex].r + (colors[nextIndex].r - colors[currentIndex].r) * t;
    int g = colors[currentIndex].g + (colors[nextIndex].g - colors[currentIndex].g) * t;
    int b = colors[currentIndex].b + (colors[nextIndex].b - colors[currentIndex].b) * t;

    setColor(r, g, b);

    stepCount++;

    if (stepCount > fadeSteps) {
      stepCount = 0;
      currentIndex = nextIndex;
      nextIndex = (nextIndex + 1) % colorCount;
    }
  }
}

void drawDisplay() { //updates the display
  display.clearDisplay(); // clears display

  display.setTextColor(WHITE); // sets text color but doesnt really work due to display hardware limitations

  // version
  display.setTextSize(1); // change text size
  display.setCursor(0, 0); // move cursor
  display.print("v "); // prints v for "version"
  display.println(version, 1); // prints code version next to v

  // speeds
  display.setTextSize(2); // sets to bigger text size
  display.setCursor(0, 16); // moves cursor
  display.print(speed_mps, 1); // then prints the m/s next to it 
  display.print("m/s"); // prints m/s units

  display.setCursor(0, 36); // moves cursor again
  display.print(speed_fps, 0); // prints speed in foot per second, why did i add this??????
  display.print("fps"); // prints units

  // steps
  display.setTextSize(1); // changes text size back
  display.setCursor(80, 0); //moves cursor
  display.print("Steps:"); // label
  display.setCursor(80, 10); // moves cursor again
  display.print(targetSteps); // prints steps that the motor moved 

  // state
  display.setCursor(80, 25); //moves cursor again
  display.print("State:"); // prints label
  display.setCursor(80, 35); // moves cursor 
  display.print(currentState); // prints state

  display.display(); // send all above to display 
}
void setColor(int redValue, int greenValue, int blueValue) { // rgb code
  analogWrite(redPin, redValue);
  analogWrite(greenPin, greenValue);
  analogWrite(bluePin, blueValue);
}
