#include <AccelStepper.h>

// Function prototypes
void startFans();
void stopFans();
void startPump();
void stopPump();
void startSpray();
void stopSpray();
void startRotation();
void stopRotation();
void startWhisk();
void stopWhisk();
void startIdle();
void stopIdle();

// Port definitions
#define WHISK_PORT 0
#define PUMP_PORT 1
#define SPRAY_PORT 2
#define FANS_PORT 3

typedef void (*EventNameFunc)();

// Event structure
struct Event {
  unsigned long startSeconds;
  EventNameFunc startFunc;
  unsigned long stopSeconds;
  EventNameFunc stopFunc;
  bool isActive;
};

#define NUM_EVENTS 4 // Number of timeline events <---------------------

// Array of events
Event events[NUM_EVENTS] = {
    // {0,  startWhisk,    0,      stopWhisk, false},
    {3, startPump, 15, stopPump, false},
    {4, startRotation, 50, stopRotation, false},
    {4, startSpray, 15, stopSpray, false},
    {40, startIdle, 5 * 60 * 60, stopIdle,
     false} // 5 hours * 60 minutes * 60 seconds
};

unsigned long cycleStartTime = 0; // Start time of the current cycle (relative)

const int latchPin = 10; // Pin LT on Relay board
const int dataPin = 11;  // Pin DS on Relay board
const int clockPin = 13; // Pin CL on Relay board

const int stepperPulPin = 5;
const int stepperDirPin = 4;

const int button = 3;

AccelStepper stepper(AccelStepper::DRIVER, stepperPulPin, stepperDirPin);

int portState = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Timeline started!");

  pinMode(latchPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(button, INPUT_PULLUP);

  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(200);

  stopAll();
  digitalWrite(13, true);
  delay(1000);
  digitalWrite(13, false);
}

void loop() {
  unsigned long currentTime =
      millis() / 1000; // Get the current time in milliseconds

  // Calculate the relative time in the current cycle
  unsigned long relativeTime = (currentTime - cycleStartTime);

  // Check each event for start and stop conditions
  for (int i = 0; i < NUM_EVENTS; i++) {
    // Check if event should start
    if (!events[i].isActive && relativeTime >= events[i].startSeconds &&
        relativeTime < events[i].stopSeconds) {
      events[i].isActive = true; // Mark event as active
      events[i].startFunc();     // Trigger start event
    }

    // Check if event should stop
    if (events[i].isActive && relativeTime >= events[i].stopSeconds) {
      events[i].isActive = false; // Mark event as inactive
      events[i].stopFunc();       // Trigger stop event
    }
  }

  // If the cycle has ended, restart the timeline
  unsigned long timelineEnd = events[NUM_EVENTS - 1].stopSeconds;
  if (relativeTime >= timelineEnd) {
    Serial.println("Restarting timeline...");
    cycleStartTime = currentTime; // Reset the cycle start time
    // Reset all event states
    for (int i = 0; i < NUM_EVENTS; i++) {
      events[i].isActive = false;
    }
    relativeTime = 0; // Reset the relative time
  }

  stepper.run();
}

void startFans() { enablePort(FANS_PORT); }
void stopFans() { disablePort(FANS_PORT); }

void startPump() { enablePort(PUMP_PORT); }
void stopPump() { disablePort(PUMP_PORT); }

void startSpray() { enablePort(SPRAY_PORT); }
void stopSpray() { disablePort(SPRAY_PORT); }

void startRotation() { stepper.moveTo(8000); }
void stopRotation() { stepper.setCurrentPosition(0); }

void startWhisk() { enablePort(WHISK_PORT); }
void stopWhisk() { disablePort(WHISK_PORT); }

void startIdle() { stopAll(); }
void stopIdle() {}

void stopAll() {
  portState = 0;
  updatePorts();
  stepper.stop();
}

void enablePort(int port) {
  bitWrite(portState, port, 1);
  updatePorts();
}

void disablePort(int port) {
  bitWrite(portState, port, 0);
  updatePorts();
}

void updatePorts() {
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, portState);
  digitalWrite(latchPin, HIGH);
}
