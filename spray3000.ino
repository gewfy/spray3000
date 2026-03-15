#include <AccelStepper.h>

// Function prototypes
void startFans();
void stopFans();
void startPump();
void stopPump();
void startAlert();
void stopAlert();
void startSpray1();
void stopSpray1();
void startSpray2();
void stopSpray2();
void startSpray3();
void stopSpray3();
void startSpray4();
void stopSpray4();
void startRotation();
void stopRotation();
void startWhisk();
void stopWhisk();
void startIdle();
void stopIdle();

// Port definitions
#define WHISK_PORT 0
#define ALERT_PORT 1
#define PUMP_PORT 2
#define SPRAY1_PORT 3
#define SPRAY2_PORT 4
#define FANS_PORT 5
#define SPRAY3_PORT 6
#define SPRAY4_PORT 7

typedef void (*EventNameFunc)();

// Event structure
struct Event {
  unsigned long startSeconds;
  EventNameFunc startFunc;
  unsigned long stopSeconds;
  EventNameFunc stopFunc;
  bool isActive;
};

#define NUM_EVENTS 20 // Number of timeline events <---------------------

// Array of events
Event events[NUM_EVENTS] = {
    {0, startAlert, 20, stopAlert, false},
    {5, startPump, 5 + 12, stopPump, false},
    {5, startSpray1, 5 + 12, stopSpray1, false},
    {5, startRotation, 5 + 15, stopRotation, false},

    {20, startIdle, 20 + 120, stopIdle, false},

    {135, startAlert, 140 + 15, stopAlert, false},
    {140, startPump, 140 + 12, stopPump, false},
    {140, startSpray2, 140 + 12, stopSpray2, false},
    {140, startRotation, 140 + 15, stopRotation, false},

    {155, startIdle, 155 + 120, stopIdle, false},

    {270, startAlert, 270 + 15, stopAlert, false},
    {275, startPump, 275 + 12, stopPump, false},
    {275, startSpray3, 275 + 12, stopSpray3, false},
    {275, startRotation, 275 + 15, stopRotation, false},

    {290, startIdle, 290 + 120, stopIdle, false},

    {405, startAlert, 410 + 15, stopAlert, false},
    {410, startPump, 410 + 12, stopPump, false},
    {410, startSpray4, 410 + 12, stopSpray4, false},
    {410, startRotation, 410 + 15, stopRotation, false},

    {425, startIdle, 5 * 60 * 60, stopIdle, false},

    /*

        {2, startSpray2, 3, stopSpray2, false},
        {3, startSpray3, 4, stopSpray3, false},
        {4, startSpray4, 5, stopSpray4, false},
        {6, startSpray1, 7, stopSpray1, false},
        {6, startSpray2, 7, stopSpray2, false},
        {6, startSpray3, 7, stopSpray3, false},
        {6, startSpray4, 7, stopSpray4, false},
        */
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

void startAlert() { enablePort(ALERT_PORT); }
void stopAlert() { disablePort(ALERT_PORT); }

void startSpray1() { enablePort(SPRAY1_PORT); }
void stopSpray1() { disablePort(SPRAY1_PORT); }

void startSpray2() { enablePort(SPRAY2_PORT); }
void stopSpray2() { disablePort(SPRAY2_PORT); }

void startSpray3() { enablePort(SPRAY3_PORT); }
void stopSpray3() { disablePort(SPRAY3_PORT); }

void startSpray4() { enablePort(SPRAY4_PORT); }
void stopSpray4() { disablePort(SPRAY4_PORT); }

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
