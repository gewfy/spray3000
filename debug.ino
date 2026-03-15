const int latchPin = 10; // Pin LT on Relay board
const int dataPin = 11;  // Pin DS on Relay board
const int clockPin = 13; // Pin CL on Relay board

int portState = 0;

void setup() {
  pinMode(latchPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
}

void loop() {
  for (int i = 0; i <= 15; i++) {
    enablePort(i);
    delay(200);
  }

  for (int i = 0; i <= 15; i++) {
    disablePort(i);
    delay(200);
  }
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
  // Send highest byte first, then lowest byte
  shiftOut(dataPin, clockPin, MSBFIRST, highByte(portState));
  shiftOut(dataPin, clockPin, MSBFIRST, lowByte(portState));
  digitalWrite(latchPin, HIGH);
}
