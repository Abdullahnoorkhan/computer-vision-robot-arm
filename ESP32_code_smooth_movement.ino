#include <Servo.h>

// ================= CONFIGURATION =================
const int ss = 6;
String axes[6] = {"M1", "M2", "M3", "M4", "M5", "M6"};
int pins[6] = {5, 4, 0, 16, 14, 12};

int rawPos[6] = {90, 90, 90, 90, 90, 90};
float currentPos[6] = {90, 90, 90, 90, 90, 90};

int minPos[6] = {0, 0, 0, 0, 0, 45};
int maxPos[6] = {180, 180, 180, 180, 180, 94};

Servo servos[6];

float stepSize = 0.0;
int maxDelayMs = 1500;

// ================= PWM CONVERSION =================
int angleToPwm(int index, float value) {
  value = constrain(value, minPos[index], maxPos[index]);
  return (int)((2000.0 / 180.0) * value + 500);
}

// ================= SYNCHRONIZED MOVE =================
bool moveServo(int index, float target) {
  float oldPos[ss];
  for (int i = 0; i < ss; i++) oldPos[i] = currentPos[i];

  // Update single axis if requested
  if (index >= 0 && index < ss) {
    currentPos[index] = constrain(target, minPos[index], maxPos[index]);
  }

  int maxDiff = 0;
  for (int i = 0; i < ss; i++) {
    int diff = (int)abs(currentPos[i] - oldPos[i]);
    maxDiff = max(maxDiff, diff);
  }

  // Write all servos together
  for (int i = 0; i < ss; i++) {
    servos[i].writeMicroseconds(angleToPwm(i, currentPos[i]));
  }

  delay(constrain(maxDiff * 4, 0, maxDelayMs));
  return maxDiff != 0;
}

// ================= COMMAND PARSER =================
String executeCommand(String cmd) {
  cmd.trim();
  if (cmd == "") return "";

  cmd.toUpperCase();

  // HOME
  if (cmd == "H") {
    for (int i = 0; i < ss; i++) currentPos[i] = rawPos[i];
    moveServo(-1, 0);   // move all together
    return "Homed all motors";
  }

  // Calculate targets first
  float target[ss];
  for (int i = 0; i < ss; i++) target[i] = currentPos[i];

  int start = 0;
  while (start < cmd.length()) {
    int comma = cmd.indexOf(',', start);
    String token = (comma == -1) ? cmd.substring(start) : cmd.substring(start, comma);
    token.trim();

    int space = token.indexOf(' ');
    if (space == -1) return "Syntax error";

    String axis = token.substring(0, space);
    String val  = token.substring(space + 1);
    axis.trim();
    val.trim();

    for (int i = 0; i < ss; i++) {
      if (axis == axes[i]) {
        if (val == "++") target[i] += 1;
        else if (val == "--") target[i] -= 1;
        else if (val == "?") target[i] = random(minPos[i], maxPos[i] + 1);
        else if (val.startsWith("+")) target[i] += val.substring(1).toFloat();
        else if (val.startsWith("-")) target[i] -= val.substring(1).toFloat();
        else target[i] = val.toFloat();

        target[i] = constrain(target[i], minPos[i], maxPos[i]);
      }
    }

    if (comma == -1) break;
    start = comma + 1;
  }

  // Apply all targets first
  for (int i = 0; i < ss; i++) currentPos[i] = target[i];

  // Single synchronized move
  moveServo(-1, 0);

  return "Motors moved simultaneously";
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  for (int i = 0; i < ss; i++) {
    servos[i].attach(pins[i], 500, 2500);
  }

  moveServo(-1, 0);
  Serial.println("Robotic arm ready");
}

// ================= LOOP =================
void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil(';');
    input.trim();
    if (input != "") {
      Serial.println("Received: " + input);
      Serial.println(executeCommand(input));
    }
  }
}
