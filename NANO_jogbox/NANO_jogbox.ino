// CNC Jog Controller
// keyID | Action
// ------|------------------
//   0   | Z-
//   1   | A-
//   2   | Z+
//   3   | Z+
//   4   | Y-
//   5   | X+
//   6   | Y+
//   7   | X-
//   8   | INC 0.001
//   9   | INC 0.010
//  10   | INC 0.100
//  11   | INC 1.000
//  12   | Zero A
//  13   | Zero Z
//  14   | Zero Y
//  15   | Zero X
//  16   | Home A
//  17   | Home Z
//  18   | Home Y
//  19   | Home X
//  22   | Return to Zero
//  23   | Home All
//  25   | Run
//  26   | Pause
//  27   | Stop

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include <esp_bt.h>
#include <esp_wifi.h>
#include <stdio.h>
#include <string.h>

// Core data structures
union FloatUnion {
  float f;
  uint8_t b[4];
};

struct PendantStatus {
  float x, y, z, a;
  byte status;
  float feedRate, line, total, stepSize, liveFeedRate;
};
PendantStatus ps;
// LCD abstraction layer
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

inline void lcdInitScreen() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
}

inline void lcdClearScreen() {
  display.clearDisplay();
}

inline void lcdGoTo(uint8_t col, uint8_t row) {
  display.setCursor(col * 6, row * 9);  // 6px width font, 8px height
}

inline void lcdPrint(const String &s) {
  display.print(s);
}

inline void lcdPrint(const char *s) {
  display.print(s);
}

inline void lcdDisplay() {
  display.display();
}

static const unsigned int PACKET_SIZE = 40;
static uint8_t dataBuf[PACKET_SIZE];
static uint8_t readBuf[PACKET_SIZE + 4];

// Serial over Bluetooth
HardwareSerial BTSerial(1);

// Keypad matrix pins
const uint8_t rowPins[] = { D3, D4, D5, D6, D7, D8, A0 };
const uint8_t colPins[] = { D9, D10, D11, D12 };
const uint8_t NUM_ROWS = sizeof(rowPins) / sizeof(rowPins[0]);
const uint8_t NUM_COLS = sizeof(colPins) / sizeof(colPins[0]);
const uint16_t DEBOUNCE_DELAY = 40;
const unsigned long REPEAT_INTERVAL = 600;

// Timing and state
unsigned long lastCounter = 0;
unsigned long lastAckTime = 0;
const unsigned long ACK_TIMEOUT = 3000;
bool connected = false;
bool waitingForSync = false;

unsigned long lastLineUpdate = 0;
const unsigned long LINE_UPDATE_INTERVAL = 55;  // ~30 FPS
unsigned long lastReconnectAttempt = 0;
const unsigned long RECONNECT_INTERVAL = 1000;

String buildDate = __DATE__;  // e.g., "Jul  4 2025"
String buildTime = __TIME__;  // e.g., "11:15:42"


bool readFloat(Stream &s, float &outVal) {
  uint8_t raw[4];

  for (int i = 0; i < 4; ++i) {
    int c = s.read();
    if (c < 0) return false;
    raw[i] = (uint8_t)c;
  }

  // Print raw bytes as decimal
  // Serial.print("DEC: ");
  // for (int i = 0; i < 4; ++i) {
  //   Serial.print(raw[i]);
  //   if (i < 3) Serial.print(" ");
  // }

  // Convert to float
  memcpy(&outVal, raw, 4);
  // Serial.print(" => ");
  // Serial.println(outVal, 3);

  return true;
}


// Parse the PendantStatus packet
PendantStatus readPendantStatusFromBuffer(PendantStatus p) {
  int availableBytes = BTSerial.available();

  // Serial.print("Buf Size:");
  // Serial.println(availableBytes);
  memcpy(&p.x, dataBuf + 0, 4);
  memcpy(&p.y, dataBuf + 4, 4);
  memcpy(&p.z, dataBuf + 8, 4);
  memcpy(&p.a, dataBuf + 12, 4);
  float tempStatus;
  memcpy(&tempStatus, dataBuf + 16, 4);
  p.status = static_cast<byte>(tempStatus);
  memcpy(&p.feedRate, dataBuf + 20, 4);
  memcpy(&p.line, dataBuf + 24, 4);
  memcpy(&p.total, dataBuf + 28, 4);
  memcpy(&p.stepSize, dataBuf + 32, 4);
  memcpy(&p.liveFeedRate, dataBuf + 36, 4);


  // Serial.print("x: ");
  // Serial.println(p.x, 6);
  // Serial.print("y: ");
  // Serial.println(p.y, 6);
  // Serial.print("z: ");
  // Serial.println(p.z, 6);
  // Serial.print("a: ");
  // Serial.println(p.a, 6);
  // Serial.print("status: ");
  // Serial.println(p.status);
  // Serial.print("feedRate: ");
  // Serial.println(p.feedRate, 6);
  // Serial.print("line: ");
  // Serial.println(p.line, 6);
  // Serial.print("total: ");
  // Serial.println(p.total, 6);
  // Serial.print("stepSize: ");
  // Serial.println(p.stepSize, 6);
  // Serial.print("liveFeedRate: ");
  // Serial.println(p.liveFeedRate, 6);

  return p;
}

// Convert status code to text
const char *getState(byte status) {
  switch (status) {
    case 0: return "IDLE";
    case 1: return "HOLD";
    case 2: return "RUN";
    default: return "BUSY";
  }
}

// Right-pad a string to length
String padRight(const String &input, size_t width = 11) {
  String s = input;
  while (s.length() < width) s += ' ';
  return s;
}

String padRight20(const String &input) {
  return padRight(input, 20);
}

// Display the pendant status on the LCD
void printPendantStatus(const PendantStatus &p) {

  lcdGoTo(0, 0);
  lcdPrint(padRight("X" + String(p.x, 3)) + padRight("J" + String(p.feedRate, 2)));

  lcdGoTo(0, 1);
  lcdPrint(padRight("Y" + String(p.y, 3)) + padRight("F" + String(p.liveFeedRate, 2)));

  lcdGoTo(0, 2);
  lcdPrint(padRight("Z" + String(p.z, 3)));

  lcdGoTo(0, 3);
  lcdPrint(padRight("A" + String(p.a, 3)) + padRight(getState(p.status)));


  display.drawLine(58, 0, 58, 36, SSD1306_WHITE);

  lcdGoTo(0, 5);
  lcdPrint(padRight20("L" + String(p.line, 0) + "/" + String(p.total, 0)));

  lcdGoTo(0, 6);
  lcdPrint(padRight("Step:" + String(p.stepSize, 3)));
  lcdDisplay();
}

// Send a key command and immediately request new data
void Send_To_Keyout(int keyID) {
  if (keyID == 99 || keyID == -1) return;

  // Send keypress and then data request
  BTSerial.print("KEY:");
  BTSerial.println(keyID);
  delay(30);  // allow host to process
  BTSerial.println("REQ");
  waitingForSync = true;
}

// Scan the button matrix for a pressed key
int scanMatrix() {
  for (uint8_t r = 0; r < NUM_ROWS; ++r) {
    digitalWrite(rowPins[r], HIGH);
    delayMicroseconds(300);
    for (uint8_t c = 0; c < NUM_COLS; ++c) {
      if (digitalRead(colPins[c]) == HIGH) {
        delay(DEBOUNCE_DELAY);
        if (digitalRead(colPins[c]) == HIGH) {
          digitalWrite(rowPins[r], LOW);
          return r * NUM_COLS + c;
        }
      }
    }
    digitalWrite(rowPins[r], LOW);
  }
  return -1;
}

// Handle key action logic (including double-tap for modifiers)
void handleCommand(int keyID) {
  static int prevCmd = -1;
  static bool running = false, paused = false;

  switch (keyID) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
      Send_To_Keyout(keyID);
      break;

    case 12:
    case 13:
    case 14:
    case 15:
    case 16:
    case 17:
    case 18:
    case 19:
    case 22:
    case 23:
      if (prevCmd == keyID) {
        Send_To_Keyout(keyID);
        prevCmd = -1;
      } else {
        prevCmd = keyID;
      }
      break;

    case 25:
      if (!running) {
        running = true;
        Send_To_Keyout(keyID);
      }
      break;

    case 26:
      if (running) {
        paused = !paused;
        Send_To_Keyout(keyID);
      }
      break;

    case 27:
      if (running || paused) {
        running = paused = false;
        Send_To_Keyout(keyID);
      }
      break;

    default:
      prevCmd = -1;
      break;
  }
}
// Animated lines like a screensaver under row 3
struct Star {
  float x, y, z;
};

const int NUM_STARS = 64;
Star stars[NUM_STARS];

void initLines() {
  for (int i = 0; i < NUM_STARS; i++) {
    stars[i].x = random(-64, 64);
    stars[i].y = random(-32, 32);
    stars[i].z = random(8, 64);
  }
}

void Indicator() {
  display.fillRect(0, 12, 128, 64 - 12, SSD1306_BLACK);
  display.drawRect(0, 12, 128, 64 - 12, SSD1306_WHITE);

  for (int i = 0; i < NUM_STARS; i++) {
    float k = 64.0 / stars[i].z;
    int sx = (int)(stars[i].x * k) + 64;
    int sy = (int)(stars[i].y * k) + 38;

    if (sx >= 0 && sx < SCREEN_WIDTH && sy >= 12 && sy < SCREEN_HEIGHT) {
      display.drawPixel(sx, sy, SSD1306_WHITE);
    }

    stars[i].z -= 1;
    if (stars[i].z <= 1) {
      stars[i].x = random(-40, 40);
      stars[i].y = random(-20, 20);
      stars[i].z = 64;
    }
  }

  lcdDisplay();
  lastLineUpdate = millis();
}

// Attempt to establish connection with host
void reconnect() {

  lcdClearScreen();
  display.setCursor(4, 2);
  lcdPrint(" - CNC Controller - ");

  while (true) {
    Indicator();
    lcdDisplay();

    if (millis() - lastReconnectAttempt >= RECONNECT_INTERVAL) {
      lastReconnectAttempt = millis();
      BTSerial.println("CONREQ");
    }

    if (BTSerial.available()) {
      String resp = BTSerial.readStringUntil('\n');
      if (resp == "CONACK") {
        lcdClearScreen();
        lcdGoTo(0, 0);
        lcdPrint(" - CNC Controller - ");
        lcdGoTo(0, 1);
        lcdPrint("UGS Connected");
        lcdDisplay();
        connected = true;
        waitingForSync = true;
        BTSerial.println("REQ");
        BTSerial.flush();
        // Serial.println("REQ sent");
        delay(500);
        lastAckTime = millis();
        break;
      }
    }
  }
}

void setup() {
  esp_bt_controller_disable();
  esp_wifi_stop();
  delay(2000);

  Serial.begin(115200);
  BTSerial.begin(9600, SERIAL_8N1, A3, A2);
  Wire.begin(21, 22);

  lcdInitScreen();
  initLines();

  for (uint8_t i = 0; i < NUM_ROWS; i++) {
    pinMode(rowPins[i], OUTPUT);
    digitalWrite(rowPins[i], LOW);
  }
  for (uint8_t i = 0; i < NUM_COLS; i++) {
    pinMode(colPins[i], INPUT_PULLDOWN);
  }

  reconnect();
}


void loop() {
  unsigned long now = millis();
  static int holdKey = -1;
  static unsigned long lastRepeat = 0;
  static int waitDelay = 0;
  if (connected) {
    int keyID = scanMatrix();
    if (keyID >= 0 && keyID != holdKey) {
      holdKey = keyID;
      lastRepeat = now;
      handleCommand(keyID);
    } else if (keyID == holdKey && keyID <= 7 && now - lastRepeat >= REPEAT_INTERVAL) {
      Send_To_Keyout(keyID);
      lastRepeat = now;
    } else if (keyID != holdKey) {
      holdKey = -1;
    }
    // stop spamming readBuf when running
    if (ps.status == 2) {
      waitDelay = 1000;
    } else {
      waitDelay = 250;
    }
    if (now - lastCounter >= waitDelay) {
      lastCounter = now;

      // Serial.print("waitingForSynct ");
      // Serial.println(waitingForSync);
      if (connected && !waitingForSync) {
        waitingForSync = true;
        BTSerial.println("REQ");
        BTSerial.flush();
        delay(100);
        // Serial.println("REQ sent");
      }
    }
  }


  if (connected && BTSerial.available() > 2) {

    memset(readBuf, 0, sizeof(readBuf));
    lastAckTime = millis();
    size_t bytesRead = BTSerial.readBytes(readBuf, 44);
    // Serial.print("bytesRead" + bytesRead);
    // Serial.print("readBuf: filled [");
    // for (int i = 0; i < sizeof(readBuf); i++) {
    //   Serial.print((char)readBuf[i]);
    // }
    // Serial.println("]");

    if (readBuf[0] == 'S' && readBuf[1] == 'Y' && readBuf[2] == 'N' && readBuf[3] == 'C') {

      // Serial.print("readBuf: SYNC [");
      // for (int i = 0; i < sizeof(readBuf); i++) {
      //   Serial.print((char)readBuf[i]);
      // }
      // Serial.println("]");

      memcpy(dataBuf, readBuf + 4, PACKET_SIZE);

      PendantStatus p;
      ps = readPendantStatusFromBuffer(p);
      lcdClearScreen();
      printPendantStatus(ps);
      waitingForSync = false;

    } else {
      // Serial.print("readBuf: fail [");
      // for (int i = 0; i < sizeof(readBuf); i++) {
      //   Serial.print((char)readBuf[i]);
      // }
      // Serial.println("]");
    }
  }
  if (connected && (millis() - lastAckTime > ACK_TIMEOUT)) {
    connected = false;
    reconnect();
  }
}