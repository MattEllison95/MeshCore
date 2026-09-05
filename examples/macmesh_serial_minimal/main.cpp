#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#define SSD1306_NO_SPLASH
#include <Adafruit_SSD1306.h>

namespace {
constexpr int kMacRxPin = 44;
constexpr int kMacTxPin = 43;
constexpr unsigned long kMacBaud = 57600;
constexpr int kVextEnablePin = PIN_VEXT_EN;
constexpr int kVextActiveLevel = PIN_VEXT_EN_ACTIVE;
constexpr int kOledResetPin = PIN_OLED_RESET;
static_assert(kVextActiveLevel == LOW,
              "Heltec V4 OLED Vext must remain active-low");
constexpr uint8_t kRequest[] = "MMTEST1?\r\n";
constexpr uint8_t kReply[] = "MMTEST1!\r\n";

HardwareSerial macUart(1);
Adafruit_SSD1306 oled(128, 64, &Wire, kOledResetPin);
size_t matched = 0;
unsigned long rxBytes = 0;
unsigned long replies = 0;
unsigned long nextFrame = 0;

void drawStatus() {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.println("MacMesh serial test");
  oled.println("57600 8N1");
  oled.print("Mac RX: ");
  oled.println(rxBytes);
  oled.print("Replies: ");
  oled.println(replies);
  oled.println("MMTEST1 responder");
  oled.display();
}
}  // namespace

void setup() {
  Serial.begin(115200);
  pinMode(kVextEnablePin, OUTPUT);
  // Heltec V4 Vext is active-low.  Give the panel a real power cycle, then
  // leave the peripheral rail enabled while Adafruit_SSD1306 owns reset.
  digitalWrite(kVextEnablePin, !kVextActiveLevel);
  delay(25);
  digitalWrite(kVextEnablePin, kVextActiveLevel);
  delay(100);
  Wire.begin(PIN_BOARD_SDA, PIN_BOARD_SCL);
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C, true, false);
  macUart.setPins(kMacRxPin, kMacTxPin);
  macUart.begin(kMacBaud, SERIAL_8N1, kMacRxPin, kMacTxPin);
  drawStatus();
}

void loop() {
  while (macUart.available()) {
    const int value = macUart.read();
    if (value < 0) break;
    ++rxBytes;
    const uint8_t byte = static_cast<uint8_t>(value);
    if (byte == kRequest[matched]) {
      ++matched;
    } else {
      matched = byte == kRequest[0] ? 1 : 0;
    }
    if (matched == sizeof(kRequest) - 1) {
      macUart.write(kReply, sizeof(kReply) - 1);
      macUart.flush();
      ++replies;
      matched = 0;
    }
  }
  if (millis() >= nextFrame) {
    nextFrame = millis() + 250;
    drawStatus();
  }
}
