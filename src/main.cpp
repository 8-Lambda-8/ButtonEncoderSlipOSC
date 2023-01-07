#include <Arduino.h>
#include <NeoPixelBus.h>
#include <OSCBoards.h>
#include <OSCBundle.h>
#include <OSCData.h>
#include <OSCMatch.h>
#include <OSCMessage.h>
#include <OSCTiming.h>

#ifdef BOARD_HAS_USB_SERIAL
#include <SLIPEncodedUSBSerial.h>
SLIPEncodedUSBSerial SLIPSerial(thisBoardsSerialUSB);
#else
#include <SLIPEncodedSerial.h>
SLIPEncodedSerial SLIPSerial(Serial);  // Change to Serial1 or Serial2 etc. for boards with
                                       // multiple serial ports that don’t have Serial
#endif

#include <Encoder.h>

#define LED_Pin 21

NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(6, LED_Pin);

#define colorSaturation 64
RgbColor red(colorSaturation, 0, 0);
RgbColor green(0, colorSaturation, 0);
RgbColor blue(0, 0, colorSaturation);
RgbColor white(colorSaturation);
RgbColor black(0);

RgbColor yellow(colorSaturation, colorSaturation, 0);
RgbColor magenta(colorSaturation, 0, colorSaturation);

RgbColor colors[] = {red, green, blue, white};

struct Button {
  uint8_t BtnPin;
  bool BtnState;
  bool BtnStateLast;
  uint8_t LedIndex;
  RgbColor LedColor;
};

Button buttons[4] = {{20, false, false, 0, red},
                     {19, false, false, 1, green},
                     {18, false, false, 2, blue},
                     {15, false, false, 3, white}};

#define EncoderBTN_0 2
#define EncoderDIR_0 3
#define EncoderCLK_0 4

#define EncoderBTN_1 5
#define EncoderDIR_1 6
#define EncoderCLK_1 7

Encoder Enc0(EncoderCLK_0, EncoderDIR_0);
Encoder Enc1(EncoderCLK_1, EncoderDIR_1);

Button encButtons[] = {{EncoderBTN_0, false, false, 4, yellow},
                       {EncoderBTN_1, false, false, 5, magenta}};

uint16_t encStates[] = {0, 0};

enum WHEEL_TYPE { INT, TILT, PAN, ZOOM, HUE, SAT, RED, GREEN, BLUE };
enum WHEEL_MODE { COARSE, FINE };

void setup() {
  Serial.begin(115200);

  strip.Begin();
  strip.Show();

  for (auto &&btn : buttons) {
    pinMode(btn.BtnPin, INPUT);
    strip.SetPixelColor(btn.LedIndex, btn.LedColor);
  }
  strip.SetPixelColor(4, blue);
  strip.SetPixelColor(5, magenta);

  strip.Show();

  delay(200);

  for (auto &&btn : buttons) {
    strip.SetPixelColor(btn.LedIndex, black);
  }
  strip.Show();
}

bool buttonRead(Button btn) { return digitalRead(btn.BtnPin); }

void updateButtons() {
  for (auto &&btn : buttons) {
    btn.BtnStateLast = btn.BtnState;
    btn.BtnState = buttonRead(btn);
  }
  for (auto &&btn : encButtons) {
    btn.BtnStateLast = btn.BtnState;
    btn.BtnState = buttonRead(btn);
  }
}

void parseOSCMessage(String &msg) {
  // prepare the message for routing by filling an OSCMessage object with our message string
  OSCMessage oscmsg;
  oscmsg.fill((uint8_t *)msg.c_str(), (int)msg.length());
  // route pan/tilt messages to the relevant update function
  oscmsg.route("/wheel/pan", [](OSCMessage &msg, int addressOffset) {
    // Enc0.write(msg.getOSCData(0)->getInt());
    // encStates[0] = msg.getOSCData(0)->getInt();
  });

  oscmsg.route("/wheel/tilt", [](OSCMessage &msg, int addressOffset) {
    // Enc1.write(msg.getOSCData(0)->getInt());
    // encStates[1] = msg.getOSCData(0)->getInt();
  });

  oscmsg.route("/key", [](OSCMessage &msg, int addressOffset) {
    
    char buffer[10];
    msg.getAddress(buffer, addressOffset + 1);
    uint8_t val = msg.getFloat(0);
    strip.SetPixelColor((uint16_t)atoi(buffer), RgbColor(val * 255, 0, val * 255));

    strip.Show();
  });
}

void sendWheelMove(WHEEL_TYPE type, int32_t ticks) {
  String wheelMsg("/wheel/");
  float outVal = float(ticks);

  if (type == PAN) {
    wheelMsg.concat("pan");
    // encStates[0] += ticks;
    // outVal = encStates[0];
  } else if (type == TILT) {
    wheelMsg.concat("tilt");
    // encStates[1] += ticks;
    // outVal = encStates[1];
  } else  // something has gone very wrong
    return;

  OSCMessage wheelUpdate(wheelMsg.c_str());
  wheelUpdate.add(outVal);
  SLIPSerial.beginPacket();
  wheelUpdate.send(SLIPSerial);
  SLIPSerial.endPacket();
}

void sendKeyPress(bool down, String key) {
  key = "/key/" + key;
  OSCMessage keyMsg(key.c_str());

  if (down)
    keyMsg.add(0.0f);
  else
    keyMsg.add(1.0f);

  SLIPSerial.beginPacket();
  keyMsg.send(SLIPSerial);
  SLIPSerial.endPacket();
}

void loop() {
  static String curMsg;
  int size;

  updateButtons();
  
  // Read Encoders
  if (Enc0.read() != 0) {
    sendWheelMove(PAN, Enc0.readAndReset());
  }
  if (Enc1.read() != 0) {
    sendWheelMove(TILT, Enc1.readAndReset());
  }

  // Send buttons
  for (auto &&btn : buttons) {
    if (btn.BtnState != btn.BtnStateLast) {
      sendKeyPress(btn.BtnState, String(btn.LedIndex));
    }
  }
  // Send Enc buttons
  for (auto &&btn : encButtons) {
    if (btn.BtnState != btn.BtnStateLast) {
      sendKeyPress(btn.BtnState, String(btn.LedIndex));
    }
  }

  size = SLIPSerial.available();
  if (size > 0) {
    // Fill the msg with all of the available bytes
    while (size--) curMsg += (char)(SLIPSerial.read());
  }
  if (SLIPSerial.endofPacket()) {
    parseOSCMessage(curMsg);

    curMsg = String();
  }
}
