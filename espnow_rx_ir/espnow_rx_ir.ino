
// IR Stuff
#include <Arduino.h>

#if !defined(ARDUINO_ESP32C3_DEV) // This is due to a bug in RISC-V compiler, which requires unused function sections :-(.
#define DISABLE_CODE_FOR_RECEIVER // Disables static receiver code like receive timer ISR handler and static IRReceiver and irparams data. Saves 450 bytes program memory and 269 bytes RAM if receiving functions are not required.
#endif
//#define SEND_PWM_BY_TIMER         // Disable carrier PWM generation in software and use (restricted) hardware PWM.
//#define USE_NO_SEND_PWM           // Use no carrier PWM, just simulate an active low receiver signal. Overrides SEND_PWM_BY_TIMER definition

void ir_setup();

//uint8_t ir_send_command = 0;
//uint8_t sCommand = 0x0D; // room leds on

/*
 * This include defines the actual pin number for pins like IR_RECEIVE_PIN, IR_SEND_PIN for many different boards and architectures
 */
#include "PinDefinitionsAndMore.h"
#include <IRremote.hpp> // include the library

#include "rxrelay.h"

/*
// Relay Stuff
#include <ESP8266WiFi.h>
#include <espnow.h>

static const uint8_t D0   = 16;
static const uint8_t D1   = 5;
static const uint8_t D2   = 4;
static const uint8_t D3   = 0;
static const uint8_t D4   = 2;
static const uint8_t D5   = 14;
static const uint8_t D6   = 12;
static const uint8_t D7   = 13;
static const uint8_t D8   = 15;
static const uint8_t D9   = 3;
static const uint8_t D10  = 1;

static const uint8_t LEDOFF = HIGH;
static const uint8_t LEDON = LOW;

#define NRELAY 4
static const uint8_t relay_pins[] = { D5, D6, D7, D8 };

typedef struct button {
  uint8_t button;
  uint8_t bstate;
  unsigned long onTime;
  unsigned long lastTime;
  uint8_t value;
} button_t;

#define NBUTTONS 4
int relay_value = 0;
button_t buttons[NBUTTONS] = { { D1, 0, 0, 0, 0 }, { D2, 0, 0, 0, 0 }, { D3, 0, 0, 0, 0 }, { D4, 0, 0, 0, 0 } };

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  int module;
  int relay;
  int value;
  int data[32];
} struct_message;

typedef struct delayed {
  uint8_t relay;
  long runAt;
} delayed_t;

delayed_t delayed = { -1, 0 };

unsigned long timerDelay = 1000;

// Create a struct_message called myData
struct_message myData;

char first = 1;
char blink = 0;

void setRelay(uint8_t relay, uint8_t value) {
  if(relay == 3) {
    // IR LEDs. Value is command
    Serial.println("LED COMMAND");
    sCommand = value;
    ir_send_command = 1;
    return;
  }

  buttons[relay].value = value;
  digitalWrite(relay_pins[relay], value?HIGH:LOW);

  if(value > 1) {
    // Momentary Relay, value is the delay in ms before turning off
    Serial.print("delay off for relay ");
    Serial.print(relay);
    Serial.print(" after ");
    Serial.println(value);

    delayed.relay = relay;
    delayed.runAt = millis() + value;
  }
}

// Callback function that will be executed when data is received
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("M:");
  Serial.print(myData.module);
  Serial.print(" R:");
  Serial.print(myData.relay);
  Serial.print(" = ");
  Serial.print(myData.value);
  Serial.println();

  uint8_t relay = myData.relay; //relay_pins[myData.relay];
  int value = myData.value;

  setRelay(relay,value);

  //if(!blink) {
    //blink = 1;
    //digitalWrite(LED_BUILTIN, value?LEDON:LEDOFF);
    //digitalWrite(relay, value?HIGH:LOW);
  //} else {
  //  blink = 0;
  //  digitalWrite(LED_BUILTIN, LEDOFF);
  //  digitalWrite(relay, LOW);
  //}

}
 */

void setup() {
  int i;
  // Initialize Serial Monitor
  Serial.begin(115200);
  Serial.println("starting");

/*
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);

  //pinMode(LED_BUILTIN, OUTPUT);
  for(i=0;i<NRELAY;i++) {
    pinMode(relay_pins[i], OUTPUT);
  }
  for(i=0;i<NBUTTONS;i++) {
    pinMode(buttons[i].button, INPUT_PULLUP);
  }
*/

  rxrelay_setup();
  ir_setup();
}

void loop() {
  int i;

/*
  if(first) {
    first = 0;
    Serial.println("rx started");
    //ir_send_command = 1;
    return;
  }

  if((delayed.runAt > 0) && (delayed.runAt < millis())) {
    Serial.print("Delay clear relay ");
    Serial.println(delayed.relay);

    setRelay(delayed.relay,0);
    delayed.runAt = 0;
  }

  ir_loop();

  for(i=0;i<NBUTTONS;i++) {
    loop_button(buttons+i,i);
  }
*/
  rxrelay_loop();
  ir_loop();
}

/*
void loop_button(button_t *btn, int i) {

  uint8_t mode = btn->bstate;
  uint8_t bstate = digitalRead(btn->button);

  if ((mode == 1) && ((millis() - btn->lastTime) > timerDelay) && (bstate)) {
    // Debounce the release
    Serial.print("disarm ");
    Serial.println(i);
    btn->bstate = 0;

    return;
  }

  if((mode == 0) && (!bstate)) {
    Serial.println("press");
    btn->bstate = 3;
    btn->onTime = millis() + 50;
    return;
  }

  if((mode == 3) && (bstate)) {
    // Debounce the press
    //Serial.println("unpress");
    mode = 0;
    return;
  }

  if((mode == 3) && (!bstate) && (btn->onTime < millis())) {
    btn->bstate = 1;
    btn->lastTime = millis();

    Serial.print("click ");
    Serial.print(i);
    Serial.print("=");
    Serial.println(!btn->value);
    setRelay(i, !btn->value);

    #ifdef TESTBUTTONS
    if(i==0) {
      // Test IR
      if(sCommand == 0x0D) sCommand = 0x1F;
      else sCommand = 0x0D;

      ir_send_command = 1;
    } else if(i==1) {
      // Test momentary
      setRelay(2, 200);
    }
    #endif

    return;
  }
}
*/

void ir_setup() {
    //pinMode(LED_BUILTIN, OUTPUT);

    //Serial.begin(115200);

    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));
    Serial.print(F("Send IR signals at pin "));
    Serial.println(IR_SEND_PIN);

    /*
     * The IR library setup. That's all!
     */
    IrSender.begin(); // Start with IR_SEND_PIN -which is defined in PinDefinitionsAndMore.h- as send pin and enable feedback LED at default feedback LED pin
    disableLEDFeedback(); // Disable feedback LED at default feedback LED pin
}

/*
 * Set up the data to be sent.
 * For most protocols, the data is build up with a constant 8 (or 16 byte) address
 * and a variable 8 bit command.
 * There are exceptions like Sony and Denon, which have 5 bit address.
 */

uint8_t sRepeats = 4;

void ir_loop() {
    if(rxrelay_ir_send <= 0) {
      return;
    }

    /*
     * Print current send values
     */
    Serial.println();
    Serial.print(F("Send now: address=0x00, command=0x"));
    Serial.print(rxrelay_ir_command, HEX);
    Serial.print(F(", repeats="));
    Serial.print(sRepeats);
    Serial.println();

    Serial.println(F("Send standard NEC with 8 bit address"));
    Serial.flush();

    // Receiver output for the first loop must be: Protocol=NEC Address=0x102 Command=0x34 Raw-Data=0xCB340102 (32 bits)
    IrSender.sendNEC(0x00, rxrelay_ir_command, sRepeats);

    rxrelay_ir_send = 0;
}

