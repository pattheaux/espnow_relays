// Relay Stuff
#include <ESP8266WiFi.h>
#include <espnow.h>

uint8_t rxrelay_ir_send = 0;
uint8_t rxrelay_ir_command = 0x00;

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
static int relay_value = 0;
static button_t buttons[NBUTTONS] = { { D1, 0, 0, 0, 0 }, { D2, 0, 0, 0, 0 }, { D3, 0, 0, 0, 0 }, { D4, 0, 0, 0, 0 } };

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

static delayed_t delayed = { -1, 0 };

static unsigned long timerDelay = 1000;

// Create a struct_message called myData
static struct_message myData;

static char first = 1;
static char blink = 0;

void rxrelay_setRelay(uint8_t relay, uint8_t value, int mode) {
  if(mode == 1) {
    // IR LEDs. Value is command
    Serial.println("LED COMMAND");
    rxrelay_ir_command = value;
    rxrelay_ir_send = 1;
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
static void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
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
  int mode = myData.data[0];

  rxrelay_setRelay(relay,value,mode);

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
 
void rxrelay_setup() {
  int i;
  // Initialize Serial Monitor
  Serial.println("rxrelay starting");

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

  //ir_setup();
}

static void loop_button(button_t *btn, int i);

void rxrelay_loop() {
  int i;

  if(first) {
    first = 0;
    Serial.println("rxrelay started");
    //ir_send_command = 1;
    return;
  }

  if((delayed.runAt > 0) && (delayed.runAt < millis())) {
    Serial.print("Delay clear relay ");
    Serial.println(delayed.relay);

    rxrelay_setRelay(delayed.relay,0,0);
    delayed.runAt = 0;
  }

  //ir_loop();

  for(i=0;i<NBUTTONS;i++) {
    loop_button(buttons+i,i);
  }
}

static void loop_button(button_t *btn, int i) {

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
    rxrelay_setRelay(i, !btn->value, 0);

    #ifdef TESTBUTTONS
    if(i==0) {
      // Test IR
      if(sCommand == 0x0D) sCommand = 0x1F;
      else sCommand = 0x0D;

      ir_send_command = 1;
    } else if(i==1) {
      // Test momentary
      rxrelay_setRelay(2, 200, 0);
    }
    #endif
    if(i==3) {
      // Test momentary
      Serial.println("TEST momentary");
      rxrelay_setRelay(3, 200, 0);
    }

    return;
  }
}
