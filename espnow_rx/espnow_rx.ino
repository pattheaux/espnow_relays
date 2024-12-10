/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp8266-nodemcu-arduino-ide/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

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
  unsigned long lastTime;
  uint8_t value;
} button_t;

#define NBUTTONS 4
int relay_value = 0;
button_t buttons[NBUTTONS] = { { D1, 0, 0, 0 }, { D2, 0, 0, 0 }, { D3, 0, 0, 0 }, { D4, 0, 0, 0 } };

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  int module;
  int relay;
  int value;
  int data[32];
} struct_message;


unsigned long timerDelay = 1000;

// Create a struct_message called myData
struct_message myData;

char first = 1;
char blink = 0;

void setRelay(uint8_t relay, uint8_t value) {
  buttons[relay].value = value;
  digitalWrite(relay_pins[relay], value?HIGH:LOW);
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
 
void setup() {
  int i;
  // Initialize Serial Monitor
  Serial.begin(115200);
  Serial.println("starting");

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
}

void loop() {
  int i;

  if(first) {
    first = 0;
    //digitalWrite(LED_BUILTIN, LEDOFF);
    Serial.println("rx started");
    return;
  }
  
  for(i=0;i<NBUTTONS;i++) {
    loop_button(buttons+i,i);
  }
}

void loop_button(button_t *btn, int i) {

  uint8_t mode = btn->bstate;
  uint8_t bstate = digitalRead(btn->button);

  if ((mode == 1) && ((millis() - btn->lastTime) > timerDelay) && (bstate)) {
    Serial.print("disarm ");
    Serial.println(i);
    btn->bstate = 0;

    return;
  }

  if((mode == 0) && (!bstate)) {
    btn->bstate = 1;
    btn->lastTime = millis();

    Serial.print("click ");
    Serial.print(i);
    Serial.print("=");
    Serial.println(!btn->value);
    setRelay(i, !btn->value);
    return;
  }
}

