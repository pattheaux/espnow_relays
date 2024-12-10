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

static const uint8_t BUTTON  = D1;

uint8_t mac0[] = {0xc4, 0xd8, 0xd5, 0x13, 0x3d, 0xfc}; // dead
uint8_t macx[] = {0xc4, 0xd8, 0xd5, 0x0e, 0xbf, 0xc1};
uint8_t mac1[] = {0xbc, 0xdd, 0xc2, 0x79, 0x92, 0x2d};
//mac2 bc:dd:c2:79:92:2d

#define NMACS 2
// REPLACE WITH RECEIVER MAC Address
uint8_t *macs[] = {
  mac0,
  mac1,
  NULL
};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  int module;
  int relay;
  int value;
  int data[32];
} struct_message;

// Create a struct_message called myData
struct_message myData;

typedef struct button {
  uint8_t button;
  uint8_t bstate;
  unsigned long lastTime;
  uint8_t value;
  uint8_t on;
} button_t;

#define NBUTTONS 4
int relay_value = 0;
button_t buttons[NBUTTONS] = { { D1, 0, 0, 0, 0 }, { D2, 0, 0, 0, 0 }, { D3, 0, 0, 0, 0 }, { D6, 0, 0, 0, 1 } };

uint8_t startup = 1;

typedef struct mmode_t {
  uint8_t mode;
  uint8_t submode;
  uint8_t flash;
  unsigned long last; 
  unsigned long next;
  unsigned long nextFlash;
} mmode_t;

unsigned long durations[] = { 1000, 1000, 1000, 1000 };
mmode_t mmode = { 0, 0, 0, 0, 0, 0 };

//unsigned long lastTime = 0;
unsigned long timerDelay = 1000;  // send readings timer

void send(int module, int relay, int value);

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0){
    Serial.println("Delivery success");
  } else {
    Serial.println("Delivery fail");
  }
}
 
void setup() {
  int i;
  // Init Serial Monitor
  Serial.begin(115200);
 
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(D5, OUTPUT);
  for(i=0;i<NBUTTONS;i++) {
    if(buttons[i].on) {
      pinMode(buttons[i].button, INPUT);
    } else {
      pinMode(buttons[i].button, INPUT_PULLUP);
    }
    buttons[i].bstate = 0;
    buttons[i].lastTime = 0;
    buttons[i].value = 0;
  }

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  memset((uint8_t *) &myData, 0, sizeof(myData));

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_add_peer(macs[0], ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
}
 
void flash(int on, int setNext) {
  digitalWrite(D5, on?LOW:HIGH);
  if(!setNext) {
    mmode.nextFlash = 0;
    mmode.flash = 1;
    if(!on) analogWrite(D5, 220); // dim
    return;
  }

  mmode.flash = !mmode.flash;
  mmode.nextFlash = (mmode.flash?100:200) + millis();
}

void mode1Next() {
  Serial.print("MODE1 step ");
  Serial.println(mmode.submode);
  //digitalWrite(D5, (mmode.submode%2==0)?LOW:HIGH);
  send((int)(mmode.submode/4),mmode.submode%4,1);
  mmode.next = millis() + 800;
  mmode.submode++;
  if(mmode.submode >= 8) {
    Serial.println("MODE1 finished");
    mmode.mode = 0;
    flash(1, 0);
    //digitalWrite(D5, HIGH);
  }
}

void mode2Next() {
  Serial.print("MODE2 step ");
  Serial.println(mmode.submode);
  send((int)(mmode.submode/4),mmode.submode%4,0);
  mmode.next = millis() + 500;
  mmode.submode++;
  if(mmode.submode >= 8) {
    Serial.println("MODE2 finished");
    mmode.mode = 0;
    flash(0, 0);
  }
}

void modeNext() {
    Serial.print("NEXT mode=");
    Serial.println(mmode.mode);
    switch(mmode.mode) {
      case 1: mode1Next(); return;
      case 2: mode2Next(); return;
    }
}

void loop() {
  int i;

  if(startup) {
    startup = 0;
    digitalWrite(D5, HIGH);
    Serial.println("sender start");
    analogWriteRange(255);
    flash(0,0);
  }

  if(mmode.nextFlash > 0 && mmode.nextFlash <= millis()) {
    flash(mmode.flash, 1);
  }

  if(mmode.mode > 0 && mmode.next <= millis()) {
    modeNext();
  }

  for(i=0;i<NBUTTONS;i++) {
    loop_button(buttons+i,i);
  }
}

void loop_button(button_t *btn, int i) {

  uint8_t mode = btn->bstate;
  uint8_t bstate = digitalRead(btn->button);
  if(btn->on) {
    bstate = !bstate;
  }

  if ((mode == 1) && ((millis() - btn->lastTime) > timerDelay) && (bstate)) {
    Serial.print("disarm ");
    Serial.println(i);
    btn->bstate = 0;

    digitalWrite(LED_BUILTIN, HIGH);
    return;
  }

  if((mode == 0) && (!bstate)) {
    btn->bstate = 1;
    btn->lastTime = millis();

    digitalWrite(LED_BUILTIN, LOW);
    btn->value = !btn->value;
    if(btn->button == D6) {
      memset(&mmode,0,sizeof(mmode_t));
      mmode.mode = btn->value?1:2;
      mmode.next = millis();
      flash(1, 1);
      //digitalWrite(D5, btn->value?LOW:HIGH);  // low is on?
    }

    Serial.print("click ");
    Serial.print(i);
    Serial.print("=");
    Serial.println(btn->value);
  }
}

void send(int module, int relay, int value) {
    // Set values to send
    myData.module = module;
    myData.relay = relay;
    myData.value = value;

    // Send message via ESP-NOW
    Serial.print("send M:");
    Serial.print(myData.module);
    Serial.print(" R:");
    Serial.print(myData.relay);
    Serial.print("=");
    Serial.print(myData.value);
    Serial.println();
    esp_now_send(macs[myData.module], (uint8_t *) &myData, sizeof(myData));
    return;
}