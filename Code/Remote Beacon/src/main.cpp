#include <Arduino.h>
#include <Wifi.h>
#include <esp_now.h>

/*
Purpose: This code is responsible for controlling the entirety of the gate actuation system. 
Author: James C. Holland
Date: 2/25/2023
*/

/////////////////
// For ESP-NOW //
/////////////////
uint8_t broadcastAddress[] = {0x30, 0xC6, 0xF7, 0x28, 0xDA, 0x98}; // 30:C6:F7:28:DA:98

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  bool e_stop;
  bool toggle;
  int speed = 255;    // Possible feature
} struct_message;

// Create a struct_message called myData
struct_message myData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Deep sleep set up

RTC_DATA_ATTR int bootCount = 0;

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

///////////////////////
// Gate Interfacing  //
///////////////////////

const int gate_toggle = 12; // For toggling the gate to open or close, pressing this while gate is in travel will stop gate
const int e_stop_pin = 19;       // Only sends kill signal to stop gate motion
bool e_stop = 0;

void setup() {
  // Init Serial Monitor
  Serial.begin(9600);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // Serial setup
  Serial.begin( 9600 );
  delay(50);
  ++bootCount; // Increment boot counter
  Serial.print( "\n\nRemote: " +bootCount);
  Serial.println("\n\n");
  print_wakeup_reason(); // Display wakeup reason

  // Deep sleep setup
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_12,1);

  // Remote logic setup
  pinMode(gate_toggle, INPUT_PULLDOWN);
  pinMode(e_stop_pin, INPUT_PULLDOWN);

  // For ESP-NOW
  Serial.println(WiFi.macAddress());  
}
 
void loop() {
  bool msg_sent = false;
  while ((digitalRead(gate_toggle)==HIGH)||(digitalRead(e_stop_pin)==HIGH))
  {
    // Check if e-stop engaged 
    if(e_stop || digitalRead(e_stop_pin) == true){
      Serial.println("Estop ");
      myData.e_stop = 1;
    }
    else{
      myData.e_stop = 0;
    }

    if(digitalRead(gate_toggle)==true)// If gate has been toggled
    {
      Serial.println("Toggle");
      myData.toggle = 1;
    }

    // Send message via ESP-NOW
    // if(msg_sent != true)
    // {
      Serial.println("Command Sent");
      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));  
      myData.toggle=0; // reset flag
      msg_sent = true;
    // }
    delay(2000);
  }

  //Go to sleep now
  Serial.println("Going to sleep now");
  delay(1000);
  esp_deep_sleep_start();
  Serial.println("This will never be printed"); 
}