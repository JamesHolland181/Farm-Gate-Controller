#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

/////////////////////////
// Current Sensor Code //
/////////////////////////
const int inPin =A0;//can change
const float SHUNT_CURRENT =10.00;//A
const float SHUNT_VOLTAGE =75.0;// mV
const float CORRECTION_FACTOR = 2.00;

const int ITERATION = 30; //can change (see video)
const float VOLTAGE_REFERENCE = 3.3;//1.1V
const int BIT_RESOLUTION =10;//and 12 for Due and MKR
const boolean DEBUG_ONCE = true;

const int threshold = 5; // Amps

float getCurrent()
{

    float averageSensorValue =0;
    int sensorValue ;
    float voltage, current;

    for(int i=0; i< ITERATION; i++)
    {   
      sensorValue = analogRead(inPin);
      delay(5);
      if(sensorValue !=0 && sensorValue < 100)
      {
        voltage = (sensorValue +0.5) * (VOLTAGE_REFERENCE /  (pow(2,BIT_RESOLUTION)-1)); 
        current  = voltage * (SHUNT_CURRENT /SHUNT_VOLTAGE )  ;
        if(i !=0){
          averageSensorValue += current+CORRECTION_FACTOR;
        }
        delay(1);
      }
    }    

    averageSensorValue /=(ITERATION-1);
    return  averageSensorValue;
}
 
/////////////////
// For ESP-NOW //
/////////////////
uint8_t broadcastAddress[] = {0x30, 0xC6, 0xF7, 0x03, 0x83, 0x7C}; // 30:C6:F7:03:83:7C

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  bool e_stop;
  bool toggle;
  // int gate_command; // Essentially the desired state
  int speed = 255;    // Possible feature
} struct_message;

// Create a struct_message called command
struct_message command;
bool toggle_state;
bool estop;
int gate_state;

esp_now_peer_info_t peerInfo;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&command, incomingData, sizeof(command));
  toggle_state = command.toggle; 
  estop = command.e_stop;
  
  Serial.print("Toggle: ");
  Serial.println(toggle_state);

  Serial.print("E-Stop: "); Serial.println(estop);
  Serial.println(); 
}

// Timer intervals
const int open_time = 5;
const int close_time = 5;
const int wait_time = 30;

// Motor limit switches
const int m1_extension = 36; // GPIO 36 (input only) used for checking motor 1's extension switch
const int m1_retraction = 39; // GPIO 39 (input only) used for checking motor 1's retraction switch

// PWM pins to allow for speed control
const uint8_t fwd_pin = 25; // GPIO 25
const uint8_t rev_pin = 26; // GPIO 26

// Pins for interfacing with the motor driver
const uint8_t fwd_enable = 27; // GPIO 27 for use as digital output
const uint8_t rev_enable = 14; // GPIO 14 for use as digital output

const uint8_t f_ch = 0;
const uint8_t r_ch = 8;

// Analog inputs to monitor current draw for safety features
// const uint8_t fwd_current = 12; // GPIO 32 for use as an ADC
// const uint8_t rev_current = 13; // GPIO 33 for use as an ADC
 
/////////////////
// Timer setup //
/////////////////
hw_timer_t *timer = NULL;
float recovery_time;

void motor_control(bool dir){
  if(dir == 1){ //Fwd
    digitalWrite(fwd_enable, HIGH);
    digitalWrite(rev_enable, LOW);
  }
  else{
    digitalWrite(fwd_enable, LOW);
    digitalWrite(rev_enable, HIGH);
  }


  while((timerReadSeconds(timer)+recovery_time < open_time) && (getCurrent() < 10) && (estop != 1))
  {  
    if(dir == 1){ //Fwd
      digitalWrite(fwd_pin, HIGH);
      digitalWrite(rev_pin, LOW);
    }
    else{
      digitalWrite(fwd_pin, LOW);
      digitalWrite(rev_pin, HIGH);
    }
    
    Serial.println(timerReadSeconds(timer));
    delay(50);
  }

  if(estop){
    recovery_time = timerReadSeconds(timer);
  }
  else{
    toggle_state = 0;
    gate_state = 2;
    Serial.print("Opened: "); Serial.println(gate_state);  
    recovery_time=0;    
  }
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("Controller\n.......\n");
  timer = timerBegin(0, 80, true);   

  pinMode(fwd_enable, OUTPUT);
  pinMode(rev_enable, OUTPUT);
  pinMode(fwd_pin, OUTPUT);
  pinMode(rev_pin, OUTPUT);  
}
 
void loop() {
// timerRestart(timer);
  // Skip the below if a toggle command isn't being sent
  while((toggle_state != 0) && (estop != 1)){
    timerRestart(timer); 
    Serial.println(timerReadSeconds(timer));

    switch (gate_state)
    {
    case 0:
      //gate_home()
      gate_state = 1;
      break;

    case 1:
      // timerRestart(timer);
      digitalWrite(fwd_enable, HIGH);
      digitalWrite(rev_enable, HIGH);

      while((timerReadSeconds(timer)+recovery_time < open_time) && (getCurrent() < 10) && (estop != 1))
      {  
        digitalWrite(fwd_pin, HIGH);
        digitalWrite(rev_pin, LOW);      
        Serial.println(timerReadSeconds(timer));
        delay(50);
      }

      if(estop){
        recovery_time = timerReadSeconds(timer);
      }
      else{
        toggle_state = 0;
        gate_state = 2;
        Serial.print("Opened: "); Serial.println(gate_state);  
        recovery_time=0;    
      }
      break;

    case 2:
      // timerRestart(timer);    
      digitalWrite(fwd_enable, HIGH);
      digitalWrite(rev_enable, HIGH);   
      
      while((timerReadSeconds(timer)+recovery_time < close_time) && (getCurrent() < 10) && (estop != 1))
      {  
        digitalWrite(fwd_pin, LOW);
        digitalWrite(rev_pin, HIGH);   
        Serial.println(timerReadSeconds(timer));
        delay(50);
      }

      if(estop){
        recovery_time = timerReadSeconds(timer);
      }
      else{
        toggle_state = 0;
        gate_state = 1;
        Serial.print("Closed: "); Serial.println(gate_state);      
        recovery_time=0;
      }
    
      break;      
    
    default:
      Serial.println("default");
      toggle_state = 0;
      break;
    }    

  }

  digitalWrite(fwd_enable, LOW);
  digitalWrite(rev_enable, LOW);  

  digitalWrite(fwd_pin, LOW);
  digitalWrite(rev_pin, LOW);  
}

