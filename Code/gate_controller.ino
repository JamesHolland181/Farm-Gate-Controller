/*
Purpose:
Author: James C. Holland
Date: 2/25/2023
*/

void setup() {
  // PWM pins to allow for speed control
  int fwd_pin = D5; 
  int rev_pin = D6; 

  // Digital pins to enable the direction (forward/reverse)
  int fwd_enable = D2;
  int rev_enable = D3;

  // Analog inputs to monitor current draw for safety features
  int fwd_current = A0;
  int rev_current = A1;

  // Set pin modes
  pinMode(fwd_pin, OUTPUT);
  pinMode(rev_pin, OUTPUT);
  pinMode(fwd_enable, OUTPUT);
  pinMode(rev_enable, OUTPUT);
  pinMode(fwd_current, INPUT);
  pinMode(rev_current, INPUT);
}

void loop() {
  // 
  
}
