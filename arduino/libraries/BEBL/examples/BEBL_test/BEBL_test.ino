#include "Wire.h"
#include "ADXL345.h"

int LEDS[] = {5, 6, 7, 9, 10, 13};
int n_led = 6;

ADXL345 Accel;

void setup(){
  Serial.begin(115200);
  for(int i=0; i<n_led; i++){
    pinMode(LEDS[i], OUTPUT);
  }
  Accel.powerOn();
  Accel.set_bw(ADXL345_BW_3); // 3 6 12 25 50 100 200 400 800 1600
  // Accel.set_bw(ADXL345_BW_1600);
  sweep(500);
}

void sweep(int d){
  int i;
  for(i=0; i<n_led; i++){
    digitalWrite(LEDS[i], HIGH);
    delay(d);
    digitalWrite(LEDS[i], LOW);
  }
  for(i=0; i<n_led; i++){
    digitalWrite(LEDS[n_led - i - 1], HIGH);
    delay(d);
    digitalWrite(LEDS[n_led - i - 1], LOW);
  }
}

int led;

void loop(){
  int i;
  double acc_data[3];
  Accel.get_Gxyz(acc_data);

  if(Accel.status){
    float length = 0.;
    for(i = 0; i < 3; i++){
      length += (float)acc_data[i] * (float)acc_data[i];
      Serial.print(acc_data[i]);
      Serial.print(" ");
    }
    length = sqrt(length);
    Serial.print(length);
    Serial.println("");

    for(i=0; i<n_led; i++){
      if(acc_data[2] < -.1 * (i + 1)){
	digitalWrite(LEDS[i], HIGH);
      }
      else{
	digitalWrite(LEDS[i], LOW);
      }
    }
  }
  else{
    Serial.print("ERROR: ADXL345 data read error:");
    Serial.println(Accel.error_code);
    digitalWrite(7, HIGH);
    while(1) delay(1000);
  }
}
