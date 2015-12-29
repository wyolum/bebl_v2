/*
  Test out the LEDs and accel of BEBL
 */

#include "Wire.h"
#include "ADXL345.h"
#include <avr/sleep.h>

ADXL345 Accel;
const int LEDS[] = {5, 6, 7, 9, 10, 13}; // top to bottom
// const int LEDS[] = {7, 9, 6, 10, 5, 13}; // middle to outside

const int n_led = 6;

const int INTERRUPT_1 = 3;
const int INTERRUPT_2 = 2;
const int N_FILTER_CHANNEL = 3;
const int N_TAP = 3;
const int N_UP_TAP = 3;

// variable for filtered accel data
double cooked[N_FILTER_CHANNEL];

// variable for filtered up data
double up[N_FILTER_CHANNEL];
double back[N_FILTER_CHANNEL];
double braking = 0;
double braking_threshold = 0.05;

double in_data[N_TAP * N_FILTER_CHANNEL];
double out_data[N_TAP * N_FILTER_CHANNEL];
double up_in_data[N_UP_TAP * N_FILTER_CHANNEL];
double up_out_data[N_UP_TAP * N_FILTER_CHANNEL];

// 2.0 Hz
double ff_taps[] = { 0.00134871194836, 0.00269742389671, 0.00134871194836 };
double fb_taps[] = { 1.0, -1.89346414636, 0.898858994155 };
// 0.1 Hz
double up_ff_taps[] = { 3.54360709978e-06, 7.08721419957e-06, 3.54360709978e-06 };
double up_fb_taps[] = { 1.0, -1.99466855305, 0.994682727481 };
/*
// 1.0 Hz
double ff_taps[] = { 0.000346041337639, 0.000692082675278, 0.000346041337639 };
double fb_taps[] = { 1.0, -1.94669754076, 0.948081706107 };
// 0.05 Hz
double up_ff_taps[] = { 8.87081773651e-07, 1.7741635473e-06, 8.87081773651e-07 };
double up_fb_taps[] = { 1.0, -1.99733427181, 0.99733782014 };
*/

// apply IIR filter with coeff from _ff_taps and _fb_taps
// to each of N_FILTER_CHANNEL accel channels
double apply_filter(double _raw, double *_in_data, double *_out_data, int n_tap, double *_ff_taps, double *_fb_taps){
  double filtered = 0;
  byte i;

  // shift taps
  for(i = n_tap - 1; i > 0; i--){
    _in_data[i] = _in_data[i - 1];
  }
  _in_data[0] = _raw;

  // out = ibuff . b - obuff . a
  for(i = 0; i < n_tap; i++){
    filtered += _in_data[i] * _ff_taps[i];
  }
  for(i = 0; i < n_tap - 1; i++){
    filtered -= _out_data[i] * _fb_taps[i + 1];
  }
  // shift taps
  for(i = n_tap - 2; i > 0; i--){
    _out_data[i] = _out_data[i - 1];
  }
  _out_data[i] = filtered;

  return filtered;
}
  
// get raw data form a2d converters
// called by get_cooked
void get_acc_data(double *acc_data){
  Accel.get_Gxyz(acc_data);
}

double dot3(double *v, double* w){
  double out = 0;
  for(int i=0; i<3; i++){
    out += v[i] * w[i];
  }
  return out;
}

double norm(double *v3){
  return sqrt(dot3(v3, v3));
}

void normalize(double *v3){
  double n = norm(v3);
  for(int i=0; i < 3; i++){
    v3[i] /= n;
  }
}

// make call to get_acc_data() and apply IIR filter
void get_cooked(){
  int i, j;
  double acc_data[N_FILTER_CHANNEL];
  
  get_acc_data(acc_data);
  for(i = 0; i < N_FILTER_CHANNEL; i++){
    cooked[i] = apply_filter(acc_data[i], in_data + i * N_TAP, out_data + i * N_TAP, N_TAP, ff_taps, fb_taps);
    up[i] = apply_filter(acc_data[i], up_in_data + i * N_UP_TAP, up_out_data + i * N_UP_TAP, N_UP_TAP, up_ff_taps, up_fb_taps);
  }
  normalize(up);
  back[0] = up[2] * up[0];
  back[1] = up[2] * up[1];
  back[2] = -1 + up[2] * up[2];
  normalize(back);
  braking = dot3(cooked, back);
}

void setup(){
  Serial.begin(115200);
  Serial.println("Buy Open Hardware and own your future!");

  pinMode(INTERRUPT_1, INPUT_PULLUP);
  pinMode(INTERRUPT_2, INPUT_PULLUP);
  
  for(int i=0; i<n_led; i++){
    pinMode(LEDS[i], OUTPUT);
  }

  // configure accel bandwidth (Hz)
  Accel.set_bw(ADXL345_BW_25); // 3 6 12 25 50 100 200 400 800 1600
  // turn off all interrupts while we configure as per datasheet
  for(int bit_i=0; bit_i < 8; bit_i++){
    Accel.setInterrupt(bit_i, 0);
  }
  

  // set up taps
  Accel.setTapThreshold(0x48);
  Accel.setTapDuration(0x1F);
  Accel.setDoubleTapLatency(0x10);
  Accel.setDoubleTapWindow(0xFF);

  // turn on interrupts
  Accel.setTapDetectionOnX(true);
  Accel.setTapDetectionOnY(true);
  Accel.setTapDetectionOnZ(true);
  Accel.setSuppressBit(true);
  Accel.setTimeInactivity(10);
  Accel.setInactivityThreshold(0X10);
  Accel.setInactivityAc(true);
  Accel.setActivityThreshold(0x1F);
  Accel.setActivityX(true);
  Accel.setActivityY(true);
  Accel.setActivityZ(true);
  Accel.setActivityAc(true);

  // taps go to interrupt 0
  Accel.setInterruptMapping(ADXL345_INT_SINGLE_TAP_BIT, 0);
  Accel.setInterruptMapping(ADXL345_INT_DOUBLE_TAP_BIT, 0);

  // activity goes to interrupt 1
  Accel.setInterruptMapping(ADXL345_INT_INACTIVITY_BIT, 0);
  Accel.setInterruptMapping(ADXL345_INT_ACTIVITY_BIT, 0);

  // enable interrupts
  Accel.setInterrupt(ADXL345_INT_DOUBLE_TAP_BIT, 1);
  Accel.setInterrupt(ADXL345_INT_SINGLE_TAP_BIT, 1);
  Accel.setInterrupt(ADXL345_INT_ACTIVITY_BIT, 1);
  // I don't think we want an inactivity interrupt, but can still set thresholds to set power low
  Accel.setInterrupt(ADXL345_INT_INACTIVITY_BIT, 1);

  Accel.setInterruptLevelBit(true); // going low on interrupt 


  // turn on sampling
  Accel.powerOn();
    // attach interrupts
  attachInterrupt(1, interrupt_handler1, LOW); //tap interrupt
  //attachInterrupt(digitalPinToInterrupt(INTERRUPT_2), interrupt_handler1, FALLING);
  // saturate filter memory with current value.  It takes a while for IIR filter to saturate
  for(int i=0; i<200; i++){
    get_cooked();
  }
  // flash leds in sequence with a delay of 10 ms
  sweep(100);
}

// if true, then there is an event that has not yet been handled.
// one bool for each intertupt pin
volatile bool event_pending[2];

void interrupt_handler1(){
  sleep_disable(); //important or we will never wake.
  detachInterrupt(1);
  event_pending[1] = true;
}
void interrupt_handler0(){
  event_pending[0] = true;
}

// flash leds in sequence back and forth
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

// LED toggle states
bool bool0, bool1;
unsigned long acc_last_activity_ms = 0;
bool is_awake = true;

const unsigned long MILLIS = 1;
const unsigned long SECONDS = 1000 * MILLIS;
const unsigned long MINUTES = 60 * SECONDS;
const unsigned long ATTENTION_SPAN = 1 * MINUTES;
const double ACC_ACTIVITY_THRESH = .1;
// loop() counter
int count = 0;
int last_ms = 0;

const int sample_period_ms = 12;
int next_sample_time_ms = 0;

double last_acc_data[3];
void loop(){
  byte source =0;
  if(event_pending[0] || event_pending[1]){
      // this also clears the interrupt on the accel
      source = Accel.getInterruptSource();
      acc_last_activity_ms = millis();
      }
  if(event_pending[0] && event_pending[1]){
    Serial.println("\nBOTH");
    Serial.println(count);
  }
  if(event_pending[1]){
    Serial.println(source, BIN);
    Serial.println(" event1");
    event_pending[1] = false;
    if (source & 1<<ADXL345_INT_SINGLE_TAP_BIT)
      Serial.println("Single Tap");
    if (source & 1<<ADXL345_INT_DOUBLE_TAP_BIT)
      Serial.println("Double Tap");
    if (source & 1<<ADXL345_INT_ACTIVITY_BIT)
      Serial.println("Activity");
    if (source & 1<<ADXL345_INT_INACTIVITY_BIT)
      Serial.println("InActivity");        
    
    //make sure the interrupt is HIGH again
    while(!digitalRead(INTERRUPT_2));
    //reattach the interrupt
    attachInterrupt(1, interrupt_handler1, LOW);
  }
  if(event_pending[0]){
    Serial.println(source, BIN);
    Serial.println(" event0");
    event_pending[0] = false;
  }
  int i;
  double acc_data[3], acc_diff;
  
  int now_ms = millis();
  if(is_awake){
    if(now_ms > next_sample_time_ms){
      count++;
      get_cooked();
      if(count % 100 < 1){
	digitalWrite(LEDS[0], HIGH);
      }
      else{
	digitalWrite(LEDS[0], LOW);
      }
      /*
	for(i = 0; i < N_FILTER_CHANNEL; i++){
	Serial.print(up[i], 8);
	Serial.print(" ");
	Serial.print(cooked[i], 8);
	Serial.print(" ");
	Serial.print(back[i], 8);
	Serial.print(" ");
	}
	Serial.println(braking);
      */
      next_sample_time_ms += sample_period_ms;
      if(count% 100 == 0){
	Serial.println(now_ms - last_ms);
	last_ms = now_ms;
      }
    }
    // Serial.println(braking);
    if(braking > braking_threshold){
      acc_last_activity_ms = millis();
      for(i=1; i<n_led; i++){
	digitalWrite(LEDS[i], HIGH);
      }
    }
    else{
      for(i=1; i<n_led; i++){
	digitalWrite(LEDS[i], LOW);
      }
    }
    if(millis() - acc_last_activity_ms > ATTENTION_SPAN){
      go_to_sleep();
    }
  }
}

void go_to_sleep(){
  for(int i=0; i < n_led; i++){
    digitalWrite(LEDS[i], LOW);
  }
  is_awake = false;
  Serial.println("SLEEP!");
  sleep_enable();
  attachInterrupt(1, interrupt_handler1, LOW);

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  cli();
  sleep_bod_disable();
  sei();
  sleep_cpu();
  /* wake up here */
  sleep_disable();
  next_sample_time_ms = millis();
  is_awake = true;
  Serial.println("AWAKE!");
}

void wake_up(){

}

