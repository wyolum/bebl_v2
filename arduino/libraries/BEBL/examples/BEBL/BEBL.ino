/*
  Test out the LEDs and accel of BEBL
 */

#include "Wire.h"
#include "ADXL345.h"

ADXL345 Accel;
// int LEDS[] = {5, 6, 7, 9, 10, 13}; // top to bottom
const int LEDS[] = {7, 9, 6, 10, 5, 13}; // top to bottom

const int n_led = 3;

const int INTERRUPT_1 = 2;
const int INTERRUPT_2 = 3;
const int N_FILTER_CHANNEL = 3;
const int N_TAP = 3;
const int N_UP_TAP = 3;

// variable for filtered accel data
double cooked[N_FILTER_CHANNEL];

// variable for filtered up data
double up[N_FILTER_CHANNEL];
double back[N_FILTER_CHANNEL];
double breaking = 0;
double breaking_threshold = 0.06;

double in_data[N_TAP * N_FILTER_CHANNEL];
double out_data[N_TAP * N_FILTER_CHANNEL];
double up_in_data[N_UP_TAP * N_FILTER_CHANNEL];
double up_out_data[N_UP_TAP * N_FILTER_CHANNEL];

// feed forward
double ff_taps[] = { 0.0078202080335, 0.015640416067, 0.0078202080335 };

// feed back
double fb_taps[] = { 1.0, -1.73472576881, 0.766006600943 };

// Slower filter for up traking, BW=0.050000
// feed forward
double up_ff_taps[] = { 0.000346041337639, 0.000692082675278, 0.000346041337639 };
// feed back
double up_fb_taps[] = { 1.0, -1.94669754076, 0.948081706107 };

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
  back[0] = up[2] * up[0];
  back[1] = up[2] * up[1];
  back[2] = -1 + up[2] * up[2];
  normalize(back);
  breaking = dot3(cooked, back);
  /*
  for(i = 0; i < N_FILTER_CHANNEL; i++){
    Serial.print(breaking, 8);
    Serial.print(" ");
    Serial.print(cooked[i], 8);
    Serial.print(" ");
    Serial.print(back[i], 8);
    Serial.print(" ");
  }
  Serial.println();
  */
}

void setup(){
  Serial.begin(115200);
  Serial.println("Buy Open Hardware and own your future!");

  pinMode(INTERRUPT_1, INPUT);
  pinMode(INTERRUPT_2, INPUT);
  
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
  Accel.setTapThreshold(0x28);
  Accel.setTapDuration(0xFF);
  Accel.setDoubleTapLatency(0x50);
  Accel.setDoubleTapWindow(0xFF);

  // turn on interrupts
  Accel.setTapDetectionOnX(true);
  Accel.setTapDetectionOnY(true);
  Accel.setTapDetectionOnZ(true);
  Accel.setSuppressBit(true);
  Accel.setTimeInactivity(10);
  Accel.setInactivityThreshold(0X02);
  Accel.setActivityX(true);
  Accel.setActivityY(true);
  Accel.setActivityZ(true);
  Accel.setActivityAc(true);

  // taps go to interrupt 0
  Accel.setInterruptMapping(ADXL345_INT_SINGLE_TAP_BIT, 0);
  Accel.setInterruptMapping(ADXL345_INT_DOUBLE_TAP_BIT, 0);

  // activity goes to interrupt 1
  Accel.setInterruptMapping(ADXL345_INT_INACTIVITY_BIT, 1);
  Accel.setInterruptMapping(ADXL345_INT_ACTIVITY_BIT, 1);

  // enable interrupts
  Accel.setInterrupt(ADXL345_INT_DOUBLE_TAP_BIT, 1);
  Accel.setInterrupt(ADXL345_INT_SINGLE_TAP_BIT, 1);
  Accel.setInterrupt(ADXL345_INT_ACTIVITY_BIT, 1);
  Accel.setInterrupt(ADXL345_INT_INACTIVITY_BIT, 1);

  Accel.setInterruptLevelBit(false); // Rising edge on interrupt
  
  // attach interrupts
  attachInterrupt(0, interrupt_handler0, RISING);
  attachInterrupt(1, interrupt_handler1, RISING);

  // tunr on sampling
  Accel.powerOn();

  // flash leds in sequence with a delay of 10 ms
  sweep(100);
}

// if true, then there is an event that has not yet been handled.
// one bool for each intertupt pin
volatile bool event_pending[2];
void interrupt_handler0(){
  event_pending[0] = true;
}
void interrupt_handler1(){
  event_pending[1] = true;
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
unsigned long acc_last_activity = 0;
bool is_awake = true;

const unsigned long MILLIS = 1;
const unsigned long SECONDS = 1000 * MILLIS;
const unsigned long MINUTES = 60 * SECONDS;
const unsigned long ATTENTION_SPAN = 5 * MINUTES;
const double ACC_ACTIVITY_THRESH = .1;
// loop() counter
int count = 0;
int last_ms = 0;

const int sample_period_ms = 12;
int next_sample_time_ms = 0;

double last_acc_data[3];
void loop(){
  byte source = Accel.getInterruptSource();
  if(event_pending[0] && event_pending[1]){
    Serial.println("\nBOTH");
    Serial.println(count);
  }
  if(event_pending[0]){
    Serial.println(source, BIN);
    Serial.println(" event0");
    event_pending[0] = false;
    //bool0 = !bool0;
    //digitalWrite(LEDS[0], bool0);
    wake_up();
  }
  if(event_pending[1]){
    Serial.println(source, BIN);
    Serial.println(" event1");
    event_pending[1] = false;
    //bool1 = !bool1;
    //digitalWrite(LEDS[n_led-1], bool1);
    wake_up();
  }
  int i;
  double acc_data[3], acc_diff;
  
  int now_ms = millis();
  if(now_ms > next_sample_time_ms){
    count++;
    get_cooked();
    next_sample_time_ms += sample_period_ms;
    if(count% 100 == 0){
      Serial.println(now_ms - last_ms);
      last_ms = now_ms;
    }
  }
  if(is_awake){
    Serial.println(breaking);
    if(breaking > breaking_threshold){
      for(i=0; i<n_led; i++){
	digitalWrite(LEDS[i], HIGH);
      }
    }
    else{
      for(i=0; i<n_led; i++){
	digitalWrite(LEDS[i], LOW);
      }
    }
    if(millis() - acc_last_activity > ATTENTION_SPAN){
      go_to_sleep();
    }
  }
}

void go_to_sleep(){
  Serial.println("SLEEP!");
  is_awake = false;
}

void wake_up(){
  is_awake = true;
}

