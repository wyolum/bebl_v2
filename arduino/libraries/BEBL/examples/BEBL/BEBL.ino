/*
  Test out the LEDs and accel of BEBL
 */

#include "Wire.h"
#include "ADXL345.h"
#include <avr/sleep.h>
#include <EEPROM.h>

ADXL345 Accel;
const int LEDS[] = {5, 6, 7, 9, 10, 13}; // top to bottom
const int MODE_BUTTON = 4;
// const int LEDS[] = {7, 9, 6, 10, 5, 13}; // middle to outside

const int n_led = 6;

const int INTERRUPT_1 = 3;
const int INTERRUPT_2 = 2;
const int N_FILTER_CHANNEL = 3;
const int N_TAP = 3;
const int N_UP_TAP = 3;

// LED toggle states
bool bool0, bool1;
unsigned long acc_last_activity_ms = 0;
bool is_awake = true;

const unsigned long MILLIS = 1;
const unsigned long SECONDS = 1000 * MILLIS;
const unsigned long MINUTES = 60 * SECONDS;
const unsigned long ATTENTION_SPAN = 5 * MINUTES;
const double ACC_ACTIVITY_THRESH = .05;

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

// loop() counter
int count = 0;
int last_ms = 0;

const int sample_period_ms = 12;
int next_sample_time_ms = 0;

double last_acc_data[3];
volatile bool event_pending[2];

union converter_t {
  double dbl_dat; 
  uint8_t byte_dat[sizeof(double)];
};
converter_t converter;

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
void get_acc_data(double *acc_data, bool count_flag){
  if(count_flag){
    int xyz[N_FILTER_CHANNEL];
    Accel.readAccel(xyz);
    for(int i=0; i<N_FILTER_CHANNEL; i++){
      acc_data[i] = xyz[i];
    }
  }
  else{
    Accel.get_Gxyz(acc_data);
  }
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

// make call to get_acc_data() and apply IIR filter.  If count_flag is true, filter integer counts without converting to G units
void get_cooked(bool count_flag){
  int i, j;
  double acc_data[N_FILTER_CHANNEL];
  
  get_acc_data(acc_data, count_flag);
  for(i = 0; i < N_FILTER_CHANNEL; i++){
    cooked[i] = apply_filter(acc_data[i], in_data + i * N_TAP, out_data + i * N_TAP, N_TAP, ff_taps, fb_taps);
    up[i] = apply_filter(acc_data[i], up_in_data + i * N_UP_TAP, up_out_data + i * N_UP_TAP, N_UP_TAP, up_ff_taps, up_fb_taps);
    if(abs(cooked[i] - up[i]) > ACC_ACTIVITY_THRESH){
      // Serial.println("Activity");
      acc_last_activity_ms = millis();
    }
  }
  if(! count_flag){
    normalize(up);
  }
  back[0] = up[2] * up[0];
  back[1] = up[2] * up[1];
  back[2] = -1 + up[2] * up[2];
  normalize(back);
  braking = dot3(cooked, back);
}

void eeprom_write_dbl(unsigned int address, double dat){
  converter.dbl_dat = dat;
  for(int i=0; i<sizeof(double); i++){
    EEPROM.write(address + i, converter.byte_dat[i]);
  }
}

double eeprom_read_dbl(unsigned int address){
  for(int i=0; i<sizeof(double); i++){
    converter.byte_dat[i] = EEPROM.read(address + i);
  }
  return converter.dbl_dat;
}

void calibrate(){
  double mins[N_FILTER_CHANNEL] = {0, 0, 0};
  double maxes[N_FILTER_CHANNEL] = {0, 0, 0};
  bool min_states[N_FILTER_CHANNEL] = {false, false, false};
  bool max_states[N_FILTER_CHANNEL] = {false, false, false};

  bool print_out = false;
  bool single_tap, double_tap;

  next_sample_time_ms = millis() + sample_period_ms;
  saturate_filters(true); // fill digital filters with counts
  while(1){ // cal till reset occurs
    while(millis() < next_sample_time_ms){
      // wait for next sample time
    }
    handle_events(single_tap, double_tap);
    if(double_tap){
      break;
    }
    get_cooked(true); // filter raw counts (not in G-units)
    next_sample_time_ms += sample_period_ms;
    print_out = false;
    for(int i=0; i<N_FILTER_CHANNEL; i++){
      if(cooked[i] < mins[i]){
	mins[i] = cooked[i];
	min_states[i] = !min_states[i];
	digitalWrite(LEDS[i], min_states[i]);
	print_out = true;
      }
      if(cooked[i] > maxes[i]){
	maxes[i] = cooked[i];
	max_states[i] = !max_states[i];
	digitalWrite(LEDS[i + N_FILTER_CHANNEL], max_states[i]);
	print_out = true;
      }
    }
    if(print_out){
      for(int i=0; i<N_FILTER_CHANNEL; i++){
	Serial.print(mins[i]);
	Serial.print(" ");
	Serial.print(maxes[i]);
	Serial.print(" ");
      }
      Serial.println("");
    }
  }
  for(int i=0; i < n_led; i++){
    digitalWrite(LEDS[i], LOW);
  }
  Serial.println("cal complete");
  int eeaddress = 0;
  for(int i=0; i < N_FILTER_CHANNEL; i++){
    eeaddress += sizeof(double);
    eeprom_write_dbl(eeaddress, mins[i]);
    eeaddress += sizeof(double);
    eeprom_write_dbl(eeaddress, maxes[i]);
  }
}

void set_cal_from_eeprom(){
  double mins[N_FILTER_CHANNEL];
  double maxes[N_FILTER_CHANNEL];
  int eeaddress = 0;
  double low = 200;
  double high = 300;
    
  bool load_cal = true; // only load reasonable values;
  for(int i=0; i < N_FILTER_CHANNEL; i++){
    eeaddress += sizeof(double);
    mins[i] = eeprom_read_dbl(eeaddress);
    if(mins[i] > -low || mins[i] < -high){
      load_cal = false;
    }
    eeaddress += sizeof(double);
    maxes[i] = eeprom_read_dbl(eeaddress);
    if(maxes[i] < low || maxes[i] > high){
      load_cal = false;
    }
  }
  if(load_cal){
    Accel.setCal(mins, maxes);
  }
  else{
    Serial.println("recal required");
  }
}

void setup(){
  Serial.begin(115200);
  Serial.println("Buy Open Hardware and own your future!");

  pinMode(INTERRUPT_1, INPUT_PULLUP);
  pinMode(INTERRUPT_2, INPUT_PULLUP);
  
  for(int i=0; i<n_led; i++){
    pinMode(LEDS[i], OUTPUT);
  }
  pinMode(MODE_BUTTON, INPUT);

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
  Accel.setInterrupt(ADXL345_INT_INACTIVITY_BIT, 1);

  // KEVIN: I don't think we want an inactivity interrupt, but can still set thresholds to set power low 
  Accel.setInterruptLevelBit(true); // going low on interrupt
  
  // turn on sampling
  Accel.powerOn();

  // attach interrupts
  attachInterrupt(1, interrupt_handler1, LOW); //tap interrupt
  if(digitalRead(MODE_BUTTON) == LOW){
    calibrate();
    int eeaddress = 0;
    Serial.println("calibration values: ");
    for(int i=0; i < N_FILTER_CHANNEL; i++){
      eeaddress += sizeof(double);
      Serial.print(eeprom_read_dbl(eeaddress));
      Serial.print(" ");
      eeaddress += sizeof(double);
      Serial.print(eeprom_read_dbl(eeaddress));
      Serial.println(" ");
    }
    Serial.println("");
  }
  set_cal_from_eeprom();
  saturate_filters(false); // fill digital filters with Gs
  // flash leds in sequence with a delay of 10 ms
  sweep(50);
  last_ms = millis();
  next_sample_time_ms = last_ms + sample_period_ms;
}

void saturate_filters(bool count_flag){
  // saturate filter current value.  It takes a while for IIR filter to saturate
  for(int i=0; i<200; i++){
    get_cooked(count_flag);
  }
}

// if true, then there is an event that has not yet been handled.
// one bool for each intertupt pin
void interrupt_handler0(){
  event_pending[0] = true;
}
void interrupt_handler1(){
  sleep_disable(); //important or we will never wake.
  detachInterrupt(1);
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
  for(i=1; i<n_led; i++){
    digitalWrite(LEDS[n_led - i - 1], HIGH);
    delay(d);
    digitalWrite(LEDS[n_led - i - 1], LOW);
  }
}

void handle_events(bool& single_tap, bool& double_tap){
  byte source = 0;
  single_tap = double_tap = false;

  if(event_pending[0] || event_pending[1]){
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
    if (source & 1<<ADXL345_INT_SINGLE_TAP_BIT){
      single_tap = true;
      Serial.println("Single Tap");
    }
    if (source & 1<<ADXL345_INT_DOUBLE_TAP_BIT){
      Serial.println("Double Tap");
      double_tap = true;
    }
    if (source & 1<<ADXL345_INT_ACTIVITY_BIT){
      Serial.println("Activity");
    }
    if (source & 1<<ADXL345_INT_INACTIVITY_BIT){
      Serial.println("InActivity");        
    }

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
}

void loop(){
  int i;
  double acc_data[3], acc_diff;
  bool single_tap, double_tap;
  handle_events(single_tap, double_tap);

  digitalWrite(LEDS[0], millis() % 1000 < 5);

  int now_ms = millis();
  if(is_awake){
    if(now_ms > next_sample_time_ms){
      count++;
      get_cooked(false);
      next_sample_time_ms += sample_period_ms;
      if(count % 1000 == 0){
	Serial.println((now_ms - last_ms)/1000.);
	last_ms = now_ms;
      }
    }
    // Serial.println(braking);
    if(braking > braking_threshold){
      acc_last_activity_ms = millis();
      for(i=0; i<n_led; i++){
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

void power_down(){
  Serial.println("SLEEP!");
  for(int i=0; i < n_led; i++){
    digitalWrite(LEDS[i], LOW);
  }
  is_awake = false;
}

void power_up(){
  next_sample_time_ms = millis();
  is_awake = true;
  Serial.println("AWAKE!");
}

void go_to_sleep(){
  power_down();

  sleep_enable();
  attachInterrupt(1, interrupt_handler1, LOW);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  cli();
  sleep_bod_disable();
  sei();
  sleep_cpu();

  /* wake up here */
  sleep_disable();
  power_up();
}


