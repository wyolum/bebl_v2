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
const int N_DOWN_TAP = 3;

// variable for filtered accel data
double volatile cooked[N_FILTER_CHANNEL];

// variable for filtered down data
double volatile down[N_FILTER_CHANNEL];

double in_data[N_TAP * N_FILTER_CHANNEL];
double out_data[N_TAP * N_FILTER_CHANNEL];
double down_in_data[N_DOWN_TAP * N_FILTER_CHANNEL];
double down_out_data[N_DOWN_TAP * N_FILTER_CHANNEL];

// feed forward
double ff_taps[] = { 0.0078202080335, 0.015640416067, 0.0078202080335 };

// feed back
double fb_taps[] = { 1.0, -1.73472576881, 0.766006600943 };

// Slower filter for down traking, BW=0.050000
// feed forward
double down_ff_taps[] = { 5.62480979076e-05, 0.000112496195815, 5.62480979076e-05 };
// feed back
double down_fb_taps[] = { 1.0, -1.97867495733, 0.978899949723 };

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

// make call to get_acc_data() and apply IIR filter
void get_cooked(){
  int i, j;
  double acc_data[N_FILTER_CHANNEL];
  
  get_acc_data(acc_data);
  for(i = 0; i < N_FILTER_CHANNEL; i++){
    cooked[i] = apply_filter(acc_data[i], in_data + i * N_TAP, out_data + i * N_TAP, N_TAP, ff_taps, fb_taps);
    down[i] = apply_filter(acc_data[i], down_in_data + i * N_DOWN_TAP, down_out_data + i * N_DOWN_TAP, N_DOWN_TAP, down_ff_taps, down_fb_taps);
  }
  for(i = 0; i < N_FILTER_CHANNEL; i++){
    Serial.print(acc_data[i], 8);
    Serial.print(" ");
    Serial.print(cooked[i], 8);
    Serial.print(" ");
    Serial.print(down[i], 8);
    Serial.print(" ");
    /*
    Serial.print(":");
    for(j = 0; j < N_TAP; j++){
      Serial.print(in_data[i * N_TAP + j]);
      Serial.print(" ");
    }
    Serial.println();
    */
  }
  Serial.println();
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
  sweep(10);
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
int last_millis = 0;

const int sample_period_ms = 12;
int next_sample_time_ms = 0;

double last_acc_data[3];
void loop(){
  count++;
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
  
  if(millis() > next_sample_time_ms){
    get_cooked();
    next_sample_time_ms += sample_period_ms;
  }
  if(count% 1000 == 0){
    if(is_awake){
      if(Accel.status){
	float length = 0.;
	for(i = 0; i < 3; i++){
	  length += (float)cooked[i] * (float)cooked[i];
	  acc_diff = cooked[i] - last_acc_data[i];
	  if(abs(acc_diff) > ACC_ACTIVITY_THRESH){
	    acc_last_activity = millis();
	    Serial.println("ACTIVE");
	  }
	  last_acc_data[i] = cooked[i];
	  // Serial.print(cooked[i]);
	  // Serial.print(" ");
	}
	// length = sqrt(length);
	// Serial.print(length);
	// Serial.print(" ");
	// Serial.print(acc_last_activity);
	// Serial.println("");
    
	/*
	  if(acc_data[2] < 0){
	  int led_i = (int)(-n_led * acc_data[2]);
	  for(i=0; i<n_led; i++){
	  digitalWrite(LEDS[i], LOW);
	  }
	  digitalWrite(LEDS[led_i], HIGH);
	  }
	*/
	for(i=0; i<n_led; i++){
	  if(acc_data[2] < -.025 * (i - i % 2) - .025){
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

    if(millis() - acc_last_activity > ATTENTION_SPAN){
      go_to_sleep();
    }
  }
}

void go_to_sleep(){
  is_awake = false;
}

void wake_up(){
  is_awake = true;
}

