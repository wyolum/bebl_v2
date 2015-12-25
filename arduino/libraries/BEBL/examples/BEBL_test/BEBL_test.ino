/*
  Test out the LEDs and accel of BEBL
 */

#include "Wire.h"
#include "ADXL345.h"
ADXL345 Accel;
// int LEDS[] = {5, 6, 7, 9, 10, 13}; // top to bottom
int LEDS[] = {7, 9, 6, 10, 5, 13}; // top to bottom

int n_led = 3;

int INTERRUPT_1 = 2;
int INTERRUPT_2 = 3;

// low pass filter acc_data in place
void filter(){
  
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
  Accel.set_bw(ADXL345_BW_3); // 3 6 12 25 50 100 200 400 800 1600

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
  if(is_awake){
    Accel.get_Gxyz(acc_data);

    if(Accel.status){
      float length = 0.;
      for(i = 0; i < 3; i++){
	length += (float)acc_data[i] * (float)acc_data[i];
	acc_diff = acc_data[i] - last_acc_data[i];
	if(abs(acc_diff) > ACC_ACTIVITY_THRESH){
	  acc_last_activity = millis();
	  Serial.println("ACTIVE");
	}
	last_acc_data[i] = acc_data[i];
	// Serial.print(acc_data[i]);
	// Serial.print(" ");
      }
      length = sqrt(length);
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

void go_to_sleep(){
  is_awake = false;
}

void wake_up(){
  is_awake = true;
}
