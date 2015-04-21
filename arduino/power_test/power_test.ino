/*
  Test out the LEDs and accel of BEBL
 */

#include "Wire.h"
#include "ADXL345.h"
//KGO_BEGIN
//atmega sleep mode includes
#include <avr/sleep.h> //Needed for sleep_mode
#include <avr/power.h> //Needed for powering down perihperals such as the ADC/TWI and Timers
long lastWakeTime;
// how many millis before we go to sleep
#define INACTIVITY 3000
//KGO_END

ADXL345 Accel;
int LEDS[] = {5, 6, 7, 9, 10, 13};
int n_led = 6;

int INTERRUPT_1 = 2;
int INTERRUPT_2 = 3;

bool event_pending[2];
void AccelSetup();
void setup(){
  pinMode(INTERRUPT_1, INPUT);
  pinMode(INTERRUPT_2, INPUT);
  
  Serial.begin(115200);
  Serial.println("Buy Open Hardware and own your future!");
  for(int i=0; i<n_led; i++){
    pinMode(LEDS[i], OUTPUT);
  }
  AccelSetup();
  sweep(10);
  lastWakeTime = millis();
}

// this also gets called when waking from sleep. we should probably figure out
// what's needed only on wakeup
void AccelSetup(){
  
  // configure accel
  Accel.set_bw(ADXL345_BW_3); // 3 6 12 25 50 100 200 400 800 1600
  //Accel.set_bw(ADXL345_BW_1600);
  Accel.setInactivityAc(true);

  // turn off interrupts
  for(int bit_i=0; bit_i < 8; bit_i++){
    Accel.setInterrupt(bit_i, 0);
  }

  Accel.setTapThreshold(0x28);
  Serial.println(Accel.getTapThreshold(), HEX);
  Accel.setTapDuration(0xFF);
  Serial.println(Accel.getTapDuration(), HEX);
  Accel.setDoubleTapLatency(0x50);
  Serial.println(Accel.getDoubleTapLatency(), HEX);
  Accel.setDoubleTapWindow(0xFF);
  Serial.println(Accel.getDoubleTapWindow(), HEX);

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

  Accel.setInterruptLevelBit(false); // Rising edge on interrupt

// This sets the ADX345 to send taps to INT1 pin which is connected to Pin 3 on arduino
  Accel.setInterruptMapping(ADXL345_INT_SINGLE_TAP_BIT, 0);
  Accel.setInterruptMapping(ADXL345_INT_DOUBLE_TAP_BIT, 0);
  
// This sets the ADX345 to send inactivity to INT2 pin which is connected to Pin 2 on arduino
  Accel.setInterruptMapping(ADXL345_INT_INACTIVITY_BIT, 1);
  Accel.setInterruptMapping(ADXL345_INT_ACTIVITY_BIT, 1);
  

  Accel.setInterrupt(ADXL345_INT_DOUBLE_TAP_BIT, 1);
  Accel.setInterrupt(ADXL345_INT_SINGLE_TAP_BIT, 1);
  
  Accel.setInterrupt(ADXL345_INT_ACTIVITY_BIT, 1);
  Accel.setInterrupt(ADXL345_INT_INACTIVITY_BIT, 1);
 

  attachInterrupt(0, interrupt_handler0, RISING);
  attachInterrupt(1, interrupt_handler1, RISING);
  Accel.powerOn();
}
void interrupt_handler0(){
  event_pending[0] = true;
}
void interrupt_handler1(){
  event_pending[1] = true;
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
bool bool0, bool1;
int count = 0;
void loop(){
  count++;
  byte tmpbyte;
  byte source = Accel.getInterruptSource();
  if(event_pending[0] && event_pending[1]){
    Serial.println("\nBOTH");
    Serial.println(count);
  }
  if(event_pending[0]){
    Serial.println(source, BIN);
    Serial.println(" event0");
    event_pending[0] = false;
    bool0 = !bool0;
      // first turn off all the LEDs
    for(int i=0; i<n_led - 1; i++){
      digitalWrite(LEDS[i],HIGH);
      delay(200);
      digitalWrite(LEDS[i], LOW);
      delay(100);
    }
    goToSleep();
  }
  if(event_pending[1]){
    Serial.println(source, BIN);
    Serial.println(" event1");
    event_pending[1] = false;
    bool1 = !bool1;
    digitalWrite(LEDS[n_led-1], bool1);
  }

  return;
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
    
    /*
    if(acc_data[2] < 0){
      int led_i = (int)(-n_led * acc_data[2]);
      for(i=0; i<n_led; i++){
	digitalWrite(LEDS[i], LOW);
      }
      digitalWrite(LEDS[led_i], HIGH);
    }
    */
    for(i=1; i<n_led - 1; i++){
      if(acc_data[2] < -.1 * (i)){
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
// KGO_BEGIN
// Atmega sleep/wake routines

void goToSleep(){ 
  Serial.println("Going to Sleep");
  detachInterrupt(0);// we don't want to keep getting interrupted when waking

  delay(100);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  //attachInterrupt(0,wake,RISING); // pin 2 intterupt Inactivity
  attachInterrupt(1,wake,RISING);// pin 3 interrupt  tap
  power_twi_disable();
//  power_spi_disable();
  power_usart0_disable();
//  power_timer0_disable(); //Needed for delay_ms and apparently for pin interrupts
  power_timer1_disable();
  power_timer2_disable(); 
  sleep_mode(); // this immediately goes to sleep
  // and when we wake up, we will execute this
  detachInterrupt(0);// we don't want to keep getting interrupted when waking
  detachInterrupt(1);

  sleep_disable();
  power_twi_enable();
//  power_spi_enable();
  power_usart0_enable();
//  power_timer0_enable(); //Needed for delay_ms
  power_timer1_enable();
  power_timer2_enable(); 
  Serial.println("waking up");
  AccelSetup();

  //Serial.println("returning from sleep");
  lastWakeTime = millis();


}
void wake()
{
  // apparently the uart takes a while to reconfigure on waking up
   Serial.begin(115200);
  delay(100); // crude debounce
  Serial.println("Waking Up");
}
//KGO_END
