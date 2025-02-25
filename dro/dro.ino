/*
  ArduinoDRO + Tach V5.12

*/

#include "BluetoothSerial.h"
#include <driver/pcnt.h>


// I/O ports config (change pin numbers if DRO, Tach sensor or Tach LED feedback is connected to different ports)
#define SCALE_CLK_PIN 2
#define REPORT_PIN  26

#define SCALE_X_PIN 21   // pin 3 was causing instability - some internal pull-up as this is a uart pin as well. Moving it to pin 21 as its safe and next to pin 3
#define SCALE_Y_PIN 4
#define SCALE_Z_PIN 5
#define TACH_PIN    14  // GPIO where tach sensor is connected

// General Settings
#define UART_BAUD_RATE 9600       //  Set this so it matches the BT module's BAUD rate 
#define UPDATE_FREQUENCY 25       //  Frequency in Hz (number of timer per second the scales are read and the data is sent to the application)

#define MAX_UPDATE_FREQUENCY 21
#define TIMER1_ALARM_TICKS 25
#define TIMER2_ALARM_TICKS 111

// Tachometer settings
#define TACH_PIN 14         // GPIO where tach sensor is connected
#define PCNT_UNIT PCNT_UNIT_0
#define PULSES_PER_REV 16    // Adjust based on number of pulses per revolution

//---END OF CONFIGURATION PARAMETERS ---


//---DO NOT CHANGE THE CODE BELOW UNLESS YOU KNOW WHAT YOU ARE DOING ---

/* iGaging Clock Settings (do not change) */
#define SCALE_CLK_PULSES 21       //iGaging and Accuremote scales use 21 bit format
#define SCALE_CLK_FREQUENCY 9000    //iGaging scales run at about 9-10KHz
#define SCALE_CLK_DUTY 20       // iGaging scales clock run at 20% PWM duty (22us = ON out of 111us cycle)

// DRO rounding sample size.  Change it to 16 for machines with power feed
#define AXIS_AVERAGE_COUNT 24

/* weighted average constants */ 
#define FILTER_SLOW_EMA AXIS_AVERAGE_COUNT  // Slow movement EMA
#define FILTER_FAST_EMA 2           // Fast movement EMA

BluetoothSerial SerialBT;   //pgt

// Variables
int16_t pulseCount = 0;
volatile unsigned long lastUpdateTime = 0;

// Some constants calculated here
long const longMax = __LONG_MAX__;
long const longMin = (- __LONG_MAX__ - (long) 1);
long const slowSc = ((long) 2000) / (((long) FILTER_SLOW_EMA) + ((long) 1));
long const fastSc = ((long) 20) / (((long) FILTER_FAST_EMA) + ((long) 1));

hw_timer_t * timer2 = NULL;
hw_timer_t * timer1 = NULL;
volatile byte state = LOW;

int const updateFrequencyCounterLimit = (int) (((unsigned long) SCALE_CLK_FREQUENCY) / ((unsigned long) UPDATE_FREQUENCY)); // = 9000/24
int const clockCounterLimit = (int) (((unsigned long) (F_CPU / 8)) / (unsigned long) SCALE_CLK_FREQUENCY) - 10;
int const scaleClockDutyLimit = (int) (((unsigned long) (F_CPU / 800)) * ((unsigned long) SCALE_CLK_DUTY) / (unsigned long) SCALE_CLK_FREQUENCY);

//variables that will store the DRO readout
volatile boolean tickTimerFlag;
volatile int updateFrequencyCounter;
volatile int tachTickCounter;

// Axis count
volatile long xValue;
volatile long xReportedValue;
volatile long yValue;
volatile long yReportedValue;
volatile long zValue;
volatile long zReportedValue;

volatile long axisLastReadX[AXIS_AVERAGE_COUNT];
volatile int axisLastReadPositionX;
volatile long axisAMAValueX;

volatile long axisLastReadY[AXIS_AVERAGE_COUNT];
volatile int axisLastReadPositionY;
volatile long axisAMAValueY;

volatile long axisLastReadZ[AXIS_AVERAGE_COUNT];
volatile int axisLastReadPositionZ;
volatile long axisAMAValueZ;

//The setup function is called once at startup of the sketch
void setup()
{
 // cli();
  tickTimerFlag = false;
  updateFrequencyCounter = 0;
  tachTickCounter = 0;

 // SerialBT.begin(115200); //Bluetooth device name
  SerialBT.begin("ESP32test");

  // Initialize DRO values
  initializeAxisAverage(axisLastReadX, axisLastReadPositionX, axisAMAValueX);
  initializeAxisAverage(axisLastReadY, axisLastReadPositionY, axisAMAValueY);  
  initializeAxisAverage(axisLastReadZ, axisLastReadPositionZ, axisAMAValueZ);
  
  // clock pin should be set as output
  pinMode(SCALE_CLK_PIN, OUTPUT);
  pinMode(REPORT_PIN, OUTPUT);

  //data pins should be set as inputs
  pinMode(SCALE_X_PIN, INPUT);
  xValue = 0L;
  xReportedValue = 0L;
  pinMode(SCALE_Y_PIN, INPUT);
  yValue = 0L;
  yReportedValue = 0L;
  pinMode(SCALE_Z_PIN, INPUT_PULLDOWN);
  zValue = 0L;
  zReportedValue = 0L;

  // Initialize the pulse counter
  pinMode(TACH_PIN, INPUT);
  pcnt_config_t pcnt_config = {
    .pulse_gpio_num = TACH_PIN,     // GPIO for pulse input
    .ctrl_gpio_num = PCNT_PIN_NOT_USED, // No control pin
    .lctrl_mode = PCNT_MODE_KEEP,   // Control signal low - Keep mode
    .hctrl_mode = PCNT_MODE_KEEP,   // Control signal high - Keep mode
    .pos_mode = PCNT_COUNT_INC,     // Count on positive edge
    .neg_mode = PCNT_COUNT_DIS,     // Do not count on negative edge
    .counter_h_lim = 32767,         // Upper limit
    .counter_l_lim = -32768,        // Lower limit
    .unit = PCNT_UNIT,              // PCNT Unit
    .channel = PCNT_CHANNEL_0       // PCNT Channel
};


  pcnt_unit_config(&pcnt_config);
  // Initialize PCNT's counter
  pcnt_counter_pause(PCNT_UNIT);
  pcnt_counter_clear(PCNT_UNIT);
  // Resume counting
  pcnt_counter_resume(PCNT_UNIT);

  //initialize timers
  setupClkTimer(); 
  sei();
}

// The loop function is called in an endless loop
void loop()
{
  if (tickTimerFlag) {
    digitalWrite(REPORT_PIN, 1); // pgt

    tickTimerFlag = false;
    
    scaleValueRounded(xReportedValue, axisLastReadX, axisLastReadPositionX, axisAMAValueX);
    //print DRO positions to the SerialBT port
    SerialBT.print(F("X"));
    SerialBT.print((long)xReportedValue);
    SerialBT.print(F(";"));
    
    scaleValueRounded(yReportedValue, axisLastReadY, axisLastReadPositionY, axisAMAValueY); 
    SerialBT.print(F("Y"));
    SerialBT.print((long)yReportedValue);
    SerialBT.print(F(";"));

    scaleValueRounded(zReportedValue, axisLastReadZ, axisLastReadPositionZ, axisAMAValueZ); 
    SerialBT.print(F("Z"));
    SerialBT.print((long)zReportedValue);
    SerialBT.print(F(";"));

		// print Tach rpm to serial port
    // ie *60 for RPM. *4 as count is over 250mS. Trigger wheel increases ticks per rev
    SerialBT.print(F(";"));
    SerialBT.print(F("T"));
    SerialBT.print((long)(pulseCount * 60 * 4  / PULSES_PER_REV )); 

    digitalWrite(REPORT_PIN, 0); // pgt
  }
}


// Timer 2 interrupt B ( Switches clock pin from low to high 21 times) at the end of clock counter limit
void IRAM_ATTR onTimer2() {
  // Only set the clock high if updateFrequencyCounter less than 21
  if (updateFrequencyCounter < 21) {
    digitalWrite(SCALE_CLK_PIN, 1); // pgt
  }
}


// Timer 2 interrupt A ( Switches clock pin from high to low) at the end of clock PWM Duty counter limit
void IRAM_ATTR onTimer1() {
  // Control the scale clock for only first 21 loops
  if (updateFrequencyCounter < 21) {
    // Set clock low if high and then delay 2us
    if (digitalRead(SCALE_CLK_PIN)) {
      digitalWrite(SCALE_CLK_PIN, 0);
      return;
    }
    timerAlarmEnable(timer2);

    // read the pin state and shift it into the appropriate variables
    // Logic by Les Jones:
    //  If data pin is HIGH set bit 20th of the axis value to '1'.  Then shift axis value one bit to the right
    //  This is called 20 times (for bits received from 0 to 19)
    if (updateFrequencyCounter < SCALE_CLK_PULSES - 1) {

      if (digitalRead(SCALE_X_PIN))
        xValue |= ((long)0x00100000 );
      xValue >>= 1;

      if (digitalRead(SCALE_Y_PIN))
        yValue |= ((long)0x00100000 );
      yValue >>= 1;

      if (digitalRead(SCALE_Z_PIN))
        zValue |= ((long)0x00100000 );
      zValue >>= 1;

    } else if (updateFrequencyCounter == SCALE_CLK_PULSES - 1) {

      //If 21-st bit is 'HIGH' inverse the sign of the axis readout
      if (digitalRead(SCALE_X_PIN))
        xValue |= ((long)0xfff00000);
      xReportedValue = xValue;
      xValue = 0L;
      
      if (digitalRead(SCALE_Y_PIN))
        yValue |= ((long)0xfff00000);
      yReportedValue = yValue;
      yValue = 0L;

      if (digitalRead(SCALE_Z_PIN))
        zValue |= ((long)0xfff00000);
      zReportedValue = zValue;
      zValue = 0L;
      
      // Tell the main loop, that it's time to sent data
      tickTimerFlag = true;
    } 
  }

  updateFrequencyCounter++;

  // Start of next cycle
  // should use updateFrequencyCounterLimit    25mS
  if ( updateFrequencyCounter >= 1000) {
    updateFrequencyCounter = 0;
  }

  tachTickCounter++;
  if ( tachTickCounter >= 10000) {     //  250mS
    tachTickCounter = 0;
  // Read and clear the pulse counter - pulses per 250ms
    pcnt_get_counter_value(PCNT_UNIT, &pulseCount);
    pcnt_counter_clear(PCNT_UNIT);
  }

 // timerAlarmEnable(timer2);
}

//initializes clock timer
void setupClkTimer()
{
  updateFrequencyCounter = 0;

  timer1 = timerBegin(1, 80, true);   // 1us ticks count up
  timer2 = timerBegin(2, 80, true);   // 1us ticks @80MHz count up
  timerAttachInterrupt(timer1, &onTimer1, true);
  timerAttachInterrupt(timer2, &onTimer2, true);
  timerAlarmWrite(timer2, TIMER2_ALARM_TICKS, false);   // timer 2, alarm after TIMER2_ALARM_TICKS ticks, dont autoreload
  timerAlarmWrite(timer1, TIMER1_ALARM_TICKS, true);     // timer 1, TIMER1_ALARM_TICKS ticks and autoreload
  timerAlarmEnable(timer2);
  timerAlarmEnable(timer1);
}


void initializeAxisAverage(volatile long axisLastRead[], volatile int &axisLastReadPosition, volatile long &axisAMAValue) {
  
  for (axisLastReadPosition = 0; axisLastReadPosition < (int) AXIS_AVERAGE_COUNT; axisLastReadPosition++) {
    axisLastRead[axisLastReadPosition] = 0;
  }
  axisLastReadPosition = 0;
  axisAMAValue = 0;

}

void scaleValueRounded(volatile long &ReportedValue, volatile long axisLastRead[], volatile int &axisLastReadPosition, volatile long &axisAMAValue)
{

  int last_pos; 
  int first_pos;
  int prev_pos;
  int filter_pos;


  long dir;
  long minValue = longMax;
  long maxValue = longMin;
  long volatility = 0;
  long valueRange;
  long ssc;
  long constant;
  long delta;

  // Save current read and increment position 
  axisLastRead[axisLastReadPosition] = ReportedValue;
  last_pos = axisLastReadPosition;

  axisLastReadPosition++;
  if (axisLastReadPosition == (int) AXIS_AVERAGE_COUNT) {
    axisLastReadPosition = 0;
  }
  first_pos = axisLastReadPosition;
  
    dir = (axisLastRead[first_pos] - axisLastRead[last_pos]) * ((long) 100);

    // Calculate the volatility in the counts by taking the sum of the differences
    prev_pos = first_pos;
    for (filter_pos = (first_pos + 1) % AXIS_AVERAGE_COUNT;
         filter_pos != first_pos;
         filter_pos = (filter_pos + 1) % AXIS_AVERAGE_COUNT)
    {
        minValue = MIN(minValue, axisLastRead[filter_pos]);
        maxValue = MAX(maxValue, axisLastRead[filter_pos]);
        volatility += ABS(axisLastRead[filter_pos] - axisLastRead[prev_pos]);
        prev_pos = filter_pos;
    }

    // Just return the read if there is no volatility to avoid divide by 0
    if (volatility == (long) 0)
    {
    axisAMAValue = axisLastRead[last_pos] * ((long) 100);
    return;
    }
  
    // If the last AMA is not within twice the sample range, then assume the position jumped
    // and reset the AMA to the current read
    maxValue = maxValue * ((long) 100);
    minValue = minValue * ((long) 100);
    valueRange = maxValue - minValue;
    if (axisAMAValue > maxValue + valueRange + ((long) 100) ||
        axisAMAValue < minValue - valueRange - ((long) 100))
    {
    axisAMAValue = axisLastRead[last_pos] * ((long) 100);
    return;
    }

    // Calculate the smoothing constant
    ssc = (ABS(dir / volatility) * fastSc) + slowSc;
    constant = (ssc * ssc) / ((long) 10000);

    // Calculate the new average
    delta = axisLastRead[last_pos] - (axisAMAValue / ((long) 100));
    axisAMAValue = axisAMAValue + constant * delta; 

    ReportedValue = (axisAMAValue + ((long) 50)) / ((long) 100);
  return;

}

long MIN(long value1, long value2){
  if(value1 > value2) {
    return value2;
  } else {
    return value1;
  }
}

long MAX(long value1, long value2){
  if(value1 > value2) {
    return value1;
  } else {
    return value2;
  }
}

long ABS(long value){
  if(value < 0) {
    return -value;
  } else {
    return value;
  }
}
