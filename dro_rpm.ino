/*
  ArduinoDRO + Tach V5.12 with ESP32 PCNT-Based Tachometer
*/

#include "BluetoothSerial.h"
#include <driver/pcnt.h>

// DRO config (if axis is not connected change in the corresponding constant value from "1" to "0")
#define SCALE_X_ENABLED 1
#define SCALE_Y_ENABLED 1
#define SCALE_Z_ENABLED 1

// I/O ports config (change pin numbers if DRO, Tach sensor or Tach LED feedback is connected to different ports)
#define SCALE_CLK_PIN 2
#define SCALE_X_PIN 21  // Pin for X-axis
#define SCALE_Y_PIN 4   // Pin for Y-axis
#define SCALE_Z_PIN 5   // Pin for Z-axis

// General Settings
#define UART_BAUD_RATE 9600  // Set to match Bluetooth module's BAUD rate
#define UPDATE_FREQUENCY 24  // Frequency in Hz for DRO updates

// Tachometer settings
#define TACH_PIN 14         // GPIO where tach sensor is connected
#define PCNT_UNIT PCNT_UNIT_0
#define PULSES_PER_REV 1    // Adjust based on number of pulses per revolution

// Bluetooth Serial
BluetoothSerial SerialBT;

// Variables
volatile int16_t pulseCount = 0;
volatile unsigned long lastUpdateTime = 0;

// PCNT Interrupt Handler
void IRAM_ATTR pcnt_isr_handler(void *arg) {
    pcnt_get_counter_value(PCNT_UNIT, &pulseCount);
    pcnt_counter_clear(PCNT_UNIT);
}

void setup() {
    Serial.begin(UART_BAUD_RATE);
    SerialBT.begin("ESP32_DRO");  // Bluetooth device name
    
    // Configure PCNT for tachometer
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = TACH_PIN,
        .ctrl_gpio_num = PCNT_PIN_NOT_USED,
        .channel = PCNT_CHANNEL_0,
        .unit = PCNT_UNIT,
        .pos_mode = PCNT_COUNT_INC,  // Count on rising edges
        .neg_mode = PCNT_COUNT_DIS,  // Do not count falling edges
        .lctrl_mode = PCNT_MODE_KEEP,
        .hctrl_mode = PCNT_MODE_KEEP,
        .counter_h_lim = 10000,  // Large value to avoid overflow
        .counter_l_lim = 0,
    };

    pcnt_unit_config(&pcnt_config);
    pcnt_counter_pause(PCNT_UNIT);
    pcnt_counter_clear(PCNT_UNIT);
    pcnt_isr_register(pcnt_isr_handler, NULL, 0, NULL);
    pcnt_intr_enable(PCNT_UNIT);
    pcnt_counter_resume(PCNT_UNIT);

    // Initialize DRO settings
    pinMode(SCALE_X_PIN, INPUT_PULLUP);
    pinMode(SCALE_Y_PIN, INPUT_PULLUP);
    pinMode(SCALE_Z_PIN, INPUT_PULLUP);
}

void loop() {
    unsigned long currentTime = millis();
    if (currentTime - lastUpdateTime >= 1000) {  // Update every second
        lastUpdateTime = currentTime;

        int16_t count;
        pcnt_get_counter_value(PCNT_UNIT, &count);
        pcnt_counter_clear(PCNT_UNIT);

        float rpm = (count * 60.0) / PULSES_PER_REV;  // Convert pulses to RPM
        Serial.printf("RPM: %.2f\n", rpm);
        SerialBT.printf("RPM: %.2f\n", rpm);

        // Read DRO values
        long xValue = digitalRead(SCALE_X_PIN);
        long yValue = digitalRead(SCALE_Y_PIN);
        long zValue = digitalRead(SCALE_Z_PIN);

        // Send DRO data over Bluetooth Serial
        Serial.printf("X: %ld, Y: %ld, Z: %ld\n", xValue, yValue, zValue);
        SerialBT.printf("X: %ld, Y: %ld, Z: %ld\n", xValue, yValue, zValue);
    }
}
