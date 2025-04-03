//Kerbol convention: main loop on core1, freertos task on core0

#include "SPI.h"

#define CS_PIN 5  // Chip Select (SS) pin for AS5047P
#define led_pin 22
#define SPI_SPEED 10000000  // SPI speed (10 MHz)
hw_timer_t *Timer0_Cfg = NULL;
TaskHandle_t Core0Handle;

uint16_t rawPosition;
uint16_t position;

void Core0Loop(void *args) {
  while (true) {
    position = rawPosition & 0x3FFF;
    Serial.println(position);
    delay(125);
  }
}

void IRAM_ATTR Timer0_ISR() {
  digitalWrite(led_pin, !digitalRead(led_pin));
  digitalWrite(CS_PIN, LOW);
  rawPosition = SPI.transfer16(0xFFFF);
  digitalWrite(CS_PIN, HIGH);
}

void setup() {
  Serial.begin(115200);
  // Set up SPI
  pinMode(led_pin, OUTPUT);
  pinMode(CS_PIN, OUTPUT);                                            // Set CS pin as output
  digitalWrite(CS_PIN, HIGH);                                         // Deselect the device
  SPI.begin(4, 16, 17, 5);                                            // Start SPI
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1));  // AS5047P uses SPI mode 1

  xTaskCreatePinnedToCore(
    Core0Loop,      /* Function to implement the task */
    "processQueue", /* Name of the task */
    10000,          /* Stack size in words */
    NULL,           /* Task input parameter */
    0,              /* Priority of the task */
    &Core0Handle,   /* Task handle. */
    0);             /* Core where the task should run */

  // Initialize Timer (with frequency in Hz)

  Timer0_Cfg = timerBegin(4000);  // Frequency in Hz

  // Attach the ISR function
  timerAttachInterrupt(Timer0_Cfg, Timer0_ISR);
  // Set the alarm value
  // (1000 ticks = 1ms) and enable autoreload
  timerAlarm(Timer0_Cfg, 1, true, 0);
}

uint16_t transfer16(uint16_t data) {
  uint16_t response = 0;

  digitalWrite(CS_PIN, LOW);        // Select the AS5047P
  response = SPI.transfer16(data);  // Send and receive 16 bits
  digitalWrite(CS_PIN, HIGH);       // Deselect the AS5047P

  return response;
}


void loop() {
}