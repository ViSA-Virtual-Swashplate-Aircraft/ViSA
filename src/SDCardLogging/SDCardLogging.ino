#include "FS.h"
#include "SD.h"
#include "SPI.h"

#define LED 4

hw_timer_t *Timer0_Cfg = NULL;
TaskHandle_t Core0Handle;

int sck = 18;
int miso = 19;
int mosi = 23;
int cs = 5;

int msgCount = 0;

const size_t QUEUE_SIZE = 3000; // 3000 is max safe queue size. Otherwise you risk overflowing memory
String messageQueue[QUEUE_SIZE];
volatile size_t head = 0;  // Write index
volatile size_t tail = 0;  // Read index

bool isQueueFull() {
  // If head has caught up to tail (i.e. it is about to "lap" the tail)
    return ((head + 1) % QUEUE_SIZE) == tail;
}

bool isQueueEmpty() {
    return head == tail;
}

void enqueueMessage(const String &message) {
    if (isQueueFull()) {
        // Drop the oldest message (increment tail)
        tail = (tail + 1) % QUEUE_SIZE;
    }
    messageQueue[head] = message;
    head = (head + 1) % QUEUE_SIZE;
}

void processMessages(fs::FS &fs) {
  File file = fs.open("/hello.txt", FILE_APPEND);
  if (!file) {
      Serial.println("Failed to open file for appending!");
      return;
  }

  // Write all messages in queue
  while (!isQueueEmpty()) {
      file.println(messageQueue[tail]);
      tail = (tail + 1) % QUEUE_SIZE;
  }
  file.close();
}

void Core0Loop(void * args) {
  while (true) {
    int start = micros();
    processMessages(SD);
    int end  = micros();
    Serial.println(end - start);
    delay(1);
  }
}

void IRAM_ATTR Timer0_ISR() {
    digitalWrite(LED, !digitalRead(LED)); // See logging rate on oscilloscope
    enqueueMessage(String(msgCount++));
}

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);

  SPI.begin(sck, miso, mosi, cs);
  if (!SD.begin(cs)) {
    Serial.println("Card Mount Failed");
    // return;
  }
  Serial.println("Card mounted");
  // writeFile(SD, "/hello.txt", "Hello ");
  // Serial.println("Wrote in setup");

  xTaskCreatePinnedToCore(
      Core0Loop, /* Function to implement the task */
      "processQueue", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      0,  /* Priority of the task */
      &Core0Handle,  /* Task handle. */
      0); /* Core where the task should run */

  // Initialize Timer (with frequency in Hz)

  Timer0_Cfg = timerBegin(10000); // Frequency in Hz

  // Attach the ISR function
  timerAttachInterrupt(Timer0_Cfg, Timer0_ISR);

  // Set the alarm value 
  // (1000 ticks = 1ms) and enable autoreload
  timerAlarm(Timer0_Cfg, 1, true, 0);
}

void loop(){
  
}