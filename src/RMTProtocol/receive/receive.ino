#include <driver/rmt.h>

#define RMT_RX_CHANNEL RMT_CHANNEL_1
#define RMT_RX_GPIO_NUM GPIO_NUM_5

#define BIT_TOTAL_TICKS 267  // Total ticks for a bit (3.33 µs)
#define HIGH_1_TICKS 178     // High time for a "1" (2/3 of bit time)
#define HIGH_0_TICKS 89      // High time for a "0" (1/3 of bit time)

const int BIT_CEILING = BIT_TOTAL_TICKS * 1.1;
const int BIT_FLOOR = BIT_TOTAL_TICKS * 0.9;

const int HIGH_1_CEILING = HIGH_1_TICKS * 1.1;
const int HIGH_1_FLOOR = HIGH_1_TICKS * 0.9;

const int HIGH_0_CEILING = HIGH_0_TICKS * 1.1;
const int HIGH_0_FLOOR = HIGH_0_TICKS * 0.9;

const int clockPin = 4;

union intBytes {
  byte b[2];
  uint16_t n;
};

union intBytes packet[5];
int packetBits = 81;
volatile bool receivingData = false;

RingbufHandle_t rx_buffer = NULL;

int level0 = 0;
int duration0 = 0;
int level1 = 0;
int duration1 = 0;

TaskHandle_t DebugTask;
TaskHandle_t DecodeTask;

int msgCount = 0;

const size_t QUEUE_SIZE = 2000;  // A queue size of 3000 causes stability issues
rmt_item32_t messageQueue[QUEUE_SIZE];
volatile size_t head = 0;  // Write index
volatile size_t tail = 0;  // Read index

String fileName;

bool isQueueFull() {
  return ((head + 1) % QUEUE_SIZE) == tail;
}

bool isQueueEmpty() {
  return head == tail;
}

void enqueueMessage(const rmt_item32_t& message) {
  if (isQueueFull()) {
    // Drop the oldest message (increment tail)
    tail = (tail + 1) % QUEUE_SIZE;
  }
  messageQueue[head] = message;
  head = (head + 1) % QUEUE_SIZE;
}

void processMessages() {
  if ((head - tail + QUEUE_SIZE) % QUEUE_SIZE < 160) {
    return;
  }

  bool foundPacketHead = false;
  rmt_item32_t currentItem;
  int totalDuration;
  int startTail;

  while (!foundPacketHead && !isQueueEmpty()) {
    currentItem = messageQueue[tail];
    totalDuration = currentItem.duration0 + currentItem.duration1;

    // Serial.println(String(totalDuration) + " " + String(currentItem.duration0));

    if (2 * BIT_FLOOR < totalDuration && totalDuration < 2 * BIT_CEILING) {
      foundPacketHead = true;          // header found
      tail = (tail + 1) % QUEUE_SIZE;  // consume header
      startTail = tail;
      break;  // exit the loop immediately
    } else {
      tail = (tail + 1) % QUEUE_SIZE;
    }
  }

  int bitNum = 7;
  int byteNum = 1;
  int intNum = 0;

  while (!isQueueEmpty() && intNum < 5) {
    level0 = messageQueue[tail].level0;
    duration0 = messageQueue[tail].duration0;
    level1 = messageQueue[tail].level1;
    duration1 = messageQueue[tail].duration1;

    totalDuration = duration0 + duration1;

    if (!level0 || level1) {  // level0 should always be 1 and vice versa
      Serial.println("invalid RMT item!");
      break;
    }

    int currentBit = 0;

    // if (!(BIT_FLOOR < totalDuration && totalDuration < BIT_CEILING)) { // bit is too long or too short
    //   Serial.println("invalid RMT item1!");
    //   Serial.println(String(currentBit) + " " + String(duration0) + " " + String(duration1) + " " + String(duration1));
    //   break;
    // }

    if (HIGH_1_FLOOR < duration0 && duration0 < HIGH_1_CEILING) {
      currentBit = 1;
    } else if (!(HIGH_0_FLOOR < duration0 && duration0 < HIGH_0_CEILING)) {
      Serial.print("invalid RMT item2!");
      // Serial.println(String(currentBit) + " " + String(duration0) + " " + String(byteNum) + " " + String(intNum));
      break;
    }

    // Serial.println(String(currentBit) + " " + String(bitNum) + " " + String(byteNum) + " " + String(intNum));

    // Serial.print(String(currentBit));

    if (currentBit) {
      packet[intNum].b[byteNum] |= (1 << bitNum);
    } else {
      packet[intNum].b[byteNum] &= ~(1 << bitNum);
    }

    bitNum--;

    if (bitNum < 0) {
      byteNum--;
      bitNum = 7;
    }

    if (byteNum < 0) {
      intNum++;
      byteNum = 1;
    }

    tail = (tail + 1) % QUEUE_SIZE;
  }
  // Serial.println();
}

void writeToSDCard(void* args) {
  while (true) {
    processMessages();
  }
}

void rmt_rx_arm() {
  rmt_config_t rmt_rx;
  rmt_rx.channel = RMT_RX_CHANNEL;
  rmt_rx.gpio_num = RMT_RX_GPIO_NUM;
  rmt_rx.clk_div = 1;
  rmt_rx.mem_block_num = 3;
  rmt_rx.rmt_mode = RMT_MODE_RX;
  rmt_rx.rx_config.idle_threshold = 275;

  rmt_config(&rmt_rx);
  rmt_driver_install(rmt_rx.channel, 2000, 0);
}

void Core0Loop(void* parameter) {
  while (true) {
    for (int i = 0; i < 5; i++) {
      Serial.print(packet[i].n);
      Serial.print("\t");
    }
    Serial.println();
  }
}

void IRAM_ATTR RMT_Start() {
  rmt_rx_start(RMT_RX_CHANNEL, false);
}

void IRAM_ATTR RMT_End() {
  receivingData = false;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Setup");
  rmt_rx_arm();
  rmt_get_ringbuf_handle(RMT_RX_CHANNEL, &rx_buffer);

  pinMode(clockPin, INPUT);

  attachInterrupt(clockPin, RMT_Start, FALLING);
  // attachInterrupt(clockPin, RMT_End, FALLING);

  xTaskCreatePinnedToCore(
    Core0Loop,  /* Function to implement the task */
    "Debug",    /* Name of the task */
    10000,      /* Stack size in words */
    NULL,       /* Task input parameter */
    0,          /* Priority of the task */
    &DebugTask, /* Task handle. */
    0);         /* Core where the task should run */

  xTaskCreatePinnedToCore(
    writeToSDCard, /* Function to implement the task */
    "decode",      /* Name of the task */
    10000,         /* Stack size in words */
    NULL,          /* Task input parameter */
    0,             /* Priority of the task */
    &DecodeTask,   /* Task handle. */
    0);            /* Core where the task should run */
}

void loop() {
  while (rx_buffer) {
    size_t rx_size = 0;
    rmt_item32_t* items = (rmt_item32_t*)xRingbufferReceive(rx_buffer, &rx_size, 100);
    size_t num_items = rx_size / sizeof(rmt_item32_t);

    for (int i = 0; i < num_items; i++) {
      enqueueMessage(items[i]);
    }

    // head = tail;

    vRingbufferReturnItem(rx_buffer, (void*)items);
  }
}