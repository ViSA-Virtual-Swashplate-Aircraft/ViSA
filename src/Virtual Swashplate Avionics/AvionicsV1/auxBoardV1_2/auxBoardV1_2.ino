#include <driver/rmt.h>
#include <esp_task_wdt.h>
#include "driver/i2c.h"
#include "SPI.h"
#include "FS.h"
#include "SD.h"

// Watchdog
#define WATCHDOG_TIMEOUT 2





// Encoder Code
#define CS_PIN 5  // Chip Select (SS) pin for AS5047P
#define SPI_SPEED 10000000  // SPI speed (10 MHz)
TaskHandle_t encoderDebugTask;

uint16_t rawPosition;
uint16_t prevPosition = 0;
uint16_t position;
int parityFails = 0;

SPIClass vspi(VSPI);

float dt = 0;
int da_position = 0;
float radps = 0;

volatile long throttleCommand = 0;
volatile float phase_delay = 0; // *2pi / 255

void encoderDebug(void *args) {
  while (true) {
    Serial.println(position);
    delay(125);
  }
}

void IRAM_ATTR encoder_ISR() {
  digitalWrite(CS_PIN, LOW);
  // UBaseType_t uxSavedInterruptStatus;
  // uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
  rawPosition = vspi.transfer16(0xFFFF);
  // taskEXIT_CRITICAL_FROM_ISR( uxSavedInterruptStatus );
  digitalWrite(CS_PIN, HIGH);

  int parityBit = (rawPosition & (1 << 15)) >> 15;
  int totalHigh = 0;

  for (int i = 0; i < 15; i++) {
    int currentBit = (rawPosition & (1 << i)) >> i;

    if (currentBit) {
      totalHigh++;
    }
  }

  if ((totalHigh + parityBit) % 2 != 0) { // parity is even
    parityFails++;
    return;
  }

  position = rawPosition & 0x3FFF;
}

void configureEncoder() {
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  vspi.begin(14, 32, 13, 5);
  vspi.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1));  // AS5047P uses SPI mode 1

  // xTaskCreatePinnedToCore(
  // encoderDebug,      /* Function to implement the task */
  // "Encoder Debug", /* Name of the task */
  // 10000,          /* Stack size in words */
  // NULL,           /* Task input parameter */
  // 0,              /* Priority of the task */
  // &encoderDebugTask,   /* Task handle. */
  // 0);             /* Core where the task should run */
}








// I2C scl 15 sda 4
// Based on: https://github.com/espressif/esp-idf/blob/v3.3/examples/peripherals/i2c/i2c_self_test/main/i2c_example_main.c
#define DATA_LENGTH 512                  /*!< Data buffer length of test buffer */

#define I2C_SLAVE_SCL_IO 25               /*!< gpio number for i2c slave clock */
#define I2C_SLAVE_SDA_IO 26               /*!< gpio number for i2c slave data */
#define I2C_SLAVE_TX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave tx buffer size */
#define I2C_SLAVE_RX_BUF_LEN 10              /*!< I2C slave rx buffer size */
#define ESP_SLAVE_ADDR_1 0x55 /*!< ESP32 7-bit slave address 1 */
#define ESP_SLAVE_ADDR_2 0x60 /*!< ESP32 7-bit slave address 2 */

int led = 13;

volatile bool canLog = false;

volatile long lastReceived = 0;

float receivedMessages = 0;
float timeouts = 0;

volatile uint16_t receivedInts[3] = {0, 0, 0};
int currentInt;
int numofItems;
int sinusoidalThrottle = 0;
long lastReceivedMax = 0;
volatile uint16_t phaseDelayScaled;
volatile int16_t rollCommandRaw;
volatile int16_t yawCommandRaw;
volatile uint32_t realMillis;
volatile float yawCommandSaturator;

volatile float yawCommandScaled;
volatile float amplitude_received;

double phaseDelay;

TaskHandle_t i2cTask;

void i2cReceive(void *args) {
  uint8_t data[6];
  int size = 0;

  while (true) {
  size = i2c_slave_read_buffer(I2C_NUM_0, data, 6, 1000 / portTICK_RATE_MS);

  if (size >= 6) {
    throttleCommand = ((uint16_t)data[0] << 8) | data[1];

    if (!(0 <= throttleCommand && throttleCommand <= 1000)) {
      throttleCommand = 0;
      Serial.println("i2c error: invalid throttle command!");
      continue;
    }

    rollCommandRaw = (((int16_t)data[2] << 8) | data[3]);
    yawCommandRaw = (((int16_t)data[4] << 8) | data[5]);

    yawCommandSaturator = map(yawCommandRaw, 0, 1000, 900, 1100) / 1000.0;

    amplitude_received = map(rollCommandRaw, 0, 1000, -1000, 1000) / 1000.0;
    yawCommandScaled = map(yawCommandRaw, 0, 1000, -1000, 1000) / 1000.0;

    lastReceived = millis();
    // receivedMessages++;
    canLog = true;

    if(amplitude_received > 0)
    {
      phaseDelay = 2.8;
    } else {
      phaseDelay = 5.4;
    }
  
    esp_task_wdt_reset();
  }
  vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void configureI2CRx() {
  xTaskCreatePinnedToCore(
      i2cReceive, /* Function to implement the task */
      "i2c Receive", /* Name of the task */
      5000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      1,  /* Priority of the task */
      &i2cTask,
      1); /* Core where the task should run */

}

void I2CDebug() {
  Serial.println(throttleCommand);
  // delay();
}









// RMT Logging
// Protocol timing constants
#define BIT_TOTAL_TICKS 90     // Total ticks for a bit
#define HIGH_1_TICKS  60      // High time for a "1"
#define HIGH_0_TICKS 30       // High time for a "0"

const int clockPin = 17;

long prevMicros = 0;

TaskHandle_t loggingTaskHandle;

union uint16Bytes {
  byte b[2];
  uint16_t n;
};

union uint16Bytes packet[6];
int packetBits = 97;

uint16_t fletcher16Checksum(const union uint16Bytes *packet, size_t len) {
  uint32_t sum1 = 0, sum2 = 0;

  for (size_t i = 0; i < len; i++) {
    sum1 = (sum1 + packet[i].n) % 255;
    sum2 = (sum2 + sum1) % 255;
  }

  return (sum2 << 8) | sum1;
}


// send data using RMT
// Bits are MSB first (big endian)
void RMTSendData(void * args) {
  while (true) {
    if (!canLog) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    rmt_item32_t items[packetBits];
    int head = 1; // Position of first bit. 0 is reserved for the header bit

    // Serial.println(radps);

    packet[0].n = (uint16_t)throttleCommand;
    packet[1].n = (uint16_t)sinusoidalThrottle;
    packet[2].n = (uint16_t)radps;
    // packet[3].n = (uint16_t)phase_delay;
    packet[5].n = fletcher16Checksum(packet, 5);


    prevPosition = position;
    prevMicros = micros();

    for (int i = 0; i < 6; i++) { // iterate through union items
      union uint16Bytes currentNum = packet[i];

      for (int x = 1; x >=0; x--) { // iterate through bytes
      byte currentByte = currentNum.b[x];

        for (int y = 7; y >= 0; y--) { // iterate through bits
          int currentBit = (currentByte >> y) & 1;  // Extract each bit

          // Serial.print(currentBit);

          if (currentBit) { // bit is 1
            items[head].level0 = 1;
            items[head].duration0 = HIGH_1_TICKS;
            items[head].level1 = 0;
            items[head].duration1 = BIT_TOTAL_TICKS - HIGH_1_TICKS;
          } else {
            items[head].level0 = 1;
            items[head].duration0 = HIGH_0_TICKS;
            items[head].level1 = 0;
            items[head].duration1 = BIT_TOTAL_TICKS - HIGH_0_TICKS;
          }
          head++;
        }
      }
    }

    digitalWrite(clockPin, HIGH);

    // Configure header bit
    items[0].level0 = 1;
    items[0].duration0 = BIT_TOTAL_TICKS;
    items[0].level1 = 0;
    items[0].duration1 = BIT_TOTAL_TICKS;

    // Send the DShot packet
    rmt_write_items(RMT_CHANNEL_1, items, packetBits, false);
    digitalWrite(clockPin, LOW);

    delayMicroseconds(190);
  }
}

void configureRMTLogging() {
  // Configure RMT
  rmt_config_t rmt_tx_config;
  rmt_tx_config.channel = RMT_CHANNEL_1;
  rmt_tx_config.gpio_num = GPIO_NUM_5;
  rmt_tx_config.clk_div = 1;
  rmt_tx_config.mem_block_num = 3;
  rmt_tx_config.tx_config.loop_en = false;
  rmt_tx_config.tx_config.carrier_en = false;
  rmt_tx_config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
  rmt_tx_config.tx_config.idle_output_en = true;
  rmt_tx_config.rmt_mode = RMT_MODE_TX;

  // Apply configuration
  rmt_config(&rmt_tx_config);
  rmt_driver_install(rmt_tx_config.channel, 0, 0);

  pinMode(clockPin, OUTPUT);
}



// DShot300 Swashplateless
#define RMT_TX_CHANNEL RMT_CHANNEL_0
#define RMT_TX_GPIO GPIO_NUM_17       // GPIO pin for DShot output

// DShot300 timing constants
#define DSHOT_BIT_TOTAL_TICKS 267     // Total ticks for a bit
#define DSHOT_HIGH_1_TICKS 178        // High time for a "1"
#define DSHOT_HIGH_0_TICKS 89         // High time for a "0"

// Calculate DShot CRC bits
uint8_t calculateDShotChecksum(uint16_t value) {
  uint8_t checksum = 0;
  for (int i = 0; i < 3; i++) {
    checksum ^= (value >> (i * 4)) & 0xF;
  }
  return checksum;
}

// Function to send a DShot command
void sendDShotCommand(uint16_t throttle, bool telemetry) {
  throttle = constrain(throttle, 0, 1000);
  throttle = map(throttle, 0, 1000, 48, 2047);
  // Prepare the 16-bit packet
  uint16_t packet = (throttle << 1) | (telemetry ? 1 : 0);
  packet = (packet << 4) | calculateDShotChecksum(packet);

  // Convert packet into RMT items
  rmt_item32_t items[16];
  for (int i = 0; i < 16; i++) {
    if (packet & (1 << (15 - i))) {  // Bit is "1"
      items[i].level0 = 1;
      items[i].duration0 = DSHOT_HIGH_1_TICKS;
      items[i].level1 = 0;
      items[i].duration1 = DSHOT_BIT_TOTAL_TICKS - DSHOT_HIGH_1_TICKS;
    } else {                          // Bit is "0"
      items[i].level0 = 1;
      items[i].duration0 = DSHOT_HIGH_0_TICKS;
      items[i].level1 = 0;
      items[i].duration1 = DSHOT_BIT_TOTAL_TICKS - DSHOT_HIGH_0_TICKS;
    }
  }

  // Send the DShot packet
  rmt_write_items(RMT_TX_CHANNEL, items, 16, false);
  rmt_wait_tx_done(RMT_TX_CHANNEL, portMAX_DELAY);
}

void configureRMTDShot() {
  // Configure RMT for DShot
  rmt_config_t rmt_tx_config;
  rmt_tx_config.channel = RMT_TX_CHANNEL;
  rmt_tx_config.gpio_num = RMT_TX_GPIO;
  rmt_tx_config.clk_div = 1;
  rmt_tx_config.mem_block_num = 1;
  rmt_tx_config.tx_config.loop_en = false;
  rmt_tx_config.tx_config.carrier_en = false;
  rmt_tx_config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
  rmt_tx_config.tx_config.idle_output_en = true;
  rmt_tx_config.rmt_mode = RMT_MODE_TX;

  // Apply configuration
  rmt_config(&rmt_tx_config);
  rmt_driver_install(rmt_tx_config.channel, 0, 0);
}

void armMotors() {
  while (millis() < 1000) {
    sendDShotCommand(0, false);
  }

  while (millis() < 1500) {
    sendDShotCommand(1000, false);
  }
}







// DShot normal motor
#define RMT_TX_CHANNEL_NORM RMT_CHANNEL_2
#define RMT_TX_GPIO_NORM GPIO_NUM_16       // GPIO pin for DShot output

// Function to send a DShot command
void sendDShotCommandNormal(uint16_t throttle, bool telemetry) {
  throttle = constrain(throttle, 0, 1000);
  throttle = map(throttle, 0, 1000, 48, 2047);
  // Prepare the 16-bit packet
  uint16_t packet = (throttle << 1) | (telemetry ? 1 : 0);
  packet = (packet << 4) | calculateDShotChecksum(packet);

  // Convert packet into RMT items
  rmt_item32_t items[16];
  for (int i = 0; i < 16; i++) {
    if (packet & (1 << (15 - i))) {  // Bit is "1"
      items[i].level0 = 1;
      items[i].duration0 = DSHOT_HIGH_1_TICKS;
      items[i].level1 = 0;
      items[i].duration1 = DSHOT_BIT_TOTAL_TICKS - DSHOT_HIGH_1_TICKS;
    } else {                          // Bit is "0"
      items[i].level0 = 1;
      items[i].duration0 = DSHOT_HIGH_0_TICKS;
      items[i].level1 = 0;
      items[i].duration1 = DSHOT_BIT_TOTAL_TICKS - DSHOT_HIGH_0_TICKS;
    }
  }

  // Send the DShot packet
  rmt_write_items(RMT_TX_CHANNEL_NORM, items, 16, false);
  rmt_wait_tx_done(RMT_TX_CHANNEL_NORM, portMAX_DELAY);
}

void configureRMTDShotNormal() {
  // Configure RMT for DShot
  rmt_config_t rmt_tx_config;
  rmt_tx_config.channel = RMT_TX_CHANNEL_NORM;
  rmt_tx_config.gpio_num = RMT_TX_GPIO_NORM;
  rmt_tx_config.clk_div = 1;
  rmt_tx_config.mem_block_num = 1;
  rmt_tx_config.tx_config.loop_en = false;
  rmt_tx_config.tx_config.carrier_en = false;
  rmt_tx_config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
  rmt_tx_config.tx_config.idle_output_en = true;
  rmt_tx_config.rmt_mode = RMT_MODE_TX;

  // Apply configuration
  rmt_config(&rmt_tx_config);
  rmt_driver_install(rmt_tx_config.channel, 0, 0);
}









// SD Card Logging
TaskHandle_t loggingTask;

String message;

int sck = 2;
int miso = 4;
int mosi = 0;
int cs = 15;

// int SD_Led = 4;

SPIClass hspi(HSPI);

int msgCount = 0;
// long receivedMessages = 0;

const size_t SD_QUEUE_SIZE = 3000;  // A queue size of 3000 causes stability issues
String SDmessageQueue[SD_QUEUE_SIZE];
volatile size_t SDhead = 0;  // Write index
volatile size_t SDtail = 0;  // Read index

String fileName;

void loggingSendData(void *args) {
  while (true) {
    Serial.println(message);
  }
}

bool isSDQueueFull() {
  return ((SDhead + 1) % SD_QUEUE_SIZE) == SDtail;
}

bool isSDQueueEmpty() {
  return SDhead == SDtail;
}

void enqueueSDMessage(const String &message) {
  if (isSDQueueFull()) {
    // Drop the oldest message (increment SDtail)
    SDtail = (SDtail + 1) % SD_QUEUE_SIZE;
  }
  SDmessageQueue[SDhead] = message;
  SDhead = (SDhead + 1) % SD_QUEUE_SIZE;
}

bool processSDMessages(fs::FS &fs) {
  File file = fs.open("/" + fileName, FILE_APPEND);
  if (!file) {
   Serial.println("Failed to open file for appending!");
    return false;
  }

  while (!isSDQueueEmpty()) {
    file.println(SDmessageQueue[SDtail]);
    SDtail = (SDtail + 1) % SD_QUEUE_SIZE;
    receivedMessages++;
  }

  file.close();

  return true;
}

void writeToSDCard(void *args) {
  while (true) {
    // Serial.println("SD card");
    // if (xSemaphoreTake(xSemaphoreScale, pdMS_TO_TICKS(1))) {
      // digitalWrite(SD_Led, !digitalRead(SD_Led));

      if (!processSDMessages(SD)) {
        // xSemaphoreGive(xSemaphoreScale);
        vTaskDelete(NULL);
      }
      // Serial.println("SD card1");
      // xSemaphoreGive(xSemaphoreScale);
    // }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void configureSDcard() {
  Serial.println("Configuring SD card...");

  hspi.begin(sck, miso, mosi, cs);

  // pinMode(SD_Led, OUTPUT);

  if (!SD.begin(cs, hspi)) {
    while (true) {Serial.println("Card Mount Failed"); delay(200);}
  }
  Serial.println("Card mounted");

  File root = SD.open("/");  // Open at root directory

  if (!root) {
    Serial.println("Could not open root directory!");
  }

  File file = root.openNextFile();
  int fileCount = 1;

  while (file) {
    fileCount++;
    file = root.openNextFile();
  }

  fileName = String(fileCount) + ".csv";
  File newfile = SD.open("/" + fileName, FILE_WRITE);
  newfile.close();

  xTaskCreate(
    writeToSDCard,
    "SD card logging",
    10000,
    NULL,
    1,
    &loggingTask);
}









// Interrupts
hw_timer_t *Encoder_Cfg = NULL;
hw_timer_t *ARM_Cfg = NULL;
long currentMicros = 0;

// void IRAM_ATTR Logging_ISR() {
//   // float radians = (position / 16383) * 2 * 3.14;

//   // float logged_position = position/102885.24 - 3.14;

//   char buf[64];
//   snprintf(buf, sizeof(buf), "%lu,%f,%lu,%lu,%lu,%d,%d",
//    currentMicros, 
//    throttleCommand, 
//    sinusoidalThrottle, 
//    amplitude, 
//    position, 
//    scaled_reading1, 
//    scaled_reading2);

//   enqueueSDMessage(String(buf));
// }

void configureInterrupts() {
  Encoder_Cfg = timerBegin(4000);  // Frequency in Hz
  // ARM_Cfg = timerBegin(100);

  // Attach the ISR function
  timerAttachInterrupt(Encoder_Cfg, encoder_ISR);

  // (1000 ticks = 1ms) and enable autoreload
  timerAlarm(Encoder_Cfg, 1, true, 0);
  // timerAlarm(ARM_Cfg, 1, true, 0);
}

TaskHandle_t debugTask;

long prevPrintTime = 0;
long currentTime = 0;
long maxPosition = 0;
float loopCount = 0;
float amplitude = 0;

int swThrottle;

void serialDebug (void *args) {
  while (true) {
    currentTime = millis();
    if (currentTime - prevPrintTime > 100) {
      Serial.print("\t SW throttle: ");
      Serial.print(swThrottle);
      Serial.print("\t throttle: ");
      Serial.print(throttleCommand);
      Serial.print("\t rps: ");
      Serial.print(radps);
      Serial.print("\t motor position: ");
      Serial.print(position);
      Serial.print("\t i2c timeouts: ");
      Serial.print(timeouts);
      Serial.print("\t amplitude: ");
      Serial.print(amplitude);
      Serial.print("\t phase delay: ");
      Serial.print(phaseDelay);
      // Serial.print("\t rollCommandScaled: ");
      // Serial.print(rollCommandScaled);
      Serial.print("\t yawCommandScaled: ");
      Serial.println(yawCommandScaled);
      
      prevPrintTime = currentTime;
    }

    phase_delay += 0.00125664;

    currentMicros = micros();


    int currentPosition = position;

    if (currentPosition != prevPosition) {
      if (currentPosition < prevPosition) {
          currentPosition += 16383;
        }

        dt = (currentMicros - prevMicros) / 1000000.0;
        da_position = currentPosition - prevPosition;
        radps = (da_position / dt) / 16383;


      prevPosition = position;
      prevMicros = micros();
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}





void setup() {
  Serial.begin(115200);
  Serial.println("Setup");

  // configureScale();

  // xTaskCreatePinnedToCore(
  //   readScale, /* Function to implement the task */
  //   "Read Scale",           /* Name of the task */
  //   5000,              /* Stack size in words */
  //   NULL,               /* Task input parameter */
  //   1,                  /* Priority of the task */
  //   &scaleTask,
  //   0);

  xTaskCreatePinnedToCore(
    serialDebug, /* Function to implement the task */
    "Serial output debug",           /* Name of the task */
    5000,              /* Stack size in words */
    NULL,               /* Task input parameter */
    1,                  /* Priority of the task */
    &debugTask,
    0);

  configureEncoder();
  // delay(500);
  configureInterrupts();

  configureRMTDShot();
  configureRMTDShotNormal();

  i2c_port_t i2c_slave_port = I2C_NUM_0;
  i2c_config_t conf_slave;
  conf_slave.sda_io_num = I2C_SLAVE_SDA_IO;
  conf_slave.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf_slave.scl_io_num = I2C_SLAVE_SCL_IO;
  conf_slave.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf_slave.mode = I2C_MODE_SLAVE;
  conf_slave.slave.addr_10bit_en = 0;
  conf_slave.slave.slave_addr = ESP_SLAVE_ADDR_2;
  i2c_param_config(i2c_slave_port, &conf_slave);
  i2c_driver_install(i2c_slave_port, conf_slave.mode,
                            I2C_SLAVE_RX_BUF_LEN,
                            I2C_SLAVE_TX_BUF_LEN, 0);

  // configureRMTLogging();

  // xTaskCreatePinnedToCore(
  //     RMTSendData, /* Function to implement the task */
  //     "RMT TX", /* Name of the task */
  //     10000,  /* Stack size in words */
  //     NULL,  /* Task input parameter */
  //     1,  /* Priority of the task */
  //     &loggingTaskHandle, 0); /* Core where the task should run */

  configureI2CRx();

  // sendDShotCommand(0, false);
  // delay(500);
  // sendDShotCommand(100, false);
  // delay(500);
  // sendDShotCommand(0, false);
  // delay(500);

  // Configure Watchdog
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WATCHDOG_TIMEOUT * 1000,  // Timeout in milliseconds
    .trigger_panic = true // Ensure system does not restart
  };

  esp_task_wdt_deinit();
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);
  esp_task_wdt_add(i2cTask);

  lastReceived = millis();
}

float absRollCommand;

void loop() {
  if (throttleCommand > 0) {
    swThrottle = 200;
  } else {
    swThrottle = 0;
  }

  if (currentTime - lastReceived > 50) {
    // Serial.print("Timeout: ");
    // Serial.println(currentTime - lastReceived);
    timeouts++;
    canLog = false;
    throttleCommand = 0;
    swThrottle = 0;
  } else {
    absRollCommand = amplitude_received * amplitude_received;
    absRollCommand = sqrt(absRollCommand);
    sinusoidalThrottle = swThrottle + sin(-1 * position * 0.00038366 + phaseDelay) *  absRollCommand * swThrottle;

    if (sinusoidalThrottle < 0) {
      sinusoidalThrottle = 0;
    }

    sendDShotCommand(sinusoidalThrottle, false);

    if (throttleCommand * yawCommandSaturator > 1000) {
      sendDShotCommandNormal(1000, false);
    } else {
      sendDShotCommandNormal(throttleCommand * yawCommandSaturator, false);
    }
  }

  esp_task_wdt_reset();

  loopCount++;

  // I2CDebug();

  // long position = position;
  // if (position > maxPosition) {
  //   maxPosition = position;
  // }

  // Serial.println(position);
  // delay(2);
}
