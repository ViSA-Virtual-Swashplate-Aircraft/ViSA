#include <driver/rmt.h>

#define RMT_TX_CHANNEL RMT_CHANNEL_0
#define RMT_TX_GPIO GPIO_NUM_5      // GPIO pin for DShot output

// Protocol timing constants
#define BIT_TOTAL_TICKS 133     // 267 total ticks for a bit (3.33 µs)
#define HIGH_1_TICKS 89      // 178 high time for a "1" (2/3 of bit time)
#define HIGH_0_TICKS 44       // 89 high time for a "0" (1/3 of bit time)

const int clockPin = 4;

union intBytes {
  byte b[2];
  uint16_t n;
};

union intBytes packet[5];
int packetBits = 81;

// Function to send a DShot command
// Bits are in reverse order
void RMTSend() {
  rmt_item32_t items[packetBits];
  uint8_t bits[packetBits];
  int head = 1; // Position of next bit

  for (int i = 0; i < 5; i++) { // iterate through union items
    union intBytes currentNum = packet[i];

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

  // Serial.println();

  // Configure header bit
  items[0].level0 = 1;
  items[0].duration0 = BIT_TOTAL_TICKS;
  items[0].level1 = 0;
  items[0].duration1 = BIT_TOTAL_TICKS;

  // Send the DShot packet
  digitalWrite(clockPin, HIGH);
  rmt_write_items(RMT_TX_CHANNEL, items, packetBits, true);
  rmt_wait_tx_done(RMT_TX_CHANNEL, portMAX_DELAY);
  digitalWrite(clockPin, LOW);
}

void configureRMT() {
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

  pinMode(clockPin, OUTPUT);
}


void setup() {
  Serial.begin(115200);
  configureRMT();
  for (int i = 0; i<5; i++) {
    packet[i].n = 4000 + i;
  }
}

void loop() {
  RMTSend();
}