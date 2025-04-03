// Based on https://github.com/espressif/esp-idf/blob/v3.3/examples/peripherals/i2c/i2c_self_test/main/i2c_example_main.c
#include "driver/i2c.h"
#include <esp_task_wdt.h>

#define DATA_LENGTH 512                  /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128               /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 /*!< delay time between different test items */

#define I2C_SLAVE_SCL_IO 15               /*!< gpio number for i2c slave clock */
#define I2C_SLAVE_SDA_IO 4               /*!< gpio number for i2c slave data */
#define I2C_SLAVE_TX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave tx buffer size */
#define I2C_SLAVE_RX_BUF_LEN 12              /*!< I2C slave rx buffer size */
#define ESP_SLAVE_ADDR 0x60 /*!< ESP32 slave address, you can set any 7bit value */

volatile uint16_t receivedInts[3] = {0, 0, 0};

void setup(){
  Serial.begin(115200);
  i2c_port_t i2c_slave_port = I2C_NUM_0;
  i2c_config_t conf_slave;
  conf_slave.sda_io_num = I2C_SLAVE_SDA_IO;
  conf_slave.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf_slave.scl_io_num = I2C_SLAVE_SCL_IO;
  conf_slave.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf_slave.mode = I2C_MODE_SLAVE;
  conf_slave.slave.addr_10bit_en = 0;
  conf_slave.slave.slave_addr = ESP_SLAVE_ADDR;
  i2c_param_config(i2c_slave_port, &conf_slave);
  i2c_driver_install(i2c_slave_port, conf_slave.mode,
                            I2C_SLAVE_RX_BUF_LEN,
                            I2C_SLAVE_TX_BUF_LEN, 0);
}

void loop(){
  uint8_t data[6];
  int size = 0;

  size = i2c_slave_read_buffer(I2C_NUM_0, data, 6, 1000 / portTICK_RATE_MS);

  if (size >= 6) {
  // Combine bytes
  uint16_t int1 = ((uint16_t)data[0] << 8) | data[1];
  uint16_t int2 = ((uint16_t)data[2] << 8) | data[3];
  uint16_t int3 = ((uint16_t)data[4] << 8) | data[5];
  
  // Print the three integers using Serial.printf:
  Serial.printf("Received ints: %d, %d, %d\n", int1, int2, int3);
  } else {
    Serial.printf("Error: Expected 6 bytes, got %d\n", size);
  }
}