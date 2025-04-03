// Libraries
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "sbus.h"
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
#include "mpu9250.h"

#include <driver/rmt.h>
#include <esp_task_wdt.h>

#define WATCHDOG_TIMEOUT 2

// I2C Variables
#include <Wire.h>
TwoWire I2Cone = TwoWire(0); // I2C bus for MPU
TwoWire I2Ctwo = TwoWire(1);  // I2C bus for board-to-board communication






// SD card logging
// TODO: implement data logging and automatic file creation (see ThrustStand.ino)
hw_timer_t *Timer0_Cfg = NULL;
TaskHandle_t loggingTask;

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

  while (!isQueueEmpty()) {
      file.println(messageQueue[tail]);
      tail = (tail + 1) % QUEUE_SIZE;
  }
  file.close();
}

void writeToSDcard(void * args) {
  while (true) {
    int start = micros();
    processMessages(SD);
    int end  = micros();
    // Serial.println(end - start);
    delay(1);
  }
}

void IRAM_ATTR Timer0_ISR() {
    enqueueMessage(String(msgCount++));
}

void configureSDcard() {
  SPI.begin(sck, miso, mosi, cs);

  if (!SD.begin(cs)) {
    Serial.println("Card Mount Failed");
    // return;
  }
  Serial.println("Card mounted");

  // Initialize Timer (with frequency in Hz)

  Timer0_Cfg = timerBegin(10000); // Frequency in Hz

  // Attach the ISR function
  timerAttachInterrupt(Timer0_Cfg, Timer0_ISR);

  // Set the alarm value 
  // (1000 ticks = 1ms) and enable autoreload
  timerAlarm(Timer0_Cfg, 1, true, 0);
}









// SBUS for rx and tx comms to radio receiver
HardwareSerial& serialPort = Serial2;
const int8_t RX_PIN = 16;
const int8_t TX_PIN = 17;

const int JOYSTICK_LOW = 174;
const int JOYSTICK_HIGH = 1811;

long throttleCommand = 0;
long rollCommand = 500;
long pitchCommand = 500;
long yawCommand = 500;
long SAToggle = 0;
long SBToggle = 0;

TaskHandle_t receiverTask;

long lastReceived = 0;

/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&serialPort, RX_PIN, TX_PIN, true);
/* SBUS object, writing SBUS */
bfs::SbusTx sbus_tx(&serialPort, RX_PIN, TX_PIN, true);
/* SBUS data */
bfs::SbusData data;

long scaleCommand(int command) {
  long constrainedCommand = constrain(command, JOYSTICK_LOW, JOYSTICK_HIGH);
  return map(constrainedCommand, JOYSTICK_LOW, JOYSTICK_HIGH, 0, 1000);
}

void configureReceiver() {
  /* Begin the SBUS communication */
  sbus_rx.Begin();
  sbus_tx.Begin();
}

void readReceiver(void * parameter) {
  while (true) {
    if (sbus_rx.Read()) {
      /* Grab the received data */
      data = sbus_rx.data();

      if (data.NUM_CH < 16) {
        continue;
      }

      rollCommand = 1000 - scaleCommand(data.ch[0]); // rollCommand is inverted
      pitchCommand = scaleCommand(data.ch[1]);
      throttleCommand = scaleCommand(data.ch[2]);
      yawCommand = scaleCommand(data.ch[3]);

      if (data.lost_frame == 0) {
        lastReceived = millis();
      }
    }

    // Serial.println(data.ch[5]);
    if (data.ch[4] == 191) {
      throttleCommand = 0;
    }

    if (millis() - lastReceived > 500) {
      abort();
    }
  }
}









// Kalman Filter
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
#define CALIBRATION_SAMPLES 1000 // Number of samples for calibration

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* Mpu9250 object */
bfs::Mpu9250 imu;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
float tempRaw;

long prevtime = 0;
long prevprinttime = 0;

long prevtime1 = 0;
long prevprinttime1 = 0;

double gyroXangle, gyroYangle; // Angle calculated using the gyro only
double compAngleX, compAngleY; // Calculated angle using Kalman filter
double kalAngleX, kalAngleY; // Calculated angle using Kalman filter

// Calibration variables
double ax_offset = 0.21, ay_offset = -0.08, az_offset = -19.95;
double gx_offset = 0.01, gy_offset = -0.01, gz_offset = 0.02;

uint32_t timer;

// Task handler
TaskHandle_t kalmanDebugTask;

void calibrateMPU9250() {
  int sampleCount = 0;
  float gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;
  float accelXSum = 0, accelYSum = 0, accelZSum = 0;

  // Collect data for calibration
  while (sampleCount < CALIBRATION_SAMPLES) {
    if (imu.Read()) {
      gyroXSum += imu.gyro_x_radps();
      gyroYSum += imu.gyro_y_radps();
      gyroZSum += imu.gyro_z_radps();

      accelXSum += imu.accel_x_mps2();
      accelYSum += imu.accel_y_mps2();
      accelZSum += imu.accel_z_mps2();

      sampleCount++;
    }
  }

  // Compute the average offsets
  gx_offset = gyroXSum / CALIBRATION_SAMPLES;
  gy_offset = gyroYSum / CALIBRATION_SAMPLES;
  gz_offset = gyroZSum / CALIBRATION_SAMPLES;

  ax_offset = accelXSum / CALIBRATION_SAMPLES;
  ay_offset = accelYSum / CALIBRATION_SAMPLES;
  az_offset = (accelZSum / CALIBRATION_SAMPLES) - 9.81; // Adjust for gravity

  Serial.println("Calibration complete!");
  Serial.print("Gyro Offsets: ");
  Serial.print(gx_offset); Serial.print(", ");
  Serial.print(gy_offset); Serial.print(", ");
  Serial.println(gz_offset);

  Serial.print("Accel Offsets: ");
  Serial.print(ax_offset); Serial.print(", ");
  Serial.print(ay_offset); Serial.print(", ");
  Serial.println(az_offset);

  while (true) {}
}

void configureIMU() {
  Wire.begin(21, 22);
  Wire.setClock(400000);
  // I2C bus,  0x68 address
  imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);
  // Initialize and configure IMU
  if (!imu.Begin()) {
    Serial.println("Error initializing communication with IMU");
    while(1) {}
  }
  // Set the sample rate divider
  if (!imu.ConfigSrd(19)) {
    Serial.println("Error configured SRD");
    while(1) {}
  }

  Serial.println("IMU Initialized");
  timer = micros();
}


// Derived from: https://github.com/TKJElectronics/KalmanFilter
void KalmanFilter() {
  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  //Calculate loop frequency
  // if(millis() - prevprinttime >= 1000){
  //   Serial.println(1000/(millis() - prevtime));
  //   Serial.println(kalAngleX);
  //   Serial.println(kalAngleY);
  //   prevprinttime = millis();
  // }
  prevtime = millis();
  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
  #ifdef RESTRICT_PITCH // Eq. 25 and 26
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  #else // Eq. 28 and 29
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  #endif

    double gyroXrate = gyroX / 131.0; // Convert to deg/s
    double gyroYrate = gyroY / 131.0; // Convert to deg/s

  #ifdef RESTRICT_PITCH
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
      kalmanX.setAngle(roll);
      compAngleX = roll;
      kalAngleX = roll;
      gyroXangle = roll;
    } else
      kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleX) > 90)
      gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
  #else
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
      kalmanY.setAngle(pitch);
      compAngleY = pitch;
      kalAngleY = pitch;
      gyroYangle = pitch;
    } else
      kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleY) > 90)
      gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
  #endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
}

void Core0Loop(void * parameter) {
  while (true) {
    Serial.print("t=");
    Serial.print(millis() / 1000.0);
    Serial.print(",KalAngleX=");
    Serial.print(kalAngleX);
    Serial.print(",KalAngleY=");
    Serial.print(kalAngleY);
    Serial.print(",gyroX=");
    Serial.print(gyroX);
    Serial.print(",gyroY=");
    Serial.print(gyroY);
    Serial.print(",gyroZ=");
    Serial.println(gyroZ);

    delay(5);
  }
}

void updateIntertialData() {
  if (imu.Read()) {
    accX = (float) imu.accel_x_mps2() - ax_offset;
    accY = (float) imu.accel_y_mps2() - ay_offset;
    accZ = (float) imu.accel_z_mps2() - az_offset;
    tempRaw = (float) imu.die_temp_c();
    gyroX = (float) imu.gyro_x_radps() - gx_offset;
    gyroY = (float) imu.gyro_y_radps() - gy_offset;
    gyroZ = (float) imu.gyro_z_radps() - gz_offset;
    }

    KalmanFilter();
}




//PID Loop
//x is roll
//kalanglex, kalangley
//gyrox, gyroy
//x is pitch
float maxRollRate = 30.0; //maximum roll rate in angles/second
float maxPitchRate = 30.0; //maximum pitch rate in angles/second
float p_roll = 0.2;
float i_roll = 0.3;
float d_roll = 0.01;
float p_pitch = 0.2;
float i_pitch = 0.3;
float d_pitch = 0.01;
float pitch_des, roll_des;
float error_roll, error_roll_prev, roll_jerk, integral_roll = 0;
float error_pitch, error_pitch_prev, pitch_jerk, integral_pitch = 0;
float roll_pid, pitch_pid;
float pid_time_prev = 0; 
float pid_time_delta;

void rate_PID() {
  pid_time_delta = millis() - pid_time_prev; //should just be one most times
  pid_time_prev = millis(); 
  roll_des = (rollCommand - 500)/500.0;
  pitch_des = (pitchCommand - 500)/500.0;
  error_roll = roll_des + gyroY;
  error_pitch = pitch_des - gyroX;
  roll_jerk = (error_roll - error_roll_prev)/pid_time_delta;
  pitch_jerk = (error_pitch - error_pitch_prev)/pid_time_delta;
  integral_roll = integral_roll + error_roll * pid_time_delta;
  integral_pitch = integral_pitch + error_pitch * pid_time_delta;
  pitch_pid = p_pitch*error_pitch + d_pitch*pitch_jerk; //not adding the integral term now
  roll_pid = p_roll*error_roll + d_roll*roll_jerk; //not adding the integral term now
  error_roll_prev = error_roll;
  error_pitch_prev = error_pitch;

  //Debug
  // Serial.print("pitch_Des "); Serial.print(pitch_des); 
  // Serial.print("\t roll_des "); Serial.print(roll_des); 
  // Serial.print("\t error_roll "); Serial.print(error_roll);
  // Serial.print("\t error_pitch "); Serial.print(error_pitch);
  // Serial.print("\t gyroX "); Serial.print(gyroX);
  // Serial.print("\t gyroY "); Serial.print(gyroY);
  // Serial.print("\t pitch_jerk "); Serial.print(pitch_jerk);
  // Serial.print("\t roll_jerk "); Serial.print(roll_jerk);
  Serial.print("\t roll_pid "); Serial.print(roll_pid);
  Serial.print("\t pitch_pid"); Serial.println(pitch_pid);
}









// DShot300
#define RMT_TX_CHANNEL RMT_CHANNEL_0
#define RMT_TX_GPIO GPIO_NUM_13       // GPIO pin for DShot output

// DShot300 timing constants
#define DSHOT_BIT_TOTAL_TICKS 267     // Total ticks for a bit (3.33 µs)
#define DSHOT_HIGH_1_TICKS 178        // High time for a "1" (2/3 of bit time)
#define DSHOT_HIGH_0_TICKS 89         // High time for a "0" (1/3 of bit time)

// Calculate DShot CRC bits
uint8_t calculateDShotChecksum(uint16_t value) {
  uint8_t checksum = 0;
  for (int i = 0; i < 3; i++) {
    checksum ^= (value >> (i * 4)) & 0xF;
  }
  return checksum;
}

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
  rmt_write_items(RMT_TX_CHANNEL, items, 16, true);
  rmt_wait_tx_done(RMT_TX_CHANNEL, portMAX_DELAY);
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
}

void armMotors() {
  while (millis() < 1000) {
    sendDShotCommand(0, false);
  }

  while (millis() < 1500) {
    sendDShotCommand(1000, false);
  }
}









// Servos and control surfaces
const int SERVO_0 = 32; // left
const int SERVO_1 = 17; // right
const int SERVO_PWM_FREQ = 333;
const int SERVO_PWM_RESOLUTION = 12;

long aileronLeft;
long aileronRight;

void armServos() {
  if (!ledcAttach(SERVO_0, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION)) {
    Serial.println("Failed to arm servo 0");
    while (true) {}
  }

  if (!ledcAttach(SERVO_1, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION)) {
    Serial.println("Failed to arm servo 1");
    while (true) {}
  }
}

void commandServos() {
  long servoLeft = rollCommand + (pitchCommand - 500);
  long servoRight = rollCommand - (pitchCommand - 500);

  aileronLeft = roll_pid*250 + pitch_pid*250 + servoLeft;
  aileronRight = roll_pid*250 - pitch_pid*250 + servoRight;

  aileronLeft = constrain(aileronLeft, 0, 1000);
  aileronRight = constrain(aileronRight, 0, 1000);
  aileronLeft = map(aileronLeft, 0, 1000, 820, 3200);
  aileronRight = map(aileronRight, 0, 1000, 820, 3200);

  // Debug
  // Serial.print("left "); Serial.print(aileronLeft);
  // Serial.print("   right "); Serial.println(aileronRight);

  ledcWrite(SERVO_1, aileronLeft);
  ledcWrite(SERVO_0, aileronRight);
}








// Setup and main loop
void setup() {
  Serial.begin(115200);
  configureReceiver();

  xTaskCreatePinnedToCore(
      readReceiver, /* Function to implement the task */
      "read receiver", /* Name of the task */
      2000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      0,  /* Priority of the task */
      &receiverTask,  /* Task handle. */
      0); /* Core where the task should run */
  

  delay(1000); // Wait to see if the killswitch is on and/or if the controller is connected. If not, receiver task will timeout and reboot

  configureIMU();
  configureRMT();
  armServos();
  // armMotors();

  // xTaskCreatePinnedToCore(
  //     loggingTask, /* Function to implement the task */
  //     "Logging", /* Name of the task */
  //     10000,  /* Stack size in words */
  //     NULL,  /* Task input parameter */
  //     0,  /* Priority of the task */
  //     &Core0Handle,  /* Task handle. */
  //     0); /* Core where the task should run */

  // xTaskCreatePinnedToCore(
  //     Core0Loop, /* Function to implement the task */
  //     "Kalman Filter Debug", /* Name of the task */
  //     10000,  /* Stack size in words */
  //     NULL,  /* Task input parameter */
  //     0,  /* Priority of the task */
  //     &kalmanDebugTask,  /* Task handle. */
  //     0); /* Core where the task should run */

  // Configure Watchdog
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WATCHDOG_TIMEOUT * 1000,  // Timeout in milliseconds
    .trigger_panic = true // Ensure system does not restart
  };

  esp_task_wdt_deinit();
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);

  // calibrateMPU9250();

  Serial.println("Setup Finished");
  delay(1000);
}

void loop() {
  updateIntertialData();
  rate_PID();

  sendDShotCommand(throttleCommand, false);
  commandServos();
  // Serial.println(throttleCommand);
  // Serial.println(lastReceived);
  esp_task_wdt_reset();

  if (data.ch[4] == 191) {
    throttleCommand = 0;
  }
}