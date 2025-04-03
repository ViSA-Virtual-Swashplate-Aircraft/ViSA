// Libraries
#include <Wire.h>
#include "SPI.h"
#include "FS.h"
#include "SD.h"
#include "sbus.h"
#include <Kalman.h>  // Source: https://github.com/TKJElectronics/KalmanFilter
#include "BMI088.h"


#include <esp_task_wdt.h>
#include <driver/rmt.h>

volatile uint16_t throttleCommand = 0;

// Watchdog
#define WATCHDOG_TIMEOUT 2

// I2C Variables
TwoWire I2Cone = TwoWire(0);  // I2C bus for MPU
TwoWire I2Ctwo = TwoWire(1);  // I2C bus for board-to-board communication


// Kalman Filter
#define RESTRICT_PITCH            // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
#define CALIBRATION_SAMPLES 1000  // Number of samples for calibration

Kalman kalmanX;  // Create the Kalman instances
Kalman kalmanY;

/* accel object */
Bmi088Accel accel(I2Cone, 0x18);
/* gyro object */
Bmi088Gyro gyro(I2Cone, 0x68);

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
float tempRaw;

long prevtime = 0;
long prevprinttime = 0;

long prevtime1 = 0;
long prevprinttime1 = 0;

double gyroXangle, gyroYangle;  // Angle calculated using the gyro only
double compAngleX, compAngleY;  // Calculated angle using Kalman filter
double kalAngleX, kalAngleY;    // Calculated angle using Kalman filter

// Calibration variables
double ax_offset = 0.21, ay_offset = -0.08, az_offset = -19.95;
double gx_offset = 0.01, gy_offset = -0.01, gz_offset = 0.02;

uint32_t timer;
uint8_t i2cData[14];  // Buffer for I2C data

// Task handlers for each core
TaskHandle_t kalmanDebugTask;
TaskHandle_t kalmanTask;

bool IsSerialBusy = false;

void configureIMU() {
  I2Cone.begin(21, 19);
  I2Cone.setClock(400000);

  int status;

  while (!Serial) {}
  /* start the sensors */
  status = accel.begin();
  if (status < 0) {
    Serial.println("Accel Initialization Error");
    Serial.println(status);
    while (1) {}
  }
  status = gyro.begin();
  if (status < 0) {
    Serial.println("Gyro Initialization Error");
    Serial.println(status);
    while (1) {}
  }

  Serial.println("IMU Initialized");

  // xTaskCreatePinnedToCore(
  //     Core0Loop, /* Function to implement the task */
  //     "Kalman Filter Debug", /* Name of the task */
  //     10000,  /* Stack size in words */
  //     NULL,  /* Task input parameter */
  //     0,  /* Priority of the task */
  //     &kalmanDebugTask,  /* Task handle. */
  //     0); /* Core where the task should run */

  timer = micros();
}


// Derived from: https://github.com/TKJElectronics/KalmanFilter
void KalmanFilter() {
  double dt = (double)(micros() - timer) / 1000000;  // Calculate delta time
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
#ifdef RESTRICT_PITCH  // Eq. 25 and 26
  double roll = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else  // Eq. 28 and 29
  double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0;  // Convert to deg/s
  double gyroYrate = gyroY / 131.0;  // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);  // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate;  // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);  // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate;                           // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);  // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt;  // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll;  // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
}

void Core0Loop(void *parameter) {
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
  /* read the accel */
  accel.readSensor();
  /* read the gyro */
  gyro.readSensor();
  /* print the data */
  accX = (float)accel.getAccelX_mss() - ax_offset;
  accY = (float)accel.getAccelY_mss() - ay_offset;
  accZ = (float)accel.getAccelZ_mss() - az_offset;
  tempRaw = (float)accel.getTemperature_C();
  gyroX = (float)gyro.getGyroX_rads() - gx_offset;
  gyroY = (float)gyro.getGyroY_rads() - gy_offset;
  gyroZ = (float)gyro.getGyroZ_rads() - gz_offset;

  KalmanFilter();
}









//PID Loop
//x is roll
//kalanglex, kalangley
//gyrox, gyroy
//x is pitch
float maxRollRate = 30.0;   //maximum roll rate in angles/second
float maxPitchRate = 30.0;  //maximum pitch rate in angles/second
float p_roll = 0.2;
float i_roll = 0.3;
float d_roll = 0.01;
float p_pitch = 0.2;
float i_pitch = 0.3;
float d_pitch = 0.01;
float p_yaw = 0.1;
float i_yaw = 0.1;
float d_yaw = 0.1;
float pitch_des, roll_des, yaw_des;
float error_roll, error_roll_prev, roll_jerk, integral_roll = 0;
float error_pitch, error_pitch_prev, pitch_jerk, integral_pitch = 0;
float error_yaw, error_yaw_prev, yaw_jerk, integral_yaw = 0;
float roll_pid, pitch_pid, yaw_pid;
float pid_time_prev = 0;
float pid_time_delta;

volatile int16_t rollCommand = 500;
volatile int16_t pitchCommand = 500;
volatile int16_t yawCommand = 500;

void rate_PID() {
  pid_time_delta = millis() - pid_time_prev;  //should just be one most times
  pid_time_prev = millis();

  roll_des = (rollCommand - 500) / 500.0;
  pitch_des = (pitchCommand - 500) / 500.0;
  yaw_des = (yawCommand - 500) / 500.0;

  error_roll = roll_des + gyroX;
  error_pitch = pitch_des - gyroY;
  error_yaw = yaw_des + gyroZ;

  roll_jerk = (error_roll - error_roll_prev) / pid_time_delta;
  pitch_jerk = (error_pitch - error_pitch_prev) / pid_time_delta;
  yaw_jerk = (error_yaw - error_yaw_prev) / pid_time_delta;

  integral_roll = integral_roll + error_roll * pid_time_delta;
  integral_pitch = integral_pitch + error_pitch * pid_time_delta;
  integral_yaw = integral_yaw + error_yaw * pid_time_delta;

  pitch_pid = p_pitch * error_pitch + d_pitch * pitch_jerk;  //not adding the integral term now
  roll_pid = p_roll * error_roll + d_roll * roll_jerk;       //not adding the integral term now
  yaw_pid = p_yaw * error_yaw + d_yaw * yaw_jerk;

  error_roll_prev = error_roll;
  error_pitch_prev = error_pitch;
  error_yaw_prev = error_yaw;

  //Debug
  // Serial.print("pitch_Des "); Serial.print(pitch_des);
  // Serial.print("\t roll_des "); Serial.print(roll_des);
  // Serial.print("\t error_roll "); Serial.print(error_roll);
  // Serial.print("\t error_pitch "); Serial.print(error_pitch);
  // Serial.print("\t gyroX "); Serial.print(gyroX);
  // Serial.print("\t gyroY "); Serial.print(gyroY);
  // Serial.print("\t pitch_jerk "); Serial.print(pitch_jerk);
  Serial.print("roll_jerk ");
  Serial.print(roll_jerk);
  Serial.print("\t roll_pid ");
  Serial.println(roll_pid);
  // Serial.print("\t yaw_pid"); Serial.println(pitch_pid);
}





// SBUS for rx and tx comms to radio receiver
/* Define the serial port and pins for the ESP32 */
HardwareSerial &serialPort = Serial2;
const int8_t RX_PIN = 18;
const int8_t TX_PIN = 12;  // Not used

const int JOYSTICK_LOW = 174;
const int JOYSTICK_HIGH = 1811;

volatile int flightMode;

TaskHandle_t receiverTask;

long lastReceived = 0;

/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&serialPort, RX_PIN, TX_PIN, true);
bfs::SbusTx sbus_tx(&serialPort, RX_PIN, TX_PIN, true);
/* SBUS data */
bfs::SbusData data;

int16_t scaleCommand(int command) {
  long constrainedCommand = constrain(command, JOYSTICK_LOW, JOYSTICK_HIGH);
  return (int16_t)map(constrainedCommand, JOYSTICK_LOW, JOYSTICK_HIGH, 0, 1000);
}

void configureReceiver() {
  /* Begin the SBUS communication */
  sbus_rx.Begin();
  sbus_tx.Begin();  // Not used
}

long rxMillis = 0;
bool startup = true;

void readReceiver(void *parameter) {
  while (true) {
    if (sbus_rx.Read()) {
      /* Grab the received data */
      data = sbus_rx.data();

      if (data.NUM_CH < 16) {
        continue;
      }

      rollCommand = 1000 - scaleCommand(data.ch[0]);  // rollCommand is inverted
      pitchCommand = scaleCommand(data.ch[1]);        // pitch
      yawCommand = scaleCommand(data.ch[3]);          // yaw

      if (data.ch[4] != 191) {
        throttleCommand = scaleCommand(data.ch[2]);  // throttle
      } else {
        throttleCommand = 0;
      }

      if (data.ch[5] == 191) {
        flightMode = 0;  // Swashplateless mode
      } else if (data.ch[5] == 997) {
        flightMode = 1;  // Normal mode
      }

      if (data.lost_frame == 0) {
        lastReceived = millis();
      }

      if (millis() - lastReceived > 500) {
        abort();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}








// SD Card
TaskHandle_t loggingTask;
TaskHandle_t debugTask;

String message;

int sck = 16;
int miso = 4;
int mosi = 17;
int cs = 5;

// int SD_Led = 4;

int msgCount = 0;
long receivedMessages = 0;

const size_t SD_QUEUE_SIZE = 1000;  // A queue size of 3000 causes stability issues
String SDmessageQueue[SD_QUEUE_SIZE];
volatile size_t SDhead = 0;  // Write index
volatile size_t SDtail = 0;  // Read index

File file;

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
    // Drop the oldest message
    SDtail = (SDtail + 1) % SD_QUEUE_SIZE;
  }

  SDmessageQueue[SDhead] = message;
  SDhead = (SDhead + 1) % SD_QUEUE_SIZE;
}

bool processSDMessages(fs::FS &fs) {
  if (!file) {
    Serial.println("Failed to open file for appending!");
    return false;
  }

  while (!isSDQueueEmpty()) {
    file.println(SDmessageQueue[SDtail]);
    SDtail = (SDtail + 1) % SD_QUEUE_SIZE;
    receivedMessages++;
  }
  file.flush();

  return true;
}

void writeToSDCard(void *args) {
  while (true) {
    if (!processSDMessages(SD)) {
      vTaskDelete(NULL);
    }
    // vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void configureSDcard() {
  Serial.println("Configuring SD card...");
  SPI.begin(sck, miso, mosi, cs);

  // pinMode(SD_Led, OUTPUT);

  if (!SD.begin(cs, SPI)) {
    while (true) {
      Serial.println("Card Mount Failed");
      delay(200);
    }
  }
  Serial.println("Card mounted");

  File root = SD.open("/");  // Open at root directory

  if (!root) {
    Serial.println("Could not open root directory!");
  }

  File head = root.openNextFile();
  int fileCount = 1;

  while (head) {
    fileCount++;
    head = root.openNextFile();
  }

  fileName = String(fileCount) + ".csv";
  File newfile = SD.open("/" + fileName, FILE_WRITE);
  newfile.close();

  file = SD.open("/" + fileName, FILE_APPEND);

  xTaskCreatePinnedToCore(
    writeToSDCard,
    "SD card logging",
    10000,
    NULL,
    1,
    &loggingTask,
    0);
}









#define I2C_DEV_ADDR_1 0x55
#define I2C_DEV_ADDR_2 0x60

// hw_timer_t *Timer0_Cfg = NULL;

TaskHandle_t i2cTask;

union int16Bytes {
  byte b[2];
  int16_t n;
};

union int16Bytes nums[3];
byte i2cPacket[6];

volatile uint32_t currentMillis;

long timeoutSum;
long droppedMessages;
long totalMessages;
long i2cLastReceived;
long sentMessages;

void i2cSend() {
  nums[0].n = throttleCommand;

  if (flightMode == 0) {
    nums[1].n = constrain((roll_pid * 3 * 1000) + 500, 0, 1000);
  } else if (flightMode == 1) {
    nums[1].n = 500;
  }

  nums[2].n = yawCommand;

  for (int i = 0; i < 3; i++) {
    i2cPacket[i * 2] = nums[i].b[1];      // MSB first
    i2cPacket[i * 2 + 1] = nums[i].b[0];
  }

  I2Ctwo.beginTransmission(I2C_DEV_ADDR_1);
  I2Ctwo.write(i2cPacket, 6);
  int error = I2Ctwo.endTransmission(true);

  if (error != 0) {
    droppedMessages++;
  } else {
    sentMessages++;
  }

  I2Ctwo.beginTransmission(I2C_DEV_ADDR_2);
  I2Ctwo.write(i2cPacket, 6);
  error = I2Ctwo.endTransmission(true);

  if (error != 0) {
    droppedMessages++;
  } else {
    sentMessages++;
  }

  totalMessages++;
  esp_task_wdt_reset();
}

void i2cSetup() {
  I2Ctwo.begin(26, 25);
  I2Ctwo.setClock(1000000);  // 1mhz clock

  // xTaskCreatePinnedToCore(
  //   i2cSend,  /* Function to implement the task */
  //   "i2c TX", /* Name of the task */
  //   2000,     /* Stack size in words */
  //   NULL,     /* Task input parameter */
  //   2,        /* Priority of the task */
  //   &i2cTask,
  //   0); /* Core where the task should run */
}









// Servos and control surfaces
const int SERVO_0 = 32;  // left
const int SERVO_1 = 33;  // right
const int SERVO_PWM_PERIOD = 333;
const int SERVO_PWM_RESOLUTION = 12;

long aileronLeft;
long aileronRight;

void armServos() {
  if (!ledcAttach(SERVO_0, SERVO_PWM_PERIOD, SERVO_PWM_RESOLUTION)) {
    Serial.println("Failed to arm servo 0");
    while (true) {}
  }

  if (!ledcAttach(SERVO_1, SERVO_PWM_PERIOD, SERVO_PWM_RESOLUTION)) {
    Serial.println("Failed to arm servo 1");
    while (true) {}
  }
}

void commandServos() {
  if (flightMode == 0) {
    long servoLeft = pitchCommand;
    long servoRight = 1000 - pitchCommand;

    aileronLeft = -pitch_pid * 250 + servoLeft;
    aileronRight = pitch_pid * 250 + servoRight;

    aileronLeft = map(aileronLeft, 0, 1000, 820, 3200);
    aileronRight = map(aileronRight, 0, 1000, 820, 3200);

  } else if (flightMode == 1) {
    long servoLeft = rollCommand + (pitchCommand - 500);
    long servoRight = rollCommand - (pitchCommand - 500);

    aileronLeft = roll_pid * 250 - pitch_pid * 250 + servoLeft;
    aileronRight = roll_pid * 250 + pitch_pid * 250 + servoRight;

    aileronLeft = map(aileronLeft, 0, 1000, 820, 3200);
    aileronRight = map(aileronRight, 0, 1000, 820, 3200);
  }

  // Debug
  // Serial.print("left "); Serial.print(aileronLeft);
  // Serial.print("   right "); Serial.println(aileronRight);

  ledcWrite(SERVO_1, aileronLeft);
  ledcWrite(SERVO_0, aileronRight);
}









// Setup and main loop

long currentTimeFromStart;
long startTime;

void setup() {
  Serial.begin(115200);

  Serial.println("Starting...");

  configureReceiver();

  xTaskCreate(
    readReceiver,    /* Function to implement the task */
    "read receiver", /* Name of the task */
    2000,            /* Stack size in words */
    NULL,            /* Task input parameter */
    1,               /* Priority of the task */
    &receiverTask);  /* Core where the task should run */


  delay(1000);  // Wait to see if the killswitch is on and/or if the controller is connected. If not, receiver task will timeout and reboot

  if (millis() - lastReceived > 500) {
    abort();
  }

  configureSDcard();
  configureIMU();

  // xTaskCreate(
  //   Core0Loop, /* Function to implement the task */
  //   "Kalman Filter Debug", /* Name of the task */
  //   5000,  /* Stack size in words */
  //   NULL,  /* Task input parameter */
  //   0,  /* Priority of the task */
  //   &kalmanDebugTask);

  i2cSetup();
  armServos();

  // Configure Watchdog
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WATCHDOG_TIMEOUT * 1000,  // Timeout in milliseconds
    .trigger_panic = true
  };

  esp_task_wdt_deinit();
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);
  // esp_task_wdt_add(receiverTask); // this is not working for some reason
  esp_task_wdt_add(i2cTask);

  enqueueSDMessage("micros,throttleCommand,rollCommand,pitchCommand,yawCommand,kalAngleX,kalAngleY,gyroX,gyroY,gyroZ,roll_pid,pitch_pid,yaw_pid,loopRate,flightMode");

  Serial.println("Setup Finished");

  startTime = millis();
  // sendDShotCommand(0, false);
  // delay(200);
  // sendDShotCommand(100, false);
  // delay(200);
  // sendDShotCommand(0, false);
}

float phase_delay = 0;
long prevPrintTime = 0;
float totalLoops = 0;


void loop() {
  currentTimeFromStart = millis() - startTime;

  // if (millis() - prevPrintTime > 100) {
  //   // processSDMessages(SD);
  //   Serial.print("millis: ");
  //   Serial.print(currentMillis);
  //   Serial.print("\t throttle: ");
  //   Serial.print(throttleCommand);
  //   Serial.print("\t loop rate: ");
  //   Serial.print(totalLoops / (currentTimeFromStart / 1000.0));
  //   Serial.print("\t i2c sends/sec: ");
  //   Serial.print(sentMessages / (currentTimeFromStart / 1000.0));
  //   Serial.print("\t i2c drops/sec: ");
  //   Serial.print(droppedMessages / (currentTimeFromStart / 1000.0));
  //   Serial.print("\t SD card logs/sec: ");
  //   Serial.println(receivedMessages / (currentTimeFromStart / 1000.0));

  //   prevPrintTime = millis();
  // }

  updateIntertialData();
  rate_PID();

  char buf[128];

  snprintf(buf, sizeof(buf), "%lu,%lu,%lu,%lu,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%lu",
           micros(), throttleCommand, rollCommand, pitchCommand, yawCommand, kalAngleX,
           kalAngleY, gyroX, gyroY, gyroZ, roll_pid, pitch_pid, yaw_pid,
           totalLoops / (currentTimeFromStart / 1000.0), flightMode);

  enqueueSDMessage(String(buf));

  if (data.ch[4] == 191) {
    throttleCommand = 0;
  } else {
    i2cSend();
  }

  if (millis() - lastReceived > 500) {
    abort();
  }

  commandServos();


  totalLoops++;

  esp_task_wdt_reset();
}