#include <Wire.h>

#include <ros.h>
#include <ros/time.h>
#include <bioin_tacto/raw_imu.h>

#define ADDR 0x4a
#define BAUD 250000u

ros::NodeHandle nh;

bioin_tacto::raw_imu rimu;
ros::Publisher rimu_reads("raw_imus_teensy", &rimu);

static float ax = 0;
static float ay = 0;
static float az = 0;
static float gx = 0;
static float gy = 0;
static float gz = 0;
static float mx = 0;
static float my = 0;
static float mz = 0;
static const short sensor_id = 0;
static uint8_t seq_num = 0;
static long ros_seq = 0;
static const float pressure = 0;
static const float temperature = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(BAUD);
  nh.getHardware()->setBaud(BAUD);

  nh.initNode();
  rimu.header.frame_id = "imus_frames";
  nh.advertise(rimu_reads);

  // Turn sensor on
  uint8_t open_pkt_size = 5;
  uint8_t open_pkt[open_pkt_size] = { 5, 0, 1, seq_num++, 2 };

  Wire.beginTransmission(ADDR);
  Wire.write(open_pkt, open_pkt_size);
  Wire.endTransmission();

  // Serial.println("On command sent");

  delay(100);

  // Enable sensors
  uint64_t period_us = 50 * 1000;  // Report every 50 ms
  uint8_t period[4];
  period[0] = period_us & 0xFF;
  period[1] = (period_us >> 8) & 0xFF;
  period[2] = (period_us >> 16) & 0xFF;
  period[3] = (period_us >> 24) & 0xFF;

  enableAcc(period);
  enableGyr(period);
  enableMag(period);

  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
}

void enableAcc(uint8_t* period){
  uint8_t acc_pkt_size = 0x15;
  uint8_t acc_pkt[acc_pkt_size] = {
    0x15,       // Length LSB
    0x00,       // Length MSB
    0x02,       // Channel
    seq_num++,  // Sequence number
    0xFD,       // Report ID
    0x01,       // Accelerometer sensor ID
    0x00,       // Feature flags
    0x00,       // Change sensitivity LSB
    0x00,       // Change sensitivity MSB
    period[0],  // Report interval LSB
    period[1],
    period[2],
    period[3],  // Report interval MSB
    0x00,       // Batch interval LSB
    0x00,
    0x00,
    0x00,  // Batch interval MSB
    0x00,  // Sensor-specific configuration word LSB
    0x00,
    0x00,
    0x00  // Sensor-specific configuration word MSB
  };

  Wire.beginTransmission(ADDR);
  Wire.write(acc_pkt, acc_pkt_size);
  Wire.endTransmission();
  // Serial.println("Accelerometer enable command sent");
}

void enableGyr(uint8_t* period){
  uint8_t gyro_pkt_size = 0x15;
  uint8_t gyro_pkt[gyro_pkt_size] = {
    0x15,       // Length LSB
    0x00,       // Length MSB
    0x02,       // Channel
    seq_num++,  // Sequence number
    0xFD,       // Report ID
    0x02,       // Gyro sensor ID
    0x00,       // Feature flags
    0x00,       // Change sensitivity LSB
    0x00,       // Change sensitivity MSB
    period[0],  // Report interval LSB
    period[1],
    period[2],
    period[3],  // Report interval MSB
    0x00,       // Batch interval LSB
    0x00,
    0x00,
    0x00,  // Batch interval MSB
    0x00,  // Sensor-specific configuration word LSB
    0x00,
    0x00,
    0x00  // Sensor-specific configuration word MSB
  };

  Wire.beginTransmission(ADDR);
  Wire.write(gyro_pkt, gyro_pkt_size);
  Wire.endTransmission();
  // Serial.println("Gyroscope enable command sent");
}

void enableMag(uint8_t* period){
  uint8_t mag_pkt_size = 0x15;
  uint8_t mag_pkt[mag_pkt_size] = {
    0x15,       // Length LSB
    0x00,       // Length MSB
    0x02,       // Channel
    seq_num++,  // Sequence number
    0xFD,       // Report ID
    0x03,       // Mag sensor ID
    0x00,       // Feature flags
    0x00,       // Change sensitivity LSB
    0x00,       // Change sensitivity MSB
    period[0],  // Report interval LSB
    period[1],
    period[2],
    period[3],  // Report interval MSB
    0x00,       // Batch interval LSB
    0x00,
    0x00,
    0x00,  // Batch interval MSB
    0x00,  // Sensor-specific configuration word LSB
    0x00,
    0x00,
    0x00  // Sensor-specific configuration word MSB
  };

  Wire.beginTransmission(ADDR);
  Wire.write(mag_pkt, mag_pkt_size);
  Wire.endTransmission();
  // Serial.println("Magnetometer enable command sent");
}

void loop() {
  // put your main code here, to run repeatedly:
  // Wire.requestFrom(ADDR, 4);
  // while(Wire.available()){
  //   int i = Wire.read();
  //   Serial.print("0x");
  //   Serial.println(i, HEX);
  // }

  static int count = 0;
  const static bool limit = false;

  if (count == 1000 && limit) {
    // Serial.println("1000 iterations finished");
    digitalWrite(LED_BUILTIN, LOW);
    for (;;)
      ;
  }

  count++;

  int read_size = 19;
  Wire.requestFrom(ADDR, read_size);
  int8_t data[read_size];
  for (uint8_t i = 0; i < read_size && Wire.available(); i++) {
    data[i] = Wire.read();
    // if (data[i] < 16) {
    //   Serial.print("0x0");
    // } else {
    //   Serial.print("0x");
    // }
    // Serial.print(data[i], HEX);
    // Serial.print("  ");
  }
  // Serial.println("");

  // Serial.print("Size: ");
  uint16_t s = (uint16_t)data[0] | ((uint16_t)data[1]) << 8;
  s &= ~0x8000;
  // Serial.println(s);

  if (data[9] == 0x01) {  // Accelerometer - m/s^2
    ax = (data[13] | (data[14] << 8)) * (1.0 / (1 << 8));
    ay = (data[15] | (data[16] << 8)) * (1.0 / (1 << 8));
    az = (data[17] | (data[18] << 8)) * (1.0 / (1 << 8));
  } else if (data[9] == 0x02) {  // Gyroscope - rad/s
    gx = (data[13] | (data[14] << 8)) * (1.0 / (1 << 9));
    gy = (data[15] | (data[16] << 8)) * (1.0 / (1 << 9));
    gz = (data[17] | (data[18] << 8)) * (1.0 / (1 << 9));
  } else if (data[9] == 0x03) {  // Magnetometer - uT
    mx = (data[13] | (data[14] << 8)) * (1.0 / (1 << 4));
    my = (data[15] | (data[16] << 8)) * (1.0 / (1 << 4));
    mz = (data[17] | (data[18] << 8)) * (1.0 / (1 << 4));
  } else {
    // Serial.print(count);
    // Serial.print(" - Unrecognised ID: ");
    // Serial.println(data[9]);
  }
  // Serial.println("----");
  //for(;;);

  rimu.sensor_id = sensor_id;
  rimu.ax = ax;
  rimu.ay = ay;
  rimu.az = az;
  rimu.gx = gx;
  rimu.gy = gy;
  rimu.gz = gz;
  rimu.mx = mx;
  rimu.my = my;
  rimu.mz = mz;
  rimu.tempe = temperature;
  rimu.header.seq = ros_seq;
  rimu.header.stamp = nh.now();
  rimu_reads.publish(&rimu);
  nh.spinOnce();

  ros_seq++;

  delay(50); // Any shorter than 50ms and random sensors get detected
}
