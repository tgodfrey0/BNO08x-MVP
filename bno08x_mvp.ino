#include <Wire.h>

#define ADDR 0x4a

static float x = 0;
static float y = 0;
static float z = 0;
static uint8_t seq_num = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(115200);

  // Turn sensor on
  uint8_t open_pkt_size = 5;
  uint8_t open_pkt[open_pkt_size] = { 5, 0, 1, seq_num++, 2 };

  Wire.beginTransmission(ADDR);
  Wire.write(open_pkt, open_pkt_size);
  Wire.endTransmission();

  Serial.println("On command sent");

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
  Serial.println("Accelerometer enable command sent");
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
  Serial.println("Gyroscope enable command sent");
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
  Serial.println("Magnetometer enable command sent");
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
    Serial.println("1000 iterations finished");
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

  if (data[9] == 0x01) {  // Accelerometer
    x = (data[13] | (data[14] << 8)) * (1.0 / (1 << 8));
    y = (data[15] | (data[16] << 8)) * (1.0 / (1 << 8));
    z = (data[17] | (data[18] << 8)) * (1.0 / (1 << 8));
    Serial.print(count);
    Serial.print(" - Accelerometer (");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(", ");
    Serial.print(z);
    Serial.println(") m/s^2");
  } else if (data[9] == 0x02) {  // Gyroscope
    x = (data[13] | (data[14] << 8)) * (1.0 / (1 << 9));
    y = (data[15] | (data[16] << 8)) * (1.0 / (1 << 9));
    z = (data[17] | (data[18] << 8)) * (1.0 / (1 << 9));
    Serial.print(count);
    Serial.print(" - Gyroscope (");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(", ");
    Serial.print(z);
    Serial.println(") rad/s");
    // Serial.print(x);
    // Serial.print(",");
    // Serial.print(y);
    // Serial.print(",");
    // Serial.print(z);
    // Serial.println("");
  } else if (data[9] == 0x03) {  // Magnetometer
    x = (data[13] | (data[14] << 8)) * (1.0 / (1 << 4));
    y = (data[15] | (data[16] << 8)) * (1.0 / (1 << 4));
    z = (data[17] | (data[18] << 8)) * (1.0 / (1 << 4));
    Serial.print(count);
    Serial.print(" - Magnetometer (");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(", ");
    Serial.print(z);
    Serial.println(") uT");
  } else {
    // Serial.print(count);
    // Serial.print(" - Unrecognised ID: ");
    // Serial.println(data[9]);
  }
  // Serial.println("----");
  //for(;;);
  delay(50); // Any shorter than 50ms and random sensors get detected
}
