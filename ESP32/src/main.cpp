// Basic demo for accelerometer readings from Adafruit ICM20948

#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_ICM20948 icm;
uint16_t measurement_delay_us = 65535; // Delay between measurements for testing
// For SPI mode, we need a CS pin
#define ICM_CS 5
// For software-SPI mode we need SCK/MOSI/MISO pins
#define ICM_SCK 18
#define ICM_MISO 19
#define ICM_MOSI 23

//MUISTIPAIKAT MAG DATALLE KUN KÄYTETÄÄN SPI:TA
#define I2C_SLV0_ADDR 0x03
#define I2C_SLV0_REG 0x04
#define I2C_SLV0_CTRL 0x05
#define AK09916_ADDR 0x0C

bool enablePrint = false;

// SPI-kirjoitus- ja lukufunktiot
void writeToRegister(uint8_t reg, uint8_t data) {
  digitalWrite(ICM_CS, LOW); // Aktivoi sirun valinta
  SPI.transfer(reg); // Lähetä rekisterin osoite
  SPI.transfer(data); // Lähetä data
  digitalWrite(ICM_CS, HIGH); // Deaktivoi sirun valinta
}
uint8_t readFromRegister(uint8_t reg) {
  digitalWrite(ICM_CS, LOW); // Aktivoi sirun valinta
  SPI.transfer(reg | 0x80); // Lähetä rekisterin osoite ja aseta MSB:ksi 1 lukemista varten
  uint8_t data = SPI.transfer(0x00); // Lue data
  digitalWrite(ICM_CS, HIGH); // Deaktivoi sirun valinta
  return data; // Palauta luetut tiedot
}

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit ICM20948 test!");
  
  //Wire.setPins(18, 19);
  //Wire.begin();
    // Alusta SPI-väylä
  SPI.begin(ICM_SCK, ICM_MISO, ICM_MOSI, ICM_CS);
  //SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0)); // Aseta SPI-asetukset

  // Alusta ESP32:n pinnit
  pinMode(ICM_CS, OUTPUT);
  digitalWrite(ICM_CS, HIGH); // Deaktivoi sirun valinta
  //VAihdetaan 3 bank
  writeToRegister(0x7F, 0x30);
  
  uint8_t value1 = readFromRegister(I2C_SLV0_ADDR);
  delay(1000);
  writeToRegister(I2C_SLV0_ADDR, AK09916_ADDR);
  uint8_t value2 = readFromRegister(I2C_SLV0_ADDR);


/*   if (!icm.setupMag()) {
    Serial.println("failed to setup mag");
  }
  else {
    Serial.println("mag setup OK");
  } */

  SPI.end();
  Serial.print("I2C_SLV0_ADDR: ");
  Serial.println(value1);
  Serial.print("I2C_SLV0_ADDR: ");
  Serial.println(value2);


  // Try to initialize!
  //if (!icm.begin_I2C()) {
    //if (!icm.begin_SPI(ICM_CS)) {
  if (!icm.begin_SPI(ICM_CS, ICM_SCK, ICM_MISO, ICM_MOSI)) {
    Serial.println("Failed to find ICM20948 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("ICM20948 Found!");
  // icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
  Serial.print("Accelerometer range set to: ");
  switch (icm.getAccelRange()) {
  case ICM20948_ACCEL_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case ICM20948_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case ICM20948_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case ICM20948_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  Serial.println("OK");

  // icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
  Serial.print("Gyro range set to: ");
  switch (icm.getGyroRange()) {
  case ICM20948_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  }

  //  icm.setAccelRateDivisor(4095);
  uint16_t accel_divisor = icm.getAccelRateDivisor();
  float accel_rate = 1125 / (1.0 + accel_divisor);

  Serial.print("Accelerometer data rate divisor set to: ");
  Serial.println(accel_divisor);
  Serial.print("Accelerometer data rate (Hz) is approximately: ");
  Serial.println(accel_rate);

  //  icm.setGyroRateDivisor(255);
  uint8_t gyro_divisor = icm.getGyroRateDivisor();
  float gyro_rate = 1100 / (1.0 + gyro_divisor);

  Serial.print("Gyro data rate divisor set to: ");
  Serial.println(gyro_divisor);
  Serial.print("Gyro data rate (Hz) is approximately: ");
  Serial.println(gyro_rate);

  // icm.setMagDataRate(AK09916_MAG_DATARATE_10_HZ);
  Serial.print("Magnetometer data rate set to: ");
  switch (icm.getMagDataRate()) {
  case AK09916_MAG_DATARATE_SHUTDOWN:
    Serial.println("Shutdown");
    break;
  case AK09916_MAG_DATARATE_SINGLE:
    Serial.println("Single/One shot");
    break;
  case AK09916_MAG_DATARATE_10_HZ:
    Serial.println("10 Hz");
    break;
  case AK09916_MAG_DATARATE_20_HZ:
    Serial.println("20 Hz");
    break;
  case AK09916_MAG_DATARATE_50_HZ:
    Serial.println("50 Hz");
    break;
  case AK09916_MAG_DATARATE_100_HZ:
    Serial.println("100 Hz");
    break;
  }
  Serial.println();

}

void loop() {

  //  /* Get a new normalized sensor event */
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag;
  sensors_event_t temp;
  icm.getEvent(&accel, &gyro, &temp, &mag);
  if (enablePrint)
  {
    Serial.print("\t\tTemperature ");
    Serial.print(temp.temperature);
    Serial.println(" deg C");

    /* Display the results (acceleration is measured in m/s^2) */
    Serial.print("\t\tAccel X: ");
    Serial.print(accel.acceleration.x);
    Serial.print(" \tY: ");
    Serial.print(accel.acceleration.y);
    Serial.print(" \tZ: ");
    Serial.print(accel.acceleration.z);
    Serial.println(" m/s^2 ");

    Serial.print("\t\tMag X: ");
    Serial.print(mag.magnetic.x);
    Serial.print(" \tY: ");
    Serial.print(mag.magnetic.y);
    Serial.print(" \tZ: ");
    Serial.print(mag.magnetic.z);
    Serial.println(" uT");

    /* Display the results (acceleration is measured in m/s^2) */
    Serial.print("\t\tGyro X: ");
    Serial.print(gyro.gyro.x);
    Serial.print(" \tY: ");
    Serial.print(gyro.gyro.y);
    Serial.print(" \tZ: ");
    Serial.print(gyro.gyro.z);
    Serial.println(" radians/s ");
    Serial.println();

    delay(100);

    //  Serial.print(temp.temperature);
    //
    //  Serial.print(",");
    //
    //  Serial.print(accel.acceleration.x);
    //  Serial.print(","); Serial.print(accel.acceleration.y);
    //  Serial.print(","); Serial.print(accel.acceleration.z);
    //
    //  Serial.print(",");
    //  Serial.print(gyro.gyro.x);
    //  Serial.print(","); Serial.print(gyro.gyro.y);
    //  Serial.print(","); Serial.print(gyro.gyro.z);
    //
    //  Serial.print(",");
    //  Serial.print(mag.magnetic.x);
    //  Serial.print(","); Serial.print(mag.magnetic.y);
    //  Serial.print(","); Serial.print(mag.magnetic.z);

    //  Serial.println();
    //
    //  delayMicroseconds(measurement_delay_us);
  }
}