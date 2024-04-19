/****************************************************************
 * Example1_Basics.ino
 * ICM 20948 Arduino Library Demo
 * Use the default configuration to stream 9-axis IMU data
 * Owen Lyke @ SparkFun Electronics
 * Original Creation Date: April 17 2019
 *
 * Please see License.md for the license information.
 *
 * Distributed as-is; no warranty is given.
 ***************************************************************/
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#include <Arduino_JSON.h>
#include "SPIFFS.h"

#include "FS.h"
#include "SD.h"
#include "SPI.h"

#define USE_SPI       // Uncomment this to use SPI

#define SERIAL_PORT Serial

#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 5     // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

#define LED1 26
#define LED2 27

#define BTN1 10
#define BTN2 9

int btn1State = HIGH;
int btn2State = HIGH;

int btn1StateOld = HIGH;
int btn2StateOld = HIGH;
bool bothButtonsPressed = false;

bool wifi = false;
bool IMU = false;

#ifdef USE_SPI
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
#endif
void printScaledAGMT(ICM_20948_SPI *sensor);
void printSensorDataFloat(ICM_20948_SPI *sensor);
void printRawAGMT(ICM_20948_AGMT_t agmt);
void getSensorData(ICM_20948_SPI *sensor);
void printSensorData();

SPIClass spi = SPIClass(HSPI); //SD Card SPI

// Replace with your network credentials
const char* ssid = "WW";
const char* password = "esp32web";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Create an Event Source on /events
AsyncEventSource events("/events");

// Json Variable to Hold Sensor Readings
JSONVar readings;

// Timer variables
unsigned long lastTime = 0;  
unsigned long lastTimeTemperature = 0;
unsigned long lastTimeAcc = 0;
unsigned long gyroDelay = 10;
unsigned long temperatureDelay = 1000;
unsigned long accelerometerDelay = 200;

float gyroX, gyroY, gyroZ;
float accX, accY, accZ;
float magX, magY, magZ;
float temperature;

//Gyroscope sensor deviation
float gyroXerror = 0.07;
float gyroYerror = 0.03;
float gyroZerror = 0.01;

void initMPU(){
  #ifdef USE_SPI
  SPI_PORT.begin();
#else
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
#endif

  //myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  bool initialized = false;
  while (!initialized)
  {

#ifdef USE_SPI
    myICM.begin(CS_PIN, SPI_PORT);
#else
    myICM.begin(WIRE_PORT, AD0_VAL);
#endif

    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }
}

void initSDCard(){
  spi.begin(14,12,13,15);
  if(!SD.begin(15,spi,8000000)){
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
    Serial.println("No SD card attached");
    return;
  }
  Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC){
    Serial.println("MMC");
  } else if(cardType == CARD_SD){
    Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
}

void initSPIFFS() {
  if (!SPIFFS.begin()) {
    SERIAL_PORT.println("An error has occurred while mounting SPIFFS");
  }
  SERIAL_PORT.println("SPIFFS mounted successfully");
}

// Initialize WiFi
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  SERIAL_PORT.println("");
  SERIAL_PORT.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    SERIAL_PORT.print(".");
    delay(1000);
  }
  SERIAL_PORT.println("");
  SERIAL_PORT.println(WiFi.localIP());
}

String getGyroReadings(){
  /* float gyroX_temp = gyroX;
  if(abs(gyroX_temp) > gyroXerror)  {
    gyroX += gyroX_temp/50.00;
  }
  
  float gyroY_temp = gyroY;
  if(abs(gyroY_temp) > gyroYerror) {
    gyroY += gyroY_temp/70.00;
  }

  float gyroZ_temp = gyroZ;
  if(abs(gyroZ_temp) > gyroZerror) {
    gyroZ += gyroZ_temp/90.00;
  } */
  readings["gyroX"] = String(gyroX);
  readings["gyroY"] = String(gyroY);
  readings["gyroZ"] = String(gyroZ);

  String jsonString = JSON.stringify(readings);
  return jsonString;
}

String getAccReadings() {
  readings["accX"] = String(accX);
  readings["accY"] = String(accY);
  readings["accZ"] = String(accZ);
  String accString = JSON.stringify (readings);
  return accString;
}

String getMagReadings() {
  readings["magX"] = String(magX);
  readings["magY"] = String(magY);
  readings["magZ"] = String(magZ);
  String accString = JSON.stringify (readings);
  return accString;
}
String getTemperature(){
  return String(temperature);
}

void setupWiFi(){
  // Handle Web Server
   if(wifi)
  {
    initWiFi();
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SD, "/index.html", "text/html");
  });

  server.serveStatic("/", SD, "/");

  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request){
    gyroX=0;
    gyroY=0;
    gyroZ=0;
    request->send(200, "text/plain", "OK");
  });

  server.on("/resetX", HTTP_GET, [](AsyncWebServerRequest *request){
    gyroX=0;
    request->send(200, "text/plain", "OK");
  });

  server.on("/resetY", HTTP_GET, [](AsyncWebServerRequest *request){
    gyroY=0;
    request->send(200, "text/plain", "OK");
  });

  server.on("/resetZ", HTTP_GET, [](AsyncWebServerRequest *request){
    gyroZ=0;
    request->send(200, "text/plain", "OK");
  });

  // Handle Web Server Events
  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 10000);
  });
  server.addHandler(&events);

  server.begin();
  }

}

void setup()
{
  pinMode(LED1,OUTPUT);
  pinMode(LED2,OUTPUT);
  pinMode(BTN1, INPUT_PULLUP);
  pinMode(BTN2, INPUT_PULLUP); 
  SERIAL_PORT.begin(115200);
  
  //initSPIFFS();  //ESP32 internal file system
  initMPU();
  initSDCard();

  
}

void loop()
{
  // read the state of the switch/button:
  btn1State = digitalRead(BTN1);
  btn2State = digitalRead(BTN2);

  if(btn1StateOld == HIGH && btn1State == LOW)
  {
    Serial.println("Button 1 pressed");
    digitalWrite(LED2, HIGH);
    IMU = true;
    delay(100);
  }
  btn1StateOld = btn1State;
   if(btn2StateOld == HIGH && btn2State == LOW)
  {
    Serial.println("Button 2 pressed");
    digitalWrite(LED2, LOW);
    IMU = false;
    delay(100);
  }
  btn2StateOld = btn2State;

  if(btn1State == LOW && btn2State == LOW)
  {
    Serial.println("Both buttons pressed");
    wifi = !wifi;
    Serial.println("Wifi boolean: " + String(wifi));
    if(wifi)
    {
      setupWiFi();
    }
    else
    {
      server.end();
      Serial.println("Server stopped");
      WiFi.disconnect();
      Serial.println("Wifi disconnected");
    }
    delay(100);
  }

  // Read the sensor data
  if(IMU)
  {
     if (myICM.dataReady())
  {
    myICM.getAGMT();
    printRawAGMT( myICM.agmt );
    //getSensorData(&myICM);
    //printSensorData();
    delay(100);
  }
  else
  {
    SERIAL_PORT.println("Waiting for data");
    delay(500);
  }
  }
  

  //Send data to the web server

  if(wifi)
  {
    if ((millis() - lastTime) > gyroDelay) {
    // Send Events to the Web Server with the Sensor Readings
    events.send(getGyroReadings().c_str(),"gyro_readings",millis());
    lastTime = millis();
  }
  if ((millis() - lastTimeAcc) > accelerometerDelay) {
    // Send Events to the Web Server with the Sensor Readings
    events.send(getAccReadings().c_str(),"accelerometer_readings",millis());
    lastTimeAcc = millis();
  }
  if ((millis() - lastTimeTemperature) > temperatureDelay) {
    // Send Events to the Web Server with the Sensor Readings
    events.send(getTemperature().c_str(),"temperature_reading",millis());
    lastTimeTemperature = millis();
  }
  }

  //Turn led1 on that indicates we are end of the loop function
  digitalWrite(LED1, HIGH);
}

void getSensorData(ICM_20948_SPI *sensor)
{
  
  temperature = sensor->temp();

  accX = sensor->accX() * 0.00980665;
  accY = sensor->accY() * 0.00980665;
  accZ = sensor->accZ() * 0.00980665;

  gyroX = sensor->gyrX() * 0.0174532925; //radians/s
  gyroY = sensor->gyrY() * 0.0174532925;
  gyroZ = sensor->gyrZ() * 0.0174532925;

  magX = sensor->magX();
  magY = sensor->magY();
  magZ = sensor->magZ();
  
}

void printSensorData()
{
  SERIAL_PORT.print("\t\tTemperature ");
  SERIAL_PORT.print(temperature);
  SERIAL_PORT.println(" deg C");

  SERIAL_PORT.print("\t\tAccel X: ");
  SERIAL_PORT.print(accX);
  SERIAL_PORT.print(" \tY: ");
  SERIAL_PORT.print(accY);
  SERIAL_PORT.print(" \tZ: ");
  SERIAL_PORT.print(accZ);
  SERIAL_PORT.println(" m/s^2 ");

  SERIAL_PORT.print("\t\tGyro X: ");
  SERIAL_PORT.print(gyroX);
  SERIAL_PORT.print(" \tY: ");
  SERIAL_PORT.print(gyroY);
  SERIAL_PORT.print(" \tZ: ");
  SERIAL_PORT.print(gyroZ);
  SERIAL_PORT.println(" rpm");

  SERIAL_PORT.print("\t\tMag X: ");
  SERIAL_PORT.print(magX);
  SERIAL_PORT.print(" \tY: ");
  SERIAL_PORT.print(magY);
  SERIAL_PORT.print(" \tZ: ");
  SERIAL_PORT.print(magZ);
  SERIAL_PORT.println(" uT");
  SERIAL_PORT.println();
}




// Below here are some helper functions to print the data nicely!

void printPaddedInt16b(int16_t val)
{
  if (val > 0)
  {
    SERIAL_PORT.print(" ");
    if (val < 10000)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 1000)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 100)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 10)
    {
      SERIAL_PORT.print("0");
    }
  }
  else
  {
    SERIAL_PORT.print("-");
    if (abs(val) < 10000)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 1000)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 100)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 10)
    {
      SERIAL_PORT.print("0");
    }
  }
  SERIAL_PORT.print(abs(val));
}

void printRawAGMT(ICM_20948_AGMT_t agmt)
{
  SERIAL_PORT.print("RAW. Acc [ ");
  printPaddedInt16b(agmt.acc.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.acc.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.acc.axes.z);
  SERIAL_PORT.print(" ], Gyr [ ");
  printPaddedInt16b(agmt.gyr.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.gyr.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.gyr.axes.z);
  SERIAL_PORT.print(" ], Mag [ ");
  printPaddedInt16b(agmt.mag.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.mag.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.mag.axes.z);
  SERIAL_PORT.print(" ], Tmp [ ");
  printPaddedInt16b(agmt.tmp.val);
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals)
{
  float aval = abs(val);
  if (val < 0)
  {
    SERIAL_PORT.print("-");
  }
  else
  {
    SERIAL_PORT.print(" ");
  }
  for (uint8_t indi = 0; indi < leading; indi++)
  {
    uint32_t tenpow = 0;
    if (indi < (leading - 1))
    {
      tenpow = 1;
    }
    for (uint8_t c = 0; c < (leading - 1 - indi); c++)
    {
      tenpow *= 10;
    }
    if (aval < tenpow)
    {
      SERIAL_PORT.print("0");
    }
    else
    {
      break;
    }
  }
  if (val < 0)
  {
    SERIAL_PORT.print(-val, decimals);
  }
  else
  {
    SERIAL_PORT.print(val, decimals);
  }
}

#ifdef USE_SPI
void printScaledAGMT(ICM_20948_SPI *sensor)
{
#else
void printScaledAGMT(ICM_20948_I2C *sensor)
{
#endif
  SERIAL_PORT.print("\t\tTemperature ");
  printFormattedFloat(sensor->temp(), 5, 2);
  SERIAL_PORT.println(" deg C");

  SERIAL_PORT.print("\t\tAccel X: ");
  printFormattedFloat(sensor->accX() * 0.00980665, 5, 2);
  SERIAL_PORT.print(" \tY: ");
  printFormattedFloat(sensor->accY() * 0.00980665, 5, 2);
  SERIAL_PORT.print(" \tZ: ");
  printFormattedFloat(sensor->accZ() * 0.00980665, 5, 2);
  SERIAL_PORT.println(" m/s^2 ");

  SERIAL_PORT.print("\t\tGyro X: ");
  printFormattedFloat(sensor->gyrX() * 0.1666667, 5, 2);
  SERIAL_PORT.print(" \tY: ");
  printFormattedFloat(sensor->gyrY() * 0.1666667, 5, 2);
  SERIAL_PORT.print(" \tZ: ");
  printFormattedFloat(sensor->gyrZ() * 0.1666667, 5, 2);
  SERIAL_PORT.println(" rpm");

  SERIAL_PORT.print("\t\tMag X: ");
  printFormattedFloat(sensor->magX(), 5, 2);
  SERIAL_PORT.print(" \tY: ");
  printFormattedFloat(sensor->magY(), 5, 2);
  SERIAL_PORT.print(" \tZ: ");
  printFormattedFloat(sensor->magZ(), 5, 2);
  SERIAL_PORT.println(" uT");
  SERIAL_PORT.println();

}
void printSensorDataFloat(ICM_20948_SPI *sensor)
{
  SERIAL_PORT.print("\t\tTemperature ");
  float temp = sensor->temp();
  SERIAL_PORT.print(temp);
  SERIAL_PORT.println(" deg C");

  SERIAL_PORT.print("\t\tAccel X: ");
  float accX = sensor->accX() * 0.00980665;
  SERIAL_PORT.print(accX);
  SERIAL_PORT.print(" \tY: ");
  float accY = sensor->accY() * 0.00980665;
  SERIAL_PORT.print(accY);
  SERIAL_PORT.print(" \tZ: ");
  float accZ = sensor->accZ() * 0.00980665;
  SERIAL_PORT.print(accZ);
  SERIAL_PORT.println(" m/s^2 ");

  SERIAL_PORT.print("\t\tGyro X: ");
  float gyrX = sensor->gyrX() * 0.1666667;
  SERIAL_PORT.print(gyrX);
  SERIAL_PORT.print(" \tY: ");
  float gyrY = sensor->gyrY() * 0.1666667;
  SERIAL_PORT.print(gyrY);
  SERIAL_PORT.print(" \tZ: ");
  float gyrZ = sensor->gyrZ() * 0.1666667;
  SERIAL_PORT.print(gyrZ);
  SERIAL_PORT.println(" rpm");

  SERIAL_PORT.print("\t\tMag X: ");
  float magX = sensor->magX();
  SERIAL_PORT.print(magX);
  SERIAL_PORT.print(" \tY: ");
  float magY = sensor->magY();
  SERIAL_PORT.print(magY);
  SERIAL_PORT.print(" \tZ: ");
  float magZ = sensor->magZ();
  SERIAL_PORT.print(magZ);
  SERIAL_PORT.println(" uT");
  SERIAL_PORT.println();
}
