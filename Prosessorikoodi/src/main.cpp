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
//int bothButtonsPressed = HIGH;

bool wifi = false;
bool IMU = false;
bool createDataFlag = false;
int dataFileIndex;

//Stopflag variables
unsigned long stopFlagTime = 0;
unsigned long startFlagTime = 0;
bool stopFlag = false;


#ifdef USE_SPI
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
#endif

//SD:n käyttöfunktioiden julistukset
void listDir(fs::FS &fs, const char * dirname, uint8_t levels);
void createDir(fs::FS &fs, const char * path);
void removeDir(fs::FS &fs, const char * path);
void readFile(fs::FS &fs, const char * path);
void writeFile(fs::FS &fs, const char * path, const char * message);
void appendFile(fs::FS &fs, const char * path, const char * message);
void renameFile(fs::FS &fs, const char * path1, const char * path2);
void deleteFile(fs::FS &fs, const char * path);
void testFileIO(fs::FS &fs, const char * path);

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

//Web Server Timer variables
unsigned long lastTime = 0;  
unsigned long lastTimeTemperature = 0;
unsigned long lastTimeAcc = 0;
unsigned long lastTimeMag = 0;
unsigned long magDelay = 100;
unsigned long gyroDelay = 10;
unsigned long temperatureDelay = 1000;
unsigned long accelerometerDelay = 10;


//
unsigned long currentTime = 0;

float gyroX, gyroY, gyroZ;
float accX, accY, accZ;
float magX, magY, magZ;
float temperature;

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
  delay(100);
  if(!SD.begin(15,spi,4000000)){
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

//String funktiot web servua varten
String getGyroReadings(){
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
  String magString = JSON.stringify (readings);
  return magString;
}
String getTemperature(){
  return String(temperature);
}

String getSDCardSize(){
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  return String(cardSize);
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
    //Tää funktio ei resetoi mitään vaan lähettää sd kortin tiedon web serverille
    events.send(getSDCardSize().c_str(),"SDcard_reading",millis());
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

//   ALKAA ALAPUOLELLA
//   kirjastoihin mahdollisesti laitettavat SD funktiot 
//

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if(!root){
    Serial.println("Failed to open directory");
    return;
  }
  if(!root.isDirectory()){
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while(file){
    if(file.isDirectory()){
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if(levels){
        listDir(fs, file.name(), levels -1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

void createDir(fs::FS &fs, const char * path){
  Serial.printf("Creating Dir: %s\n", path);
  if(fs.mkdir(path)){
    Serial.println("Dir created");
  } else {
    Serial.println("mkdir failed");
  }
}

void removeDir(fs::FS &fs, const char * path){
  Serial.printf("Removing Dir: %s\n", path);
  if(fs.rmdir(path)){
    Serial.println("Dir removed");
  } else {
    Serial.println("rmdir failed");
  }
}

void readFile(fs::FS &fs, const char * path){
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if(!file){
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while(file.available()){
    Serial.write(file.read());
  }
  file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("Failed to open file for writing");
    return;
  }
/*   if(file.print(message)){
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  } */
  file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
  //Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file){
    Serial.println("Failed to open file for appending");
    return;
  }
  if(file.print(message)){
      //Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
  Serial.printf("Renaming file %s to %s\n", path1, path2);
  if (fs.rename(path1, path2)) {
    Serial.println("File renamed");
  } else {
    Serial.println("Rename failed");
  }
}

void deleteFile(fs::FS &fs, const char * path){
  Serial.printf("Deleting file: %s\n", path);
  if(fs.remove(path)){
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }
}

void testFileIO(fs::FS &fs, const char * path){
  File file = fs.open(path);
  static uint8_t buf[512];
  size_t len = 0;
  uint32_t start = millis();
  uint32_t end = start;
  if(file){
    len = file.size();
    size_t flen = len;
    start = millis();
    while(len){
      size_t toRead = len;
      if(toRead > 512){
        toRead = 512;
      }
      file.read(buf, toRead);
      len -= toRead;
    }
    end = millis() - start;
    Serial.printf("%u bytes read for %u ms\n", flen, end);
    file.close();
  } else {
    Serial.println("Failed to open file for reading");
  }


  file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("Failed to open file for writing");
    return;
  }

  size_t i;
  start = millis();
  for(i=0; i<2048; i++){
    file.write(buf, 512);
  }
  end = millis() - start;
  Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
  file.close();
}

int createDataFile(fs::FS &fs, const char * dirname){
  int counter = 0;
  Serial.printf("Numeroidun datatiedoston luonti!\n");

  File root = fs.open(dirname);
  if(!root){
    Serial.println("Failed to open directory");
    return 0;
  }
  if(!root.isDirectory()){
    Serial.println("Not a directory");
    return 0;
  }

  File file = root.openNextFile();
  while(file){
    if(!file.isDirectory()){
      counter++;
      Serial.printf("Filujen määrä: %d\n",counter);
    }
    file = root.openNextFile();
  }
  counter++;
  //tehdään datafilun nimen merkkijono
  char str[32] = "";
  sprintf(str,"/data/%d.csv",counter);

  writeFile(SD, str, "");

  return counter;
}
//   LOPPUU
//   kirjastoihin laitettavat SD funktiot 
//

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
  //SD testikoodia
  listDir(SD,"/",0);
}

void loop()
{
  // read the state of the switch/button:
  btn1State = digitalRead(BTN1);
  btn2State = digitalRead(BTN2);

  //Start or stop the IMU sensor
  if(btn1StateOld == HIGH && btn1State == LOW)
  {
    Serial.println("Button 1 pressed");
    digitalWrite(LED1, !digitalRead(LED1));
    createDataFlag = !createDataFlag;
    IMU = !IMU;
    if(createDataFlag){
      dataFileIndex = createDataFile(SD,"/data");
    }
  }
  btn1StateOld = btn1State;
  
  //Start or stop the wifi server
  if(btn2StateOld == HIGH && btn2State == LOW)
  {
    Serial.println("Button 2 pressed");
    digitalWrite(LED2, !digitalRead(LED2));
    wifi = !wifi;
    if(wifi)
    {
      setupWiFi();
    }
    else
    {
      server.end();
      Serial.println("Server stopped");
      WiFi.disconnect(true);
      Serial.println("Wifi disconnected");
      WiFi.mode(WIFI_OFF);
      Serial.println("Wifi off");
    }
    delay(100);
  }
  btn2StateOld = btn2State;

  if(btn1State == LOW && btn2State == LOW)
  {
    //Don't use if not necessary
    Serial.println("Both buttons pressed");
  }
  

  // Read the sensor data
  if(IMU)
  {
    if(!stopFlag && abs(gyroZ) > 10) //Jos rpm 100
    {
      if(startFlagTime == 0) // If this is the first time gyroZ is above 10
      {
        startFlagTime = millis();
      }
      else if (millis() - startFlagTime >= 2000) //Jos rpm voimassa x millisekuntia 
      {
        stopFlag = true;
        Serial.println("Stop flag activated");
        startFlagTime = 0;
      }
    }
    else 
    {
      startFlagTime = 0; // Reset the timer if gyroZ is not above 10
    }

    if(stopFlag && abs(gyroZ) < 1)
    {
      if(stopFlagTime == 0)
      {
        stopFlagTime = millis();
      }
      else if (millis() - startFlagTime >= 2000)
      {
        IMU = false;
        stopFlag = false;
        digitalWrite(LED1, LOW);
        stopFlagTime = millis();
        Serial.println("Stop flag deactivated");
        stopFlagTime = 0;
      }
    }
    else 
    {
      stopFlagTime = 0;
    }

    if (myICM.dataReady())
    {
      myICM.getAGMT();
      //printRawAGMT( myICM.agmt );
      getSensorData(&myICM);
      //printSensorData();
      currentTime = millis();
      char path[32];
      char buffer[64];
      int ret;
      sprintf(path, "/data/%d.csv",dataFileIndex);

      //tämä toteutus on _hirvittävä_, TODO: siivoa toistuvat rivit
      for (int i=1;i<=10;i++){
        switch(i){
          case 1:
            ret = snprintf(buffer,sizeof buffer,"%f,",gyroX);
            if(ret<0){
              Serial.print("Sensor data write to buffer FAILED\n");
            }
            if(ret>=sizeof buffer){
              Serial.print("Result was truncated - check buffer size");
            }
            appendFile(SD,path,buffer);
            break;
          case 2:
            ret = snprintf(buffer,sizeof buffer,"%f,",gyroY);
            if(ret<0){
              Serial.print("Sensor data write to buffer FAILED\n");
            }
            if(ret>=sizeof buffer){
              Serial.print("Result was truncated - check buffer size");
            }
            appendFile(SD,path,buffer);
            break;
          case 3:
            ret = snprintf(buffer,sizeof buffer,"%f,",gyroZ);
            if(ret<0){
              Serial.print("Sensor data write to buffer FAILED\n");
            }
            if(ret>=sizeof buffer){
              Serial.print("Result was truncated - check buffer size");
            }
            appendFile(SD,path,buffer);
            break;
          case 4:
            ret = snprintf(buffer,sizeof buffer,"%f,",accX);
            if(ret<0){
              Serial.print("Sensor data write to buffer FAILED\n");
            }
            if(ret>=sizeof buffer){
              Serial.print("Result was truncated - check buffer size");
            }
            appendFile(SD,path,buffer);
            break;
          case 5:
            ret = snprintf(buffer,sizeof buffer,"%f,",accY);
            if(ret<0){
              Serial.print("Sensor data write to buffer FAILED\n");
            }
            if(ret>=sizeof buffer){
              Serial.print("Result was truncated - check buffer size");
            }
            appendFile(SD,path,buffer);
            break;
          case 6:
            ret = snprintf(buffer,sizeof buffer,"%f,",accZ);
            if(ret<0){
              Serial.print("Sensor data write to buffer FAILED\n");
            }
            if(ret>=sizeof buffer){
              Serial.print("Result was truncated - check buffer size");
            }
            appendFile(SD,path,buffer);
            break;
          case 7:
            ret = snprintf(buffer,sizeof buffer,"%f,",magX);
            if(ret<0){
              Serial.print("Sensor data write to buffer FAILED\n");
            }
            if(ret>=sizeof buffer){
              Serial.print("Result was truncated - check buffer size");
            }
            appendFile(SD,path,buffer);
            break;
          case 8:
            ret = snprintf(buffer,sizeof buffer,"%f,",magY);
            if(ret<0){
              Serial.print("Sensor data write to buffer FAILED\n");
            }
            if(ret>=sizeof buffer){
              Serial.print("Result was truncated - check buffer size");
            }
            appendFile(SD,path,buffer);
            break;
          case 9:
            ret = snprintf(buffer,sizeof buffer,"%f,",magZ);
            if(ret<0){
              Serial.print("Sensor data write to buffer FAILED\n");
            }
            if(ret>=sizeof buffer){
              Serial.print("Result was truncated - check buffer size");
            }
            appendFile(SD,path,buffer);
            break;
            case 10:
            ret = snprintf(buffer,sizeof buffer,"%d\n",currentTime);
            if(ret<0){
              Serial.print("Sensor data write to buffer FAILED\n");
            }
            if(ret>=sizeof buffer){
              Serial.print("Result was truncated - check buffer size");
            }
            appendFile(SD,path,buffer);
            break;
        }
      }
    }
    else
    {
      SERIAL_PORT.println("Waiting for data");
      delay(500);
    }
  }
  //Send data to the web server

  if(wifi && IMU)
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
    if ((millis() - lastTimeMag) > magDelay) {
    // Send Events to the Web Server with the Sensor Readings
    events.send(getMagReadings().c_str(),"magnetometer_readings",millis());
    lastTimeMag = millis();
    }
    if ((millis() - lastTimeTemperature) > temperatureDelay) {
    // Send Events to the Web Server with the Sensor Readings
    events.send(getTemperature().c_str(),"temperature_reading",millis());
    lastTimeTemperature = millis();
    }
  }
}