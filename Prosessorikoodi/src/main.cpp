/****************************************************************
 * Example2_Advanced.ino
 * ICM 20948 Arduino Library Demo
 * Shows how to use granular configuration of the ICM 20948
 * Owen Lyke @ SparkFun Electronics
 * Original Creation Date: April 17 2019
 *
 * Please see License.md for the license information.
 *
 * Distributed as-is; no warranty is given.
 * 
 * The ESP32 Web Server Code Example, created by Rui Santos, was utilized as a basis for this 
 * project. Full credit and details can be found at https://randomnerdtutorials.com.
 ***************************************************************/
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Arduino_JSON.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <esp_log.h>

#define USE_SPI       // Uncomment this to use SPI

#define SERIAL_PORT Serial

#define SPI_PORT SPI     // Your desired SPI port.       Used only when "USE_SPI" is defined
#define SPI_FREQ 4000000 // You can override the default SPI frequency
#define CS_PIN 5         // Which pin you connect CS to. Used only when "USE_SPI" is defined


#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

#define LED1 26
#define LED2 27
#define PWR_LED 10

#define BTN1 25
#define BTN2 9

int btn1State = HIGH;
int btn2State = HIGH;

int btn1StateOld = HIGH;
int btn2StateOld = HIGH;
unsigned long btn1PressTime = 0;
unsigned long btn2PressTime = 0;

bool wifi = false;
bool IMU = false;
bool createDataFlag = false;
int dataFileIndex;

//Stopflag variables
unsigned long stopFlagTime = 0;
unsigned long startFlagTime = 0;
bool stopFlag = false;
bool autoStop = true;

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
int createDataFile(fs::FS &fs, const char * dirname);

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
unsigned long gyroDelay = 100;
unsigned long temperatureDelay = 1000;
unsigned long accelerometerDelay = 100;


float gyroX, gyroY, gyroZ;
float accX, accY, accZ;
float magX, magY, magZ;
float temperature;

void initMPU(){
  //Example2_Advanced.ino
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
    myICM.begin(CS_PIN, SPI_PORT, SPI_FREQ); // Here we are using the user-defined SPI_FREQ as the clock speed of the SPI bus
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
  // In this advanced example we'll cover how to do a more fine-grained setup of your sensor
  SERIAL_PORT.println("Device connected!");

  // Here we are doing a SW reset to make sure the device starts in a known state
  myICM.swReset();
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    SERIAL_PORT.print(F("Software Reset returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }
  delay(250);

  // Now wake the sensor up
  myICM.sleep(false);
  myICM.lowPower(false);

  // The next few configuration functions accept a bit-mask of sensors for which the settings should be applied.

  // Set Gyro and Accelerometer to a particular sample mode
  // options: ICM_20948_Sample_Mode_Continuous
  //          ICM_20948_Sample_Mode_Cycled
  myICM.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    SERIAL_PORT.print(F("setSampleMode returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }
  
  // Set full scale ranges for both acc and gyr
  ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors

  myFSS.a = gpm16; // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                  // gpm2
                  // gpm4
                  // gpm8
                  // gpm16

  myFSS.g = dps2000; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                    // dps250
                    // dps500
                    // dps1000
                    // dps2000

  myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    SERIAL_PORT.print(F("setFullScale returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }

  // Set up Digital Low-Pass Filter configuration
  ICM_20948_dlpcfg_t myDLPcfg;    // Similar to FSS, this uses a configuration structure for the desired sensors
  myDLPcfg.a = acc_d473bw_n499bw; // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
                                  // acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
                                  // acc_d111bw4_n136bw
                                  // acc_d50bw4_n68bw8
                                  // acc_d23bw9_n34bw4
                                  // acc_d11bw5_n17bw
                                  // acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz
                                  // acc_d473bw_n499bw

  myDLPcfg.g = gyr_d361bw4_n376bw5; // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)
                                    // gyr_d196bw6_n229bw8
                                    // gyr_d151bw8_n187bw6
                                    // gyr_d119bw5_n154bw3
                                    // gyr_d51bw2_n73bw3
                                    // gyr_d23bw9_n35bw9
                                    // gyr_d11bw6_n17bw8
                                    // gyr_d5bw7_n8bw9
                                    // gyr_d361bw4_n376bw5

  myICM.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    SERIAL_PORT.print(F("setDLPcfg returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }

  // Choose whether or not to use DLPF
  // Here we're also showing another way to access the status values, and that it is OK to supply individual sensor masks to these functions
  ICM_20948_Status_e accDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Acc, true);
  ICM_20948_Status_e gyrDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Gyr, true);
  SERIAL_PORT.print(F("Enable DLPF for Accelerometer returned: "));
  SERIAL_PORT.println(myICM.statusString(accDLPEnableStat));
  SERIAL_PORT.print(F("Enable DLPF for Gyroscope returned: "));
  SERIAL_PORT.println(myICM.statusString(gyrDLPEnableStat));

  // Choose whether or not to start the magnetometer
  myICM.startupMagnetometer();
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    SERIAL_PORT.print(F("startupMagnetometer returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }

  SERIAL_PORT.println();
  SERIAL_PORT.println(F("Configuration complete!"));
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
    digitalWrite(LED1, !digitalRead(LED1));
    digitalWrite(LED2, !digitalRead(LED2));
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

String getFileCountInSDCard(){
  File root = SD.open("/data");
  int fileCount = 0;
  while(true){
    File entry =  root.openNextFile();
    if (! entry){
      // no more files
      break;
    }
    fileCount++;
    entry.close();
  }
  return String(fileCount);
}

String getAutoStop(){
  autoStop = !autoStop;
  if(autoStop){
    //Serial.println("AutoStop ON");
    return "ON"; 
  }
  else
  {
    //Serial.println("AutoStop OFF");
    return "OFF";
  }
}
void startStopMeasurement(){
  createDataFlag = !createDataFlag;
  IMU = !IMU;
  if(createDataFlag){
    dataFileIndex = createDataFile(SD,"/data");
  }
  if(wifi && IMU){
    digitalWrite(LED1, LOW);
  }
  else if(wifi && !IMU)
  {
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, LOW);
  }
  else if(!wifi && !IMU)
  {
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
  }

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
  
  server.on("/startstop", HTTP_GET, [](AsyncWebServerRequest *request){
    startStopMeasurement();
    request->send(200, "text/plain", "OK");
  });

  server.on("/autostop", HTTP_GET, [](AsyncWebServerRequest *request){
    events.send(getAutoStop().c_str(),"autostop_reading",millis());
    request->send(200, "text/plain", "OK");
  });

  server.on("/cardsize", HTTP_GET, [](AsyncWebServerRequest *request){
    events.send(getSDCardSize().c_str(),"SDcard_reading",millis());
    request->send(200, "text/plain", "OK");
  });

  server.on("/filecount", HTTP_GET, [](AsyncWebServerRequest *request){
    events.send(getFileCountInSDCard().c_str(),"SDcard_file_count",millis());
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

  accX = sensor->accX() * 0.00980665; //m/s^2
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
  unsigned long currentTime = millis();
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

  SERIAL_PORT.print("\t\tTime: ");
  SERIAL_PORT.print(currentTime);
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
  int counter = 1;
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
      //Serial.printf("Filujen määrä: %d\n",counter);
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

void saveDataToFile(){
  unsigned long currentTime = millis();
  char path[32];
  char buffer[141];
  int ret;
  sprintf(path, "/data/%d.csv",dataFileIndex);
  
  float sensorData[] = {gyroX, gyroY, gyroZ, accX, accY, accZ, magX, magY, magZ, (float)currentTime};

  for (int i=0; i<10; i++){
    const char* format = (i < 9) ? "%f," : "%f\n";
    ret = snprintf(buffer, sizeof buffer, format, sensorData[i]);
    if(ret<0){
      Serial.print("Sensor data write to buffer FAILED\n");
    }
    if(ret>=sizeof buffer){
      Serial.print("Result was truncated - check buffer size");
    }
    appendFile(SD,path,buffer);
  }
}

void setWiFiOnOff()
{
  digitalWrite(LED1, !digitalRead(LED1));
  wifi = !wifi;
  if(wifi)
  {
    setupWiFi();
    Serial.println("Wifi on");
  }
  else
  {
    server.end();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    Serial.println("Wifi off");
  }
}

void autoStopMeasurement(){
  if(!stopFlag && abs(gyroZ) > 10) //Jos rpm 100
  {
    if(startFlagTime == 0) // If this is the first time gyroZ is above 10
    {
      startFlagTime = millis();
    }
    else if (millis() - startFlagTime >= 2000) //Jos rpm voimassa x millisekuntia 
    {
      stopFlag = true;
      //Serial.println("Stop flag activated");
      startFlagTime = 0;
      if(wifi)
      {
        setWiFiOnOff(); //When the stop flag is activated, turn off the WiFi radio
      }
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
      stopFlagTime = millis();
      //Serial.println("Stop flag deactivated");
      stopFlagTime = 0;
      digitalWrite(LED2, LOW);
      if(!wifi)
      {
        setWiFiOnOff(); //When the stop flag is deactivated, turn on the WiFi radio
      }
    }
  }
  else 
  {
    stopFlagTime = 0;
  }
}

void printDataCsv()
{
  unsigned long currentTime = millis();
  SERIAL_PORT.print(gyroX);
  SERIAL_PORT.print(",");
  SERIAL_PORT.print(gyroY);
  SERIAL_PORT.print(",");
  SERIAL_PORT.print(gyroZ);
  SERIAL_PORT.print(",");
  SERIAL_PORT.print(accX);
  SERIAL_PORT.print(",");
  SERIAL_PORT.print(accY);
  SERIAL_PORT.print(",");
  SERIAL_PORT.print(accZ);
  SERIAL_PORT.print(",");
  SERIAL_PORT.print(magX);
  SERIAL_PORT.print(",");
  SERIAL_PORT.print(magY);
  SERIAL_PORT.print(",");
  SERIAL_PORT.print(magZ);
  SERIAL_PORT.print(",");
  SERIAL_PORT.print(currentTime);
  SERIAL_PORT.println();
}

void printFreeHeap()
{
  SERIAL_PORT.print("Free heap: ");
  SERIAL_PORT.println(ESP.getFreeHeap());
}

void goToSleep()
{
  digitalWrite(LED1, HIGH);
  delay(500);
  digitalWrite(LED1, LOW);
  delay(500);
  digitalWrite(LED1, HIGH);
  delay(500);
  digitalWrite(LED1, LOW);
  delay(500);
  myICM.sleep(true);
  myICM.lowPower(true);
  //esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
  gpio_pullup_en(GPIO_NUM_25);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_25, LOW);
  esp_deep_sleep_start();
}

void setup()
{
  pinMode(LED1,OUTPUT);
  pinMode(LED2,OUTPUT);
  pinMode(PWR_LED,OUTPUT);
  pinMode(BTN1, INPUT_PULLUP);
  pinMode(BTN2, INPUT_PULLUP); 
  SERIAL_PORT.begin(115200);
  
  initMPU();
  initSDCard();
  //SD testikoodia
  listDir(SD,"/data",0);
  digitalWrite(LED1, HIGH);
  delay(500);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, HIGH);
  delay(500);
  digitalWrite(LED2, LOW);
  digitalWrite(PWR_LED, HIGH);
}

void loop()
{
  // read the state of the switch/button:
  btn1State = digitalRead(BTN1);
  btn2State = digitalRead(BTN2);

  //Button functions
  if(btn1StateOld == HIGH && btn1State == LOW)
  {
    Serial.println("Button 1 pressed");
    startStopMeasurement();
  }
  btn1StateOld = btn1State;
  

if(btn2StateOld == HIGH && btn2State == LOW)
{
  btn2PressTime = millis(); // Tallenna aika, kun nappi painetaan alas
}
else if(btn2StateOld == LOW && btn2State == HIGH)
{
  unsigned long pressDuration = millis() - btn2PressTime; // Laske, kuinka kauan nappi oli alhaalla

  if(pressDuration < 3000) //
  {
    Serial.println("Button 2 short press");
    //Start or stop the wifi server
    setWiFiOnOff();
  }
  else
  {
    Serial.println("Button 2 long press");
    goToSleep();
  }

  delay(100);
}

btn2StateOld = btn2State;
  

  // Read the sensor data
  if(IMU)
  {
    digitalWrite(LED2, !digitalRead(LED2));
    if(autoStop)
    {
      autoStopMeasurement();
    }
    if (myICM.dataReady())
    {
      myICM.getAGMT();
      //printRawAGMT( myICM.agmt );
      getSensorData(&myICM);
      //printSensorData();
      //printDataCsv();
      saveDataToFile();
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
  //printFreeHeap();
}