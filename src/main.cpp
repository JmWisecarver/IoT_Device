#include <Arduino.h>
#include <SoftwareSerial.h>
#include "TinyGPS++.h"
#include <SPI.h>
#include <Wire.h>
#include "driver/gpio.h"
#include <Adafruit_NeoPixel.h>
#include <Adafruit_TSL2591.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <NTPClient.h>

/**********TSL2591 REGISTERS**********/
#define TSL2591ADDRESS 0x29 

/**********HDC1080 REGISTERS**********/
#define HDC1080TEMPADDRESS 0x00
#define HDC1080HUMIDITYADDRESS 0x01
#define HDC1080CONFIGADDRESS 0x02
#define HDC1080ADDRESS 0x40

/**********MOTOR PIN NUMBERS**********/
#define MOTORINPUTONE GPIO_NUM_17
#define MOTORINPUTTWO GPIO_NUM_21
#define MOTORINPUTTHREE GPIO_NUM_27
#define MOTORINPUTFOUR GPIO_NUM_33

#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_39) | (1ULL<<GPIO_OUTPUT_IO_34) | (1ULL<<GPIO_OUTPUT_IO_25) | (1ULL<<GPIO_OUTPUT_IO_26))

/**********PIXEL LED PIN SETUP**********/
#define NEOPIXEL_LED 19
#define LED_LENGTH 8

/**********WIFI HOTSPOT LOGIN INFO**********/
const char* ssid = "AndroidAP235F";
const char* password = "umbl7415";

/**********ROTATIONAL VALUES FOR QUEUES**********/
#define ROTATECLOCKWISE 1
#define ROTATECOUNTERCLOCKWISE 2

/**********RTOS SERVER INFO**********/
const int iotd = 1020;
const char* serverKey = "2436e8c114aa64ee";
String serverName = "http://ec2-34-215-160-110.us-west-2.compute.amazonaws.com/";
String regWithServer = "IOTAPI/RegisterWithServer";
String queryServer = "IOTAPI/QueryServerForCommands";
String submitData = "IOTAPI/IOTData";

void readHDC1080_t(void * queue);
void readGroveAir530_t(void * queue);
void runStepperMotor_t(void * queue);
void runLEDStrip_t(void * queue);
void runLightSensor_t(void * queue);
//void runOLED_t(void * queue);
void serverComm_t(void * queue);
String convertTimeInfoToString(tm * timeinfo);

SemaphoreHandle_t i2cMutex;

class dataQueues
{
public:
    QueueHandle_t stepperQueue;
    QueueHandle_t LEDQueue;
    QueueHandle_t temperatureQueue;
    QueueHandle_t humidityQueue;
    QueueHandle_t lightQueue;
    QueueHandle_t gpsQueue;
};

class HDC1080Queues
{
  public:
    QueueHandle_t temperatureQueue;
    QueueHandle_t humidityQueue;
};

void setup() 
{
  Serial.begin(9600);
  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  i2cMutex = xSemaphoreCreateMutex();
  QueueHandle_t stepperQueue;
  QueueHandle_t LEDQueue;
  QueueHandle_t temperatureQueue;
  QueueHandle_t humidityQueue;
  QueueHandle_t lightQueue;
  QueueHandle_t gpsQueue;
  temperatureQueue = xQueueCreate(1, (sizeof(uint32_t*)));
  humidityQueue = xQueueCreate(1, sizeof(uint32_t*));
  LEDQueue = xQueueCreate(10, (sizeof(int)));
  stepperQueue = xQueueCreate(10, (sizeof(int)));
  lightQueue = xQueueCreate(1, (sizeof(uint16_t*)));
  gpsQueue = xQueueCreate(3, (sizeof(double*)));
  dataQueues *collectDataQueues_ptr = new dataQueues;
  HDC1080Queues *HDCQueue_ptr = new HDC1080Queues;
  collectDataQueues_ptr->gpsQueue = gpsQueue;
  collectDataQueues_ptr->stepperQueue = stepperQueue;
  collectDataQueues_ptr->LEDQueue = LEDQueue;
  collectDataQueues_ptr->temperatureQueue = temperatureQueue;
  collectDataQueues_ptr->humidityQueue = humidityQueue;
  collectDataQueues_ptr->lightQueue = lightQueue;
  HDCQueue_ptr->temperatureQueue = temperatureQueue;
  HDCQueue_ptr->humidityQueue = humidityQueue;
  Wire.begin();  
  xTaskCreate(readHDC1080_t, "Read HDC1080", 8192, (void *)HDCQueue_ptr, 6, NULL);
  xTaskCreate(readGroveAir530_t, "Read the air530 grove GPS data", 8096, (void*)gpsQueue, 4, NULL);
  xTaskCreate(runStepperMotor_t, "Run the stepper motor", 8096, (void*)stepperQueue, 3, NULL);
  xTaskCreate(runLEDStrip_t, "Activate the LED strip", 8096, (void*)LEDQueue, 2, NULL);
  xTaskCreate(runLightSensor_t, "Activate the light sensor", 8096, (void*)lightQueue, 5, NULL);
  //xTaskCreate(runOLED_t, "Run the OLED", 8096, NULL, 2, NULL);
  xTaskCreate(serverComm_t, "Activate communication with the server", 8096, (void *)collectDataQueues_ptr, 7, NULL);
}

void loop() 
{
  
}

//Task to handle all communications with the server
void serverComm_t(void * queue)
{

  const char* ntpServer = "pool.ntp.org";
  const long  gmtOffset_sec = -25200;
  const int   daylightOffset_sec = 0;
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  struct tm timeinfo;
  dataQueues *queueClass = (dataQueues *)queue;
  QueueHandle_t stepperQueue = queueClass->stepperQueue;
  QueueHandle_t LEDQueue = queueClass->LEDQueue;
  QueueHandle_t temperatureQueue = queueClass->temperatureQueue;
  QueueHandle_t humidityQueue = queueClass->humidityQueue;
  QueueHandle_t lightQueue = queueClass->lightQueue;
  QueueHandle_t gpsQueue = queueClass->gpsQueue;
  uint32_t * temperature;
  uint32_t * humidity;
  uint16_t * light;
  double * lat;
  double * lng;
  double * alt;
  xQueueReceive(temperatureQueue, &temperature, 1000);
  xQueueReceive(humidityQueue, &humidity , 1000);
  xQueueReceive(lightQueue, &light, 1000);
  xQueueReceive(gpsQueue, &lat, 1000);
  xQueueReceive(gpsQueue, &lng, 1000);
  xQueueReceive(gpsQueue, &alt, 1000);


  WiFiClient client;
  HTTPClient http;
  //Authenticate and log into the server
  StaticJsonDocument<200> doc;
  StaticJsonDocument<200> dataDoc;
  String jsonDoc;
  String jsonDataDoc;
  doc["key"] = serverKey;
  doc["iotid"] = 1020;
  serializeJson(doc, jsonDoc);

  http.begin(client, (serverName + regWithServer));
  http.addHeader("Content-Type", "application/json");
  int httpResponseCode = http.POST(jsonDoc);

  Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
      String payload = http.getString();
      Serial.print("Payload: ");
      Serial.println(payload);
      deserializeJson(doc, payload);
      Serial.print("Deserialized auth code: ");
      String auth_code = doc["auth_code"];
      Serial.println(auth_code);
      doc["auth_code"] = auth_code;
      //Put auth in json and start querying server for commands
      serializeJson(doc, jsonDoc);
      int interval = 0;
      int commandVar = 0;
      int flashVar = 2;
        
      // Free resources
      http.end();
      dataDoc["auth_code"] = auth_code;
      int commandSeconds = 5;
      int sendSeconds = 5;
      unsigned long startCommandCheckTime = millis();
      unsigned long startSendCheckTime = millis();
      unsigned long currentTime = millis();
      unsigned long elapsedTime;
      Adafruit_SSD1306 display = Adafruit_SSD1306(128, 64, &Wire);
      xSemaphoreTake(i2cMutex, 10000);
      display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
      display.clearDisplay();
      display.display();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0,0);
      xSemaphoreGive(i2cMutex);
      bool sendDataNowToggle = false;
      while(1)  {
        currentTime = millis();
        if((currentTime - startCommandCheckTime) > (commandSeconds * 1000)) {
          startCommandCheckTime = millis();
        //if check for commands from server elapsed time is up
        http.begin(client, (serverName + queryServer));
        http.addHeader("Content-Type", "application/json");
        httpResponseCode = http.POST(jsonDoc);
        String payload = http.getString();
        Serial.print("Payload: ");
        Serial.println(payload);
        http.end();
        //Parse out commands from the json
        deserializeJson(doc, payload);
        String test = doc["commands"];
        Serial.println(test);

        // parse a JSON array
        deserializeJson(doc, test);
  
  //For each item in the array send a command to their appropriate destination.
  for (JsonObject item : doc.as<JsonArray>()) {
    String command = item["command"];
    String seconds = item["seconds"];
    Serial.println(command);
    if(command == "RotQCW") {
      commandVar = ROTATECLOCKWISE;
      xQueueSend(stepperQueue, &commandVar, 0);
      Serial.println("Rotate clockwise");
    }
    else if(command == "RotQCCW") {
      commandVar = ROTATECOUNTERCLOCKWISE;
      xQueueSend(stepperQueue, &commandVar, 0);
      Serial.println("Rotate counterClockwise");
    }
    else if(command == "Flash") {
      if(flashVar == 2)
        flashVar = 1;
      else
        flashVar = 2;
      xQueueSend(LEDQueue, &flashVar, 0);
      Serial.println("Flash LEDs");
    }
    else if(command == "SendNow") {
      Serial.println("Send data now");
      sendDataNowToggle = true;
    }
    else if(command == "SetCheckFreq")  {
      Serial.println("Set command check frequency");
      Serial.print("Command check at ");
      Serial.println(seconds);
      commandSeconds = seconds.toInt();
    }
    else if(command == "SetSendFreq") {
      Serial.println("Set data send frequency");
      sendSeconds = seconds.toInt();
    }
  }
  elapsedTime = 0;
  }
        if((currentTime - startSendCheckTime) > (sendSeconds * 1000) || (sendDataNowToggle == true))  {
          if (sendDataNowToggle == true)  {
          sendDataNowToggle = false;
          }
        startSendCheckTime = millis();
        getLocalTime(&timeinfo);
        String currentTime = convertTimeInfoToString(&timeinfo);
        
        if(xSemaphoreTake(i2cMutex, 250))
        {
          display.clearDisplay();
          display.setCursor(0,0);
          display.print(currentTime);
          display.display();
          display.endWrite();
          xSemaphoreGive(i2cMutex);
        }
        
        //Send all data from other tasks by getting a copy from the pointers being used
        //Will likely need to use a mutex here to protect it
        Serial.println("*****DATA*****");
        Serial.print("Light level is: ");
        Serial.println(*light);
        Serial.print("Temperature is: ");
        Serial.println(*temperature);
        Serial.print("Humidity is: ");
        Serial.println(*humidity);
        Serial.print("Latitude is: ");
        Serial.println(*lat);
        Serial.print("Longitude is: ");
        Serial.println(*lng);
        Serial.print("Altitude is: ");
        Serial.println(*alt);
        Serial.println("*****DATA*****");

        

        dataDoc["temperature"] = *temperature;
        dataDoc["humidity"] = *humidity;
        dataDoc["light"] = *light;
        dataDoc["latitude"] = *lat;
        dataDoc["longitude"] = *lng;
        dataDoc["altitude"] = 0;
        dataDoc["time"] = currentTime;

        serializeJson(dataDoc, jsonDataDoc);
        http.begin(client, (serverName + submitData));
        http.addHeader("Content-Type", "application/json");
        httpResponseCode = http.POST(jsonDataDoc);
        payload = http.getString();
        Serial.print("Payload: ");
        Serial.println(payload);
        http.end();
      }
        vTaskDelay(100);
      }

}

//Task to handle the OLED
void runOLED_t(void * queue)
{
  
  Adafruit_SSD1306 display = Adafruit_SSD1306(128, 64, &Wire);
  
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.print("Connecting to SSID\n'adafruit':");
  display.print("connected!");
  display.println("IP: 10.0.1.23");
  display.println("Sending val #0");
  display.setCursor(0,0);
  display.display();
  
  
  while(1)
  {
    //display.display();
    //yield();
    vTaskDelay(1000);
  }
}

//Actibates the neoPixel LEDs.
void runLightSensor_t(void * queue)
{
  QueueHandle_t lightQueue = (QueueHandle_t)queue;
  uint16_t light = 0;
  uint16_t * light_ptr = &light;
  xQueueSend(lightQueue, &light_ptr, 10);
  Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);
  tsl.begin();
  tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
  //tsl.setGain(TSL2591_GAIN_HIGH);   // 428x gain
  
  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_200MS);
  tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_400MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_500MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS);  // longest integration time (dim light)

  /* Display the gain and integration time for reference sake */  
  vTaskDelay(1000);
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Gain:         "));
  tsl2591Gain_t gain = tsl.getGain();
  switch(gain)
  {
    case TSL2591_GAIN_LOW:
      Serial.println(F("1x (Low)"));
      break;
    case TSL2591_GAIN_MED:
      Serial.println(F("25x (Medium)"));
      break;
    case TSL2591_GAIN_HIGH:
      Serial.println(F("428x (High)"));
      break;
    case TSL2591_GAIN_MAX:
      Serial.println(F("9876x (Max)"));
      break;
  }
  Serial.print  (F("Timing:       "));
  Serial.print((tsl.getTiming() + 1) * 100, DEC); 
  Serial.println(F(" ms"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
  while(1)
  {
    if(xSemaphoreTake(i2cMutex, 250)) {
    light = tsl.getLuminosity(TSL2591_VISIBLE);
    xSemaphoreGive(i2cMutex);
    }
    //uint16_t x = tsl.getLuminosity(TSL2591_FULLSPECTRUM);
    //uint16_t x = tsl.getLuminosity(TSL2591_INFRARED);

    //Serial.print(F("[ ")); Serial.print(millis()); Serial.print(F(" ms ] "));
    //Serial.print(F("Luminosity: "));
    //Serial.println(light, DEC);
    vTaskDelay(1000);
  }
}

//Activates the LEDs on the vanduino board
void runLEDStrip_t(void * queue)
{
  QueueHandle_t LEDQueue = (QueueHandle_t)queue;
  int command = 0;
  int commandVar = 0;
  Adafruit_NeoPixel strip =
    Adafruit_NeoPixel(LED_LENGTH, NEOPIXEL_LED, NEO_GRBW + NEO_KHZ800);
    while(1)
    {
      if(xQueueReceive(LEDQueue, &commandVar, 0) == pdTRUE)
        command = commandVar;
      if(command == 1)
      {
        int i = 0;
        i = (rand() % 4);
        strip.setPixelColor(i, 0xFF0000);
        strip.show();
        vTaskDelay(8);
        i = (rand() % 4);
        strip.setPixelColor(i, 0xFF00FF);
        strip.show();
        vTaskDelay(8);
        i = (rand() % 4);
        strip.setPixelColor(i, 0xFF00);
        strip.show();
        vTaskDelay(8);
        i = (rand() % 4);
        strip.setPixelColor(i, 0xFF);
        strip.show();
        vTaskDelay(8);
      }
      else if(command == 2)
      {
        strip.setPixelColor(0, 0x000000);
        strip.setPixelColor(1, 0x000000);
        strip.setPixelColor(2, 0x000000);
        strip.setPixelColor(3, 0x000000);
        strip.show();
        vTaskDelay(250);
      }
      else  {
        vTaskDelay(250);
      }
    }
}

//Run the stepper motor based on commands received
void runStepperMotor_t(void * queue)
{
  gpio_num_t step[4] = {MOTORINPUTONE, MOTORINPUTTWO, MOTORINPUTTHREE, MOTORINPUTFOUR};
  QueueHandle_t stepperQueue = (QueueHandle_t)queue;
  /******************INITIALIZE PINS******************/
  gpio_set_direction(MOTORINPUTONE, GPIO_MODE_OUTPUT);
  gpio_set_direction(MOTORINPUTTWO, GPIO_MODE_OUTPUT);
  gpio_set_direction(MOTORINPUTTHREE, GPIO_MODE_OUTPUT);
  gpio_set_direction(MOTORINPUTFOUR, GPIO_MODE_OUTPUT);

  int queueCommand = 0;
  
  while(1)
  {
    int steps = 0;
    int command = 0;
    //Receive command here as an int from the server comm function
    if(xQueueReceive(stepperQueue, &queueCommand, 100) == pdTRUE)
    {
      command = queueCommand;
    }
    
    switch (command)
    {
      case 1:
        do
        {
          for(int i = 0; i < 4; i++)
          {
            gpio_set_level(step[i], 1);
            vTaskDelay(4);
            gpio_set_level(step[i], 0);
          }
          steps++;
        } while ((steps <= 180));
        break;
      case 2:
        do
        {
          for(int i = 3; i >= 0; i--)
          {
            gpio_set_level(step[i], 1);
            vTaskDelay(4);
            gpio_set_level(step[i], 0);
          }
          steps++;
        } while ((steps <= 180));
        break;
      default:
      break;
    }
    //check once per second
    vTaskDelay(1000);
  }
}

//Read latitude and longitude data from the seed gps plugged into vandaluino GP13/GP12
void readGroveAir530_t(void * queue)
{   
    TinyGPSPlus gps;
    QueueHandle_t gpsReadings = (QueueHandle_t)queue;
    //This will be changed to regular serial once I can't help it anymore or I get better debugging tools
    double lat = 0;
    double lng = 0;
    double alt = 0;
    double * lat_ptr = &lat;
    double * lng_ptr = &lng;
    double * alt_ptr = &alt;
    xQueueSend(gpsReadings, &lat_ptr, 10);
    xQueueSend(gpsReadings, &lng_ptr, 10);
    xQueueSend(gpsReadings, &alt_ptr, 10);
    SoftwareSerial SoftSerial(13, 12);
    SoftSerial.begin(9600);
    
    while(1)
    {
        if (SoftSerial.available())                     // if date is coming from software serial port ==> data is coming from SoftSerial shield
        {
            while(SoftSerial.available())               // reading data into char array
            {
                if(gps.encode(SoftSerial.read()))
                {
                    lat = gps.location.lat();
                    lng = gps.location.lng();

                    Serial.print("Lat is: ");
                    Serial.println(lat);
                    Serial.print("lng is: ");
                    Serial.println(lng);
                    //xQueueSend(gpsReadings, &latlng, 0);
                }
            } 
        }
        vTaskDelay(100);
    }
}

//Read temperature and humidity from the HDC1080
void readHDC1080_t(void * queue) 
{
  HDC1080Queues *queueClass = (HDC1080Queues *)queue;
  QueueHandle_t temperatureQueue = queueClass->temperatureQueue;
  QueueHandle_t humidityQueue = queueClass->humidityQueue;
  uint32_t tempCelcius = 0;
  uint32_t relHumidity = 0;
  uint32_t * tempCelcius_ptr = &tempCelcius;
  uint32_t * relHumidity_ptr = &relHumidity;
  xQueueSend(temperatureQueue, &tempCelcius_ptr, 10);
  xQueueSend(humidityQueue, &relHumidity_ptr, 10);
  uint8_t binaryTemp[2];
  uint8_t binaryHumidity[2];
  uint8_t config[3];
  config[0] = HDC1080CONFIGADDRESS;
  config[1] = 0x06;
  config[2] = 0x0;
  Wire.beginTransmission(HDC1080ADDRESS);   //Configure the HDC1080
  Wire.write(config, 3);
  Wire.endTransmission();

  while(1)
  {
    /**********************READ TEMPERATURE **********************/

    if(xSemaphoreTake(i2cMutex, 250)) {
    Wire.beginTransmission(HDC1080ADDRESS);
    Wire.write(HDC1080TEMPADDRESS);                       //Tell I2C device we are reading from temperature register
    Wire.endTransmission();
    vTaskDelay(5);
    Wire.requestFrom(HDC1080ADDRESS, 2);    //Receive data from the I2C device
    while(Wire.available())
    {
      binaryTemp[0] = Wire.read();  // Read from the starting register
      binaryTemp[1] = Wire.read();  // Read from the ending register
    }

    tempCelcius = binaryTemp[0] << 8 | binaryTemp[1];
    tempCelcius = tempCelcius << 8; //This bit shift is used to avoid using floating point while 
    //This set of equations are the math from the datasheet to get the temp
    tempCelcius = tempCelcius*165;
    tempCelcius = tempCelcius/65536;
    //Shift back
    tempCelcius = tempCelcius >> 8;
    tempCelcius = (tempCelcius-40);

    //Serial.println(tempCelcius);

    /**********************READ HUMIDITY**********************/
    Wire.beginTransmission(HDC1080ADDRESS);
    Wire.write(HDC1080HUMIDITYADDRESS);
    Wire.endTransmission();
    vTaskDelay(5);
    Wire.requestFrom(HDC1080ADDRESS, 2);
    while(Wire.available())
    {
      binaryHumidity[0] = Wire.read(); // read from the starting register
      binaryHumidity[1] = Wire.read();
    }
    relHumidity = binaryHumidity[0] << 8 | binaryHumidity[1];
    //humidity calculation not quite as shown in the data-sheet
    relHumidity = relHumidity*100;
    relHumidity = relHumidity >> 16;
    xSemaphoreGive(i2cMutex);
    }

    //Serial.println(relHumidity);
    vTaskDelay(1000);
  }
}

String convertTimeInfoToString(tm * timeInfo)
{
  String completeFormat;
  
  String year = String(timeInfo->tm_year + 1900);
  String month = String(timeInfo->tm_mon + 1);
  String day = String(timeInfo->tm_mday);
  String hour = String(timeInfo->tm_hour);
  String minute = String(timeInfo->tm_min);
  String second = String(timeInfo->tm_sec);
  if((timeInfo->tm_mon + 1) < 10)
    month = "0" + month;
  if((timeInfo->tm_mday) < 10)
    day = "0" + day;
  if((timeInfo->tm_hour) < 10)
    hour = "0" + hour;
  if((timeInfo->tm_min) < 10)
    minute = "0" + minute;
  if((timeInfo->tm_sec) < 10)
    second = "0" + second;
  completeFormat = year + "-" + month + "-" + day + " " + hour + ":" + minute + ":" + second;
  Serial.print("Complete date is: ");
  Serial.println(completeFormat);
  return completeFormat;
}