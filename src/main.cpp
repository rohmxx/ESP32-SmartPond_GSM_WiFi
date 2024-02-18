/**
 * SmartPond IoT Firmware for ESP32 with GSM and/or WiFi Connection
 * Sensor used  : SEN0161-V2, SEN0244, DS18B20
 * Database     : Firebase RTDB
 * Calibration  : pH  -> enterph, calph, exitph
 *                TDS -> enter, cal:xxx, exit
 *
 * @author  Rohman Aditiya
 * @version v2024.02.19
 */


#include <Arduino.h>
#include <EEPROM.h>
#include <string.h>
#include <stdio.h> 
#include <stdlib.h> 
// #include <ArduinoHttpClient.h>

#define wifi_device 1
#define gsm_device 2

uint8_t device_mode = wifi_device;

#define pump_pin 25
unsigned long last_update = 0;
int ph_value = 0, tds_value = 0, temperature_value = 0, pump_state = 0;
String oxygen_value = "LOW";
int log_state[6] = {0};
int log_time[6] = {0};

/*Firebase Configuration*/
#include <Firebase_ESP_Client.h>

const char FIREBASE_HOST[]   = "pfmuda-smartpond-default-rtdb.asia-southeast1.firebasedatabase.app/";
const String FIREBASE_AUTH   = "AIzaSyDmBWX7vt_AgYQUtJMOKk0BOCDi0BeRj0o";
const String FIREBASE_SECRET = "yb8rX7u3kGXIAu5uqdQnAslGAFrFpMHiJ1Xd1I4c";
const String CONTROL_PATH    = "/pump_state";
const String SENSOR_PATH     = "/sensor_data";
const int SSL_PORT           = 443;

#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"


/*WiFI Configuration*/
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
WiFiClient esp32_wifi;
// HttpClient http_client_wifi = HttpClient(sim800l_gsm, FIREBASE_HOST, SSL_PORT);

#define WIFI_SSID "rohmxx-GalaxyA51"
#define WIFI_PASSWORD "hahahihi"

void wifi_init(){
  int counterwifi = 0;
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED){
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print(".");
    counterwifi+=1;
    delay(1000);
    if(counterwifi>5){
      break;
    }
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
}

void get_pump_state(){
  WiFiClientSecure *client = new WiFiClientSecure;
  if(client) {
    client -> setInsecure();
    {
      HTTPClient https;
      if (https.begin(*client, "https://pfmuda-smartpond-default-rtdb.asia-southeast1.firebasedatabase.app/device"+(String)device_mode+"/control/pump_state.json?auth=AIzaSyDmBWX7vt_AgYQUtJMOKk0BOCDi0BeRj0o")) {
        int httpCode = https.GET();
        if (httpCode > 0) {
          if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
            String payload = https.getString();
            // Serial.println(payload);
            sscanf(payload.c_str(), "\"%d\"", &pump_state);
          }
        }
        else {
          Serial.printf("[HTTPS] GET... failed, error: %s\n", https.errorToString(httpCode).c_str());
        }
        https.end();
      } 
      else {
        Serial.printf("[HTTPS] Unable to connect\n");
      }
    }
    delete client;
  } 
  else {
    Serial.println("Unable to create client");
  }

  if(digitalRead(pump_pin)!=pump_state){
    digitalWrite(pump_pin, pump_state);
    // update log
    for (int i = 5; i > 1; i--){
      log_state[i] = log_state[i-1];
      log_time[i] = log_time[i-1];
    }
    log_state[1] = pump_state;
    log_time[1] = last_update;

    // for (int i = 1; i <= 5; i++){
    WiFiClientSecure *client = new WiFiClientSecure;
    if(client) {
      client -> setInsecure();
      {
        HTTPClient https;
        if (https.begin(*client, "https://pfmuda-smartpond-default-rtdb.asia-southeast1.firebasedatabase.app/device"+(String)device_mode+"/logs.json?auth=AIzaSyDmBWX7vt_AgYQUtJMOKk0BOCDi0BeRj0o")) {
          String datacontent = "[null,{\"pump_state\":\""+(String)log_state[1]+"\",\"timestamp\":\""+(String)log_time[1]+"\"},"+
                                      "{\"pump_state\":\""+(String)log_state[2]+"\",\"timestamp\":\""+(String)log_time[2]+"\"},"+
                                      "{\"pump_state\":\""+(String)log_state[3]+"\",\"timestamp\":\""+(String)log_time[3]+"\"},"+
                                      "{\"pump_state\":\""+(String)log_state[4]+"\",\"timestamp\":\""+(String)log_time[4]+"\"},"+
                                      "{\"pump_state\":\""+(String)log_state[5]+"\",\"timestamp\":\""+(String)log_time[5]+"\"}]";
          int httpCode = https.PUT(datacontent);
          if (httpCode > 0) {
            if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
              String payload = https.getString();
              // Serial.println(payload);
            }
          } 
          else {
            Serial.printf("[HTTPS] GET... failed, error: %s\n", https.errorToString(httpCode).c_str());
          }
          https.end();
        } 
        else {
          Serial.printf("[HTTPS] Unable to connect\n");
        }
      }
      delete client;
    } 
    else {
      Serial.println("Unable to create client");
    }
    // }
  }
}

void get_logs(){
  WiFiClientSecure *client = new WiFiClientSecure;
  if(client) {
    client -> setInsecure();
    {
      HTTPClient https;
      // if (https.begin(*client, "https://pfmuda-smartpond-default-rtdb.asia-southeast1.firebasedatabase.app/device"+(String)device_mode+"/logs/"+(String)i+"/pump_state.json?auth=AIzaSyDmBWX7vt_AgYQUtJMOKk0BOCDi0BeRj0o")) {
      if (https.begin(*client, "https://pfmuda-smartpond-default-rtdb.asia-southeast1.firebasedatabase.app/device"+(String)device_mode+"/logs.json?auth=AIzaSyDmBWX7vt_AgYQUtJMOKk0BOCDi0BeRj0o")) {
        int httpCode = https.GET();
        if (httpCode > 0) {
          if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
            String payload = https.getString();
            // Serial.println(payload);
            sscanf(payload.c_str(), "[null,{\"pump_state\":\"%d\",\"timestamp\":\"%d\"},{\"pump_state\":\"%d\",\"timestamp\":\"%d\"},{\"pump_state\":\"%d\",\"timestamp\":\"%d\"},{\"pump_state\":\"%d\",\"timestamp\":\"%d\"},{\"pump_state\":\"%d\",\"timestamp\":\"%d\"}]", 
                                                              &log_state[1],       &log_time[1],           &log_state[2],       &log_time[2],           &log_state[3],       &log_time[3],           &log_state[4],       &log_time[4],           &log_state[5],       &log_time[5]);
          }
        }
        else {
          Serial.printf("[HTTPS] GET... failed, error: %s\n", https.errorToString(httpCode).c_str());
        }
        https.end();
      } 
      else {
        Serial.printf("[HTTPS] Unable to connect\n");
      }
    }
    delete client;
  } 
  else {
    Serial.println("Unable to create client");
  }
}

void post_firebase_wifi(){
  WiFiClientSecure *client = new WiFiClientSecure;
  if(client) {
    client -> setInsecure();
    {
      HTTPClient https;
      if (https.begin(*client, "https://pfmuda-smartpond-default-rtdb.asia-southeast1.firebasedatabase.app/device"+(String)device_mode+"/sensor_data.json?auth=AIzaSyDmBWX7vt_AgYQUtJMOKk0BOCDi0BeRj0o")) {
        String datacontent = "";
        datacontent = "{\"last_update\":\"";
        datacontent += last_update;
        datacontent += "\", \"oxygen\":\"";
        datacontent += oxygen_value;
        datacontent += "\", \"ph\":\"";
        datacontent += ph_value;
        datacontent += "\", \"tds\":\"";
        datacontent += tds_value;
        datacontent += "\", \"temperature\":\"";
        datacontent += temperature_value;
        datacontent += "\"}";
        int httpCode = https.PUT(datacontent);
        if (httpCode > 0) {
          if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
            String payload = https.getString();
            Serial.println(payload);
          }
        } 
        else {
          Serial.printf("[HTTPS] GET... failed, error: %s\n", https.errorToString(httpCode).c_str());
        }
        https.end();
      } 
      else {
        Serial.printf("[HTTPS] Unable to connect\n");
      }
    }
    delete client;
  } 
  else {
    Serial.println("Unable to create client");
  }
}


/*Time*/
#include <NTPClient.h>
#include "time.h"

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

unsigned long getTime() {
  timeClient.update();
  unsigned long now = timeClient.getEpochTime();
  return now;
}


/*Temperature Sensor Configuration*/
#include <OneWire.h>
#include <DallasTemperature.h>
#define temperature_pin 32

const int oneWireBus = temperature_pin;     
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

void temperature_setup(){
  sensors.begin();
}

int read_temperature(){
  sensors.requestTemperatures(); 
  float temperatureC = sensors.getTempCByIndex(0);
  return (int)temperatureC;
}


/*pH Sensor Configuration*/
#include "DFRobot_PH.h"

#define PH_PIN 34
float voltage,phValue,temperature = 25;
DFRobot_PH ph;

void ph_setup()
{
  ph.begin();
}

int read_ph()
{
  static unsigned long timepoint = millis();
  if(millis()-timepoint>1000U){
    timepoint = millis();
    temperature = read_temperature();
    voltage = analogRead(PH_PIN)/4096.0*3300;
    phValue = ph.readPH(voltage,temperature);
  }
  ph.calibration(voltage,temperature);
  return (int)phValue;
}


/*TDS Sensor Configuration*/
#include "GravityTDS.h"

#define TdsSensorPin 35
GravityTDS gravityTds;

float tdsValue = 0;

void tds_setup()
{
  gravityTds.setPin(TdsSensorPin);
  gravityTds.setAref(3.3);
  gravityTds.setAdcRange(4096);
  gravityTds.begin();
}

int read_tds()
{
  temperature = read_temperature();
  gravityTds.setTemperature(temperature);
  gravityTds.update();
  tdsValue = gravityTds.getTdsValue();
  return (int)tdsValue;
}


/*LCD I2C*/
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);

char dataprint[21] = "";

void lcdi2c_init(){
  lcd.init();                  
  lcd.backlight();
  lcd.clear();
}

void lcdi2c_print(int x, int y, String data){
  lcd.setCursor(x, y);
  lcd.print(data);
}



/*Main Program*/
unsigned long prevMillis = 0;

void setup() {
  pinMode(pump_pin, OUTPUT);
  digitalWrite(pump_pin, 0);
  Serial.begin(115200);

  lcdi2c_init();

  // ph_setup();
  tds_setup();
  temperature_setup();

  wifi_init();
  // if(WiFi.status() != WL_CONNECTED){
  //   lcdi2c_print(0, 0, "WiFi not connected");
  //   WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  // }
  // else if(WiFi.status() == WL_CONNECTED){
  //   lcdi2c_print(0, 0, "WiFi connected    ");
  // }

}

void loop(){
  ph_value = -0.0043*(analogRead(34))+15.087;
  tds_value = read_tds();
  temperature_value = read_temperature();

  if(ph_value>4 && tds_value<500){
    oxygen_value = "HIGH";
  }
  else{
    oxygen_value = "LOW ";
  }

  if((millis()-prevMillis)>1000){
    lcdi2c_print(0, 1, "pH:   ");
    lcdi2c_print(10, 1, "O2:     ");
    lcdi2c_print(0, 2, "TDS:     ");
    lcdi2c_print(10, 2, "PUMP:    ");
    lcdi2c_print(0, 3, "Temperature:     ");

    lcdi2c_print(0, 1, "pH: "+(String)ph_value);
    lcdi2c_print(10, 1, "O2: "+oxygen_value);
    lcdi2c_print(0, 2, "TDS: "+(String)tds_value);
    if(digitalRead(25)==1) lcdi2c_print(10, 2, "PUMP: ON ");
    else if(digitalRead(25)==0) lcdi2c_print(10, 2, "PUMP: OFF");
    lcdi2c_print(0, 3, "Temperature: "+(String)temperature_value);

    if(WiFi.status() != WL_CONNECTED){
      lcdi2c_print(0, 0, "WiFi not connected");
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
    if(WiFi.status() == WL_CONNECTED){
      lcdi2c_print(0, 0, "WiFi connected    ");
      last_update = getTime();
      get_logs();
      for (int i = 1; i <= 5; i++){
        Serial.printf("%d - %d %d\n", i, log_state[i], log_time[i]);
      }
      get_logs();
      get_pump_state();
      // post_firebase_wifi();
    }
    
    Serial.printf("timestamp: %d, pH: %3d - %4d, O2: %s, TDS: %4d, Temperature: %3d, pump: %d, log1state: %d, log1time: %d\n", last_update, ph_value, analogRead(34), oxygen_value, tds_value, temperature_value,pump_state, log_state[1], log_time[1]);

    prevMillis = millis();
  }
}