/**************************************************MACROS**************************************/
// DEFAULT OUTPUT PIN STATE
#define _DEFAULT_ON HIGH
#define _DEFAULT_OFF LOW
// SERVER INFO
#define _VERSION "ESP IoT Device Starter Kit v1.4.1"
#define _HOSTNAME "ESP-IoT-Device1-"
// MQTT TOPICS
#define _MQTT_BASE "devices/esp01"
#define _MQTT_LOG _MQTT_BASE "/log"
#define _MQTT_UPTIME _MQTT_BASE "/uptime"
#define _DELAY_BUTTON 500
#define _DELAY_BEEPER 1000
#define _DELAY_BUTTON_LONG_PRESS 8000
#define _DELAY_SENSOR_DATA 300 * 1000
#define _DELAY_SYSTEM_STEPS 1500

/**************************** INCLUDES ****************************************/
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <FS.h> 
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <AsyncDelay.h>
#include <DHT.h>
#include <DebounceEvent.h>
#include <ESP8266WiFi.h>
#include <NTPClient.h>
#include <PubSubClient.h>
#include <Ticker.h>
#include <Time.h>
#include <TimeLib.h>
#include <WiFiManager.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include "Flasher.cpp"
#include "Uptime.cpp"

/***************************** function declarations *******************************************/

void mqttCallback(char *topic, byte *payload, unsigned int length);
void log(String message, bool sendMQTT = false);
boolean isValidNumber(String str);
void connectWiFi();
void connectMqtt();
void resetWiFiSettings();
void wifiConfigModeCallback(WiFiManager *myWiFiManager);
void tickerWifiMqttConfigCallback();
void tickerOneSecondCallback();
time_t syncSystemTime();
String getSystemDateTime();
void publishUptime();
bool loadConfigFile();
bool saveConfigFile();
void saveConfigCallback();
void openPort(int portNumber);
void startBeeper();
void getSensorData();
void remote_firmware_update_handler();
void getSensorData(); 
void hr(int beatAvg,float beatsPerMinute,int irValue);
extern void remote_FW_setup();
extern void remote_FW(); 
void AP_MODE_BLINKING();
void FW_UP_MODE_BLINKING();
void WIFI_connection_mode_blinling();

/*********************************GLOBAL VARIABLES***************************************/

int no_finger=0;
unsigned long previous = 0;        
long interv=(2*(60*(1000)));  
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0; 
float beatsPerMinute;
int beatAvg;
const long inter = 1250; 
unsigned long prev = 0; 
String systemIpInfo;
char strTime[32];
int flag=0;
// to save settings, Spiffs, FS
const char *CONFIG_FILE = "/config.json";
bool shouldSaveConfig = false;
char mqttServer[40] = "";
char mqttPort[7] = "";
char mqttUser[40] = "";
char mqttPass[40] = "";
int counter=0;
// GSR SENSORS VARIABLES
const int GSR=A0;
int sensorValue=0;
int gsr_average=0;
char heg_data[10];
float spo2;
float ratio;

/**********************************GLOBAL OBJECTS*****************************/

MAX30105 particleSensor;
WiFiUDP ntp_UDP;
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
NTPClient time_Client(ntp_UDP, "pool.ntp.org"); 
Uptime systemUptime;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", (60 * 60 * 5), (60000 * 60));
Ticker tickerWiFiMqttConfig;
Ticker tickerOneSecond;

// delays
AsyncDelay delayPort1;
AsyncDelay delayPort2;  
AsyncDelay delayBeeper;
AsyncDelay delaySensorData;

/*************************************************************SET UP FUNCTION***************************************************************/

int ticks0, ticks1, ticks2, ticks3, ticks4 = 0;

//Scoring variables
long redValue = 0;
long irValue = 0;
float redAvg = 0;
float irAvg = 0;
float hegratio = 0;
float baseline = 0;

float score = 0;

float p1, p2 = 0;
float v1, v2 = 0;
float accel = 0;

float ratioAvg, adcAvg, posAvg; //velAvg, accelAvg;

float scoreAvg;


//Timing variables
unsigned long startMillis;
unsigned long currentMillis;

const int sampleRate = 1.5; // ADC read rate (ms). ADS1115 has a max of 860sps or 1/860 * 1000 ms or 1.16ms
const int samplesPerRatio = 5; // Minimum number of samples per LED to accumulate before making a measurement. Adjust this with your LED rate so you sample across the whole flash at minimum.
const int BAUDRATE = 115200;

MAX30105 hr_Sensor;

void setup() 
{
   
    Serial.begin(115200);
    
    Serial.println("\n\n===== STARTING =====");
    
    time_Client.begin();

    time_Client.setTimeOffset(19800);
 
    pinMode(LED_BUILTIN,OUTPUT);
    
    digitalWrite(LED_BUILTIN,HIGH);
    
    connectWiFi();
   
    connectMqtt();

    
    timeClient.begin();
    
    timeClient.update();

    setSyncProvider(syncSystemTime);
    
    setSyncInterval(60 * 5);

    setTime(syncSystemTime());

    tickerOneSecond.attach(1, tickerOneSecondCallback);

    delaySensorData.start(_DELAY_SENSOR_DATA, AsyncDelay::MILLIS);

    systemUptime.update();
    
    publishUptime();

    log("System " + systemIpInfo, true);
    
    String msg = String(_VERSION);
    
    msg.concat(" | System ready");
    
    log(msg, true);
 
    Serial.println("Initializing...sensor.......................");

    if(!particleSensor.begin(Wire, I2C_SPEED_FAST)) 
      {
            Serial.println("MAX30105 was not found. Please check wiring/power. ");
            while (1);
      }
       if(!hr_Sensor.begin(Wire, I2C_SPEED_FAST)) 
      {
            Serial.println("MAX30105 was not found. Please check wiring/power. ");
            while (1);
      }
    Serial.println("Place your index finger on the sensor with steady pressure.");

     byte ledBrightness = 0xFF;                                                       //Options: 0=Off to 255=50mA
  byte sampleAverage = 4;                                                         //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2;                                                              //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  int sampleRate = 860;                                                         //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411;                                                        //Options: 69, 118, 215, 411
  int adcRange = 2048;

    particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); 
    
    particleSensor.setPulseAmplitudeRed(0x0A); 
    
    particleSensor.setPulseAmplitudeGreen(0); 

     hr_Sensor.setup(); //Configure sensor with default settings
     hr_Sensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
     hr_Sensor.setPulseAmplitudeGreen(0); 
    
    remote_FW_setup();

}

/******************************************************LOOP FUNCTION*******************************************************/ 

void loop() 
{
    timeClient.update();

    if(WiFi.status() != WL_CONNECTED) 
      {  
        connectWiFi();
      }
      
    if(!mqttClient.connected()) 
      {
        connectMqtt();
      } 
      else 
      {
        mqttClient.loop();
      }

    hegduino();   
  
    getSensorData();
     
    remote_firmware_update_handler();
}

/*............................FUNCTIONS INVOLVED..........................................*/


/**********************************************FUNCTION FOR REMOTE FIRMWARE UPDATE********************************************************/

void remote_firmware_update_handler()
{
  unsigned long current = millis();
  
  if (current- previous >= interv) 
  {
    
    previous = current;

    remote_FW();

  }
 
 }

/************************************FUNCTION TO SEE ARRIVED MESSAGE FROM ANY TOPIC******************************************************************************/
 
void mqttCallback(char *topic, byte *payload, unsigned int length) 
{
    char temp[10];
    
    int i;
    
    static int j;
    
    Serial.print("Message arrived in topic: ");
   
    Serial.println(topic);
    
    Serial.print("Message:");
    
    for (i = 0; i < length; i++) 
    {
        Serial.print((char) payload[i]);
    
        temp[i]=(char)payload[i];
    }
   
   temp[i]='\0';
    
   Serial.println();
   
   Serial.println("-----------------------");
   
   delay(1000);
   
   if(strcmp(temp,"{\"msg\":\"get")==0)
   {
    // client.publish("esp8266", "hello hi         oooo  emqx");
     flag=1;
   }

}

void tickerOneSecondCallback() 
{
    if (systemUptime.Seconds == 0) 
    {
        publishUptime();
    }
}

/*************************************************FUNCTION FOR LED BLINKING DURING AP AND STA MODE************************************************/

void tickerWifiMqttConfigCallback() 
{
 static int cnt=0;
 
 if(cnt%2==0)
 digitalWrite(LED_BUILTIN,LOW);
 else
 digitalWrite(LED_BUILTIN,HIGH);

 cnt++;
}

void wifiConfigModeCallback(WiFiManager *myWiFiManager) 
{   
    log("Entered WiFi Config Mode...");

    log("AP IP: " + WiFi.softAPIP().toString());

    log("AP Setup at " + myWiFiManager->getConfigPortalSSID());   
}
/*************************************************FUNCTION FOR CONNECTING TO WIFI**********************************************************************/
   
void connectWiFi() 
{
     
    WiFi.enableAP(false);

    log("Connecting WiFi...");
    
    String hostName = String(_HOSTNAME) + WiFi.macAddress().substring(9);
    
    hostName.replace(":", "");
    
    WiFi.hostname(hostName);

    tickerWiFiMqttConfig.attach(0.50, tickerWifiMqttConfigCallback);

    WiFiManager wifiManager;

    wifiManager.setAPCallback(wifiConfigModeCallback);

    wifiManager.setSaveConfigCallback(saveConfigCallback);

    wifiManager.setConfigPortalTimeout(120);

    wifiManager.setConnectTimeout(15);

    loadConfigFile();

    WiFiManagerParameter custom_text("<p><b>MQTT Settings</b></p><hr/>");

    WiFiManagerParameter custom_mqtt_server("mqttServer", "MQTT Broker", mqttServer, 40);
    
    WiFiManagerParameter custom_mqtt_port("mqttPort", "MQTT Port", mqttPort, 10);
    
    WiFiManagerParameter custom_mqtt_user("mqttUser", "MQTT User", mqttUser, 40);
    
    WiFiManagerParameter custom_mqtt_pass("mqttPass", "MQTT Password", mqttPass, 40);

    wifiManager.addParameter(&custom_text);
    
    wifiManager.addParameter(&custom_mqtt_server);
    
    wifiManager.addParameter(&custom_mqtt_port);
    
    wifiManager.addParameter(&custom_mqtt_user);
    
    wifiManager.addParameter(&custom_mqtt_pass);

    if(!wifiManager.autoConnect(String(hostName + "-ConfigAP").c_str())) 
    {
        log("Failed to connect and hit timeout");

        ESP.restart();
        
        delay(_DELAY_SYSTEM_STEPS);
    }

    strcpy(mqttServer, custom_mqtt_server.getValue());

    strcpy(mqttPort, custom_mqtt_port.getValue());
    
    strcpy(mqttUser, custom_mqtt_user.getValue());
    
    strcpy(mqttPass, custom_mqtt_pass.getValue());

    if(shouldSaveConfig) 
    {
        saveConfigFile();
        
        shouldSaveConfig = false;
    }

    String ip = WiFi.localIP().toString();
    
    systemIpInfo = "IP: " + ip + " Hostname: " + hostName;
    
    log("WiFi connected at SSID: [" + WiFi.SSID() + "] " + systemIpInfo);

    tickerWiFiMqttConfig.detach();
    
    delay(_DELAY_SYSTEM_STEPS);
}

/*****************************************************FUNCTION FOR RESETTING WIFI SETTINGS********************************************************************************/

void resetWiFiSettings() 
{

    log("Going to reset WiFi settings...");

    delay(_DELAY_SYSTEM_STEPS);

    WiFiManager wifiManager;
    
    wifiManager.resetSettings();

    log("Rebooting device...");
   
    delay(500);

    ESP.restart();
}  

/*********************************************FUNCTION FOR CONNECTING TO MQTT*******************************************/

void connectMqtt() 
{
    log("Connecting to MQTT broker [" + String(mqttServer) + "]...");

    tickerWiFiMqttConfig.attach(0.05, tickerWifiMqttConfigCallback);

    int port = 1883;
    
    if (isValidNumber(String(mqttPort))) 
    {
        port = atoi(mqttPort);
    } 
    else 
    {
        log("ERR - Invalid MQTT port defined in configs, using default port 1883");
    }

    mqttClient.setServer(mqttServer, port);

    mqttClient.setCallback(mqttCallback);

    String clientId = String(_HOSTNAME) + WiFi.macAddress().substring(9);
    clientId.replace(":", "");

    while(!mqttClient.connect(clientId.c_str(), mqttUser, mqttPass)) 
    {

        delay(500);
        
        tickerWiFiMqttConfig.detach();

        delay(_DELAY_SYSTEM_STEPS);

        return;
    }

    while (!mqttClient.connect("mqttx_f3ba57381", mqttUser, mqttPass)) 
    {
        delay(500);

        tickerWiFiMqttConfig.detach();

        delay(_DELAY_SYSTEM_STEPS);

        return;
    }

   // mqttClient.subscribe("esp8266/test");
    
    log("MQTT broker connected", true);

    digitalWrite(LED_BUILTIN,HIGH);
 
    tickerWiFiMqttConfig.detach();
    
    delay(_DELAY_SYSTEM_STEPS);
}

/*********************************************FUNCTIONS FOR PROCESSING SYSTEM TIME***************************************************/

time_t syncSystemTime() 
{
    
    return timeClient.getEpochTime();
}

String getSystemDateTime() 
{
    sprintf(strTime, "%02d-%s-%04d %02d:%02d:%02d", day(), monthShortStr(month()), year(), hour(), minute(), second());
    
    return strTime;
}

void publishUptime() 
{
    String timeString = systemUptime.getUptime();

    mqttClient.publish(_MQTT_UPTIME, timeString.c_str(), true);

    log("System Uptime: " + timeString, true);
}

void log(String message, bool sendMQTT) 
{
    String logMessage = "Log: " + message;

    Serial.println(logMessage);

    if (sendMQTT && mqttClient.connected()) 
    {
        String mqttMessage = getSystemDateTime() + " | " + message;
       // mqttClient.publish(_MQTT_LOG, mqttMessage.c_str());
    }
}

/*..........................................FUNCTION FOR SAVING INPUT WIFI AND MQTT CONFIGURATION INTO FILE SYSTEM**************************************/

bool saveConfigFile() 
{   
    log("Saving config file...");
    
    DynamicJsonBuffer jsonBuffer;
    
    JsonObject &json = jsonBuffer.createObject();

    json["mqttServer"] = mqttServer;
    
    json["mqttPort"] = mqttPort;
    
    json["mqttUser"] = mqttUser;
    
    json["mqttPass"] = mqttPass;

    File file = SPIFFS.open(CONFIG_FILE, "w");
    
    if(!file) 
    {
        log("ERR - Failed to open config file for saving");
        return false;
    }

    json.prettyPrintTo(Serial);

    Serial.println("");

    json.printTo(file);
    
    file.close();

    log("Config file was successfully saved");
    
    return true;
}

void saveConfigCallback() 
{
    log(">>> Should save config!");
    
    shouldSaveConfig = true;
}

/*************************************************FUNCTION FOR LOADING WIFI CONFIGURATION FILE FILE SYSTEM************************************************/

bool loadConfigFile() 
{
    log("Mounting FS...");

    if (SPIFFS.begin()) 
    {
        log("FS mounted.");
        
        if (SPIFFS.exists(CONFIG_FILE)) 
        {
            log("Reading config file... ");
            
            File configFile = SPIFFS.open(CONFIG_FILE, "r");
            
            if (configFile) 
            {
                log("Config file opened and retrieved data: ");
                
                size_t size = configFile.size();

                std::unique_ptr<char[]> buf(new char[size]);

                configFile.readBytes(buf.get(), size);
                
                DynamicJsonBuffer jsonBuffer;
                
                JsonObject &json = jsonBuffer.parseObject(buf.get());
                
                json.prettyPrintTo(Serial);
                
                Serial.println("");

                if (json.success()) 
                {
                    if (json.containsKey("mqttServer")) 
                    {
                        strcpy(mqttServer, json["mqttServer"]);
                    }
                    if (json.containsKey("mqttPort")) 
                    {
                        strcpy(mqttPort, json["mqttPort"]);
                    }
                    if(json.containsKey("mqttUser")) 
                    {
                        strcpy(mqttUser, json["mqttUser"]);
                    }
                    if (json.containsKey("mqttPass")) 
                    {
                        strcpy(mqttPass, json["mqttPass"]);
                    }
                    log("Successfully loaded json config");
                } 
                else 
                {
                    log("ERR - failed to load json config");
                    return false;
                }
            }
        }
    } 
    else 
    {
        log("ERR - failed to mount FS");
        return false;
    }
    return true;
}

/*********************************FUNCTION TO CHECK WHEATHER INPUT STRING CONSIST VALID NUMBER OR NOT****************************************************/

bool isValidNumber(String str) 
{
    if (!(str.charAt(0) == '+' || str.charAt(0) == '-' || isDigit(str.charAt(0))))
        return false;

    for (byte i = 1; i < str.length(); i++) 
    {
        if (!(isDigit(str.charAt(i)) || str.charAt(i) == '.'))
            return false;
    }
    return true;
}

/************************************************FUNCTION FOR READING SENSOR DATA***********************************************************/

void getSensorData() 
{
  
  long irValue = hr_Sensor.getIR();

  if (checkForBeat(irValue) == true)
  {
    long delta = millis() - lastBeat;
   
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; 
      
      rateSpot %= RATE_SIZE; //Wrap variable

      beatAvg = 0;

      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      
      beatAvg /= RATE_SIZE;
      
    }
    
  }

 unsigned long curr= millis();

  if (curr - prev >= inter) 
  {
    prev = curr;
    GSR_READINGS();
    hr(beatAvg,beatsPerMinute,irValue);
  }
  
  if(irValue < 50000)
   no_finger=1; 
  else
   no_finger=0; 

   led_blink();
}

void led_blink()
{
   static int cnt=0;
   
   if(cnt%2==0)
   digitalWrite(LED_BUILTIN,LOW);
   else
   digitalWrite(LED_BUILTIN,HIGH);
   
   cnt++;
}

/*******************************************FUNCTION FOR PUBLISHING SENSOR READINGS TO MQTT BROKER*** *************************************************************/

void hr(int beatAvg,float beatsPerMinute,int irValue)
{
   int i;
     
   char output[35],hr_data[25];
   char str[15];
     
   time_Client.update();

   String formattedTime = time_Client.getFormattedTime();  
   
   for(i=0;formattedTime[i];i++)
   {
    str[i]=formattedTime[i];
   }
   
   str[i]='\0';
     
     float bpm=95.2,avg_bpm=30.5;
     
   mqttClient.publish("esp8266",str);
   
   if(no_finger==1)
      {
         mqttClient.publish("esp8266","No finger");
      }
   if(no_finger==0)
      {     
        sprintf(hr_data, "{\nBPM:%.2f\nAVG BPM:%d\ngsr_average:%d\nBase:%.4f\nred:%.2f\nir:%.2f\nratio:%.4f\nscore:%.2f}",beatsPerMinute,beatAvg,gsr_average,baseline,redAvg,irAvg,hegratio,score); 
     
        mqttClient.publish("HEART RATE",hr_data);
      }
}
/***************************************FUNCTION FOR GSR READING CALCULATIONS********************************************************/

void GSR_READINGS()
{
  
   char gsr_data[10];
   
   long sum=0;
   
   for(int i=0;i<10;i++)                                                                                            //Average the 10 measurements to remove the glitch
      {
      sensorValue=analogRead(GSR);
      sum += sensorValue;
      }
   gsr_average = sum/10;

}

/*********************************************FUNCTIONS RELATED TO HEG***************************************/

void hegduino()
{
   currentMillis = millis();
  
      if(currentMillis - startMillis >= sampleRate) 
      {
              ticks0++;
              
              if(ticks0 > 250)
                {                                                           // Wait for 250 samples of good signal before getting baseline
                                                                                         // IR IN 12, RED IN 13
                         if((ticks1 < 250) && (ticks2 < 250)) 
                             {                                                                  // Accumulate samples for baseline
                                  redValue += particleSensor.getRed();
                                  ticks1++;
               
                                  irValue += particleSensor.getIR();
                                  ticks2++;                            
                             }
                             else 
                             {
                                                                                                             // signalDetermined = true;
                                  redAvg = redValue * 100 / ticks1;
                                  irAvg = irValue / ticks2;

                                  baseline = redAvg / irAvg;                                              // Set baseline ratio
                                  hegratio = baseline;
                                  ticks0 = 0;                                                            // Reset counters
                                  ticks1 = 0;
                                  ticks2 = 0;
                                  ticks3++;
                  
                                  redValue = 0;                                                         // Reset values
                                  irValue = 0;
                             }
              }
              else 
              {
                 ticks0++;
        
                 redValue += particleSensor.getRed();
                 ticks1++;
        
                 irValue += particleSensor.getIR();
                 ticks2++;
             
                 if((ticks2 > samplesPerRatio) && (ticks1 > samplesPerRatio))
                   {                                                                         // Accumulate 50 samples per LED before taking reading
                           redAvg = redValue * 100 / ticks1;                                // Divide value by number of samples accumulated // Scalar multiplier to make changes more apparent
                           irAvg = irValue / ticks2;
                           hegratio = redAvg / irAvg; // Get ratio
                           ratioAvg += hegratio;

                     //      p1 = p2;
                       //    p2 = ratio - baseline;                                              // Position
                         //  posAvg += p2 - p1;
                
                      //     v1 = v2;
                        //   v2 = (p2 - p1) * ticks0 * 0.001; // Velocity in ms
                          // velAvg += v2;

                           //accel = (v2 - v1) * ticks0 * 0.001; // Acceleration in ms^2
                           //accelAvg += accel;

                           score += hegratio-baseline; // Simple scoring method. Better is to compare current and last SMA
                           scoreAvg += score;

                ticks0 = 0; //Reset Counters
                ticks1 = 0;
                ticks2 = 0;

                ticks3++;

                redValue = 0; //Reset values to get next average
                irValue = 0;
              }
            }
          }

          startMillis = currentMillis;
}
/*******************************************LED BLINKING MODES***************************************************************/

void AP_MODE_BLINKING()
{
  digitalWrite(LED_BUILTIN,HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN,LOW);
}

void FW_UP_MODE_BLINKING()
{
  digitalWrite(LED_BUILTIN,HIGH);
  delay(50);
  digitalWrite(LED_BUILTIN,LOW);
}

void WIFI_connection_mode_blinling()
{
  digitalWrite(LED_BUILTIN,HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN,LOW);
}
