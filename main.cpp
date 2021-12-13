#include <Arduino.h>
#include <MAX30105.h>
#include <arduinoFFT.h>
#include <WiFiManager.h>
#include <OSCMessage.h>
#include <heartRate.h>
#include <NTPClient.h>
#include <ElegantOTA.h>

using namespace std;
typedef struct{
double powerVLF=0,powerLF=0,powerHF=0,BPM=0,coherence=0,phasic=0,tonic=0;
} Data_t;

/*variable dec*/
IPAddress IP;
int OSCPort;
WiFiUDP timeUDP;
NTPClient timeClient(timeUDP,"pool.ntp.org",19800);


 
/*Func declaration*/
void getDatafromSensor(double rea[],double imag[],double &BPM,uint16_t size);
void getGSRData(double rea[],double imag[],uint16_t size);
void askCredentials(IPAddress &IPADDR,int &P);
void FFT(double rea[],double imag[],unsigned int size,unsigned short int dir);
void Magnitude(double rea[],double imag[],unsigned int size);
void powerSpectrum(double mag[],double res[],unsigned int size);
void OSC(Data_t data,IPAddress &IPADDR,int &P,NTPClient &client);
double average(double arr[],uint16_t size);
void FirmwareUpdate();

void setup()
{
 Serial.begin(115200);
 askCredentials(IP,OSCPort);
 timeClient.begin();
 
}
void loop()
{
  static uint32_t previousFFTTime=0;
  static uint32_t GSRTime=0;
  uint32_t entryTime;
  static Data_t data;
  static double BPM=0;
  static double real[512];
  static double imaginary[512];
  static double power[512];
  static double resisReal[128];
  static double resisImaginary[128];
  static double resisPower[128];
  if(WiFi.status()!=WL_CONNECTED)
  {
    ESP.reset();
  }
  getDatafromSensor(real,imaginary,BPM,512);
  if(millis()-GSRTime>=1000)
  {
    getGSRData(resisReal,resisImaginary,256);
  }
  if(millis()-previousFFTTime>=2000)
  {
    entryTime=millis();
    FFT(real,imaginary,512,FFT_FORWARD);
    Magnitude(real,imaginary,512);
    powerSpectrum(real,power,512);
    FFT(resisReal,resisImaginary,128,FFT_FORWARD);
    Magnitude(resisReal,resisImaginary,128);
    powerSpectrum(resisReal,resisPower,128);
    data.BPM=0;
    data.coherence=0;
    data.phasic=0;
    data.tonic=0;
    data.powerHF=0;
    data.powerVLF=0;
    data.powerLF=0;
    for(int i=0; i<3; i++)
    {
      data.powerVLF+=power[i];
    }
    for(int i=3; i<42; i++)
    {
      data.powerLF+=power[i];
    }
    for(int i=43; i<256; i++)
    {
      data.powerHF+=power[i];
    }
    for(int i=0; i<4; i++)
    {
      data.tonic+=resisPower[i];
    }
    for(int i=4;i<256; i++)
    {
      data.phasic+=resisPower[i];
    }
    data.coherence=data.powerLF/(data.powerVLF+data.powerHF);
    data.BPM=BPM;
    Serial.println(BPM);
    OSC(data,IP,OSCPort,timeClient);
    previousFFTTime=millis();
    Serial.printf("Done in %ld \n",millis()-entryTime);
  }
  


 
}


void getDatafromSensor(double rea[],double imag[],double &BPM,uint16_t size)
{
  static MAX30105 Sensor;
  static vector<uint32_t> datavec;
  static bool init=false;
  static uint32_t previousBeat=millis();
  static double beatArray[4]={0,0,0,0};
  static int i=0;
  uint32_t sample;
  if(!init)
  {
      datavec.reserve(size+2);
     if(Sensor.begin())
     {
       Sensor.setup(32,1,3,50,411,4096);
       init=true;
     }
     else
     {
       Serial.println("Failed sensor init");
       while(1)
       {
         ESP.wdtFeed();
       }
     }
  }
  Sensor.check();
  while(Sensor.available())
  {
    sample=Sensor.getFIFOIR();
    Sensor.nextSample();
    if(sample>=50000)
    {
      if(checkForBeat(sample))
      {
        BPM=60000/(millis()-previousBeat);
        previousBeat=millis();
        beatArray[i]=BPM;
        i+=1;
        i%=4;
        BPM=average(beatArray,4);
        if(BPM<45)
        {
          BPM=45;
        }
        else if(BPM>120)
        {
          BPM=120;
        }
        else
        {
          BPM=BPM;
        }
      }
      datavec.push_back(sample);
      if(datavec.size()>size)
      {
        datavec.erase(datavec.begin());
      }
    }
    else
    {
      datavec.clear();
      checkForBeat(0);
      BPM=NAN;
    }
  }
   for(int i=0;i<size;i++)
   {
     imag[i]=0;
     rea[i]=0;
   }
   for(int i=0;i<datavec.size();i++)
   {
     rea[i]=datavec[i];
   }
   
}

void getGSRData(double rea[],double imag[],uint16_t size)
{
 static bool init=false;
 static vector<uint16_t> resistVec;
 uint16_t sample;
 if(!init)
 {
   pinMode(PIN_A0,INPUT);
   resistVec.reserve(size+2);
   init=true;
 }
 for(uint16_t i=0; i<size; i++)
 {
   rea[i]=0;
   imag[i]=0;
 }
 sample=analogRead(A0);
 if(sample>50)
 {
   resistVec.push_back(sample);
   for(uint16_t i=0; i<resistVec.size();i++)
   {
     rea[i]=(1024+2*resistVec[i])*10000/(512-resistVec[i]);
   }
 }
 else
 {
   resistVec.clear();
   for(uint16_t i=0; i<size; i++)
   {
     rea[i]=INFINITY;
   }
 }
 
 
} 

void askCredentials(IPAddress &IPADDR,int &P) 
{
    
    begin:
    WiFi.enableAP(false);
    Serial.println("Connecting WiFi...");
    WiFiManager wifiManager;
    wifiManager.resetSettings();
    WiFiManagerParameter OSCPORT("PORT","PORT NUMBER","8080",5);
    WiFiManagerParameter OSCIP("OSC IP","OSC Port","198.162.4.1",20);
    wifiManager.addParameter(&OSCIP);
    wifiManager.addParameter(&OSCPORT);
    wifiManager.startConfigPortal("ESP8266");
    wifiManager.setConfigPortalTimeout(300);
    String ip = WiFi.localIP().toString();
    String systemIpInfo = "IP: " + ip + " Hostname: ";
    Serial.println("WiFi connected at SSID: [" + WiFi.SSID() + "] " + systemIpInfo);
    P=String((OSCPORT.getValue())).toInt();
    if(!IPADDR.fromString(OSCIP.getValue()))
    {
      goto begin;
    }
    if(!P)
    {
      goto begin;
    }
    
}
void FFT(double rea[],double imag[],unsigned int size,unsigned short int dir)
{

   arduinoFFT FFTComputer;
   FFTComputer.Compute(rea,imag,size,dir); 

}
 void Magnitude(double rea[],double imag[],unsigned int size)
{
    arduinoFFT FFTComputer;
    FFTComputer.ComplexToMagnitude(rea,imag,size);
}
void powerSpectrum(double mag[],double res[],unsigned int size)
{
  for(unsigned int j=0; j<size; j++)
  {
      res[j]=pow(mag[j],2)/(2*PI);
  }

}
void OSC(Data_t data,IPAddress &IPADDR,int &P,NTPClient &client)
{
    
    WiFiUDP Udp;
    client.update();
    OSCMessage msg[7]={OSCMessage("/BPM"),OSCMessage("/powerVLF"),OSCMessage("/powerLF"),OSCMessage("/powerHF"),OSCMessage("/phasic"),OSCMessage("/tonic"),OSCMessage("/time")};
    msg[0].add(data.BPM);
    msg[1].add(data.powerVLF);
    msg[2].add(data.powerLF);
    msg[3].add(data.powerHF);
    msg[4].add(data.phasic);
    msg[5].add(data.tonic);
    msg[5].add(client.getFormattedTime().c_str());
    for(int i=0; i<7; i++)
    {
    Udp.beginPacket(IPADDR,P);
    msg[i].send(Udp);
    Udp.endPacket();
    msg[i].empty();
    }
    Serial.println("Done OSC");
    

}
double average(double array[],uint16_t size)
{
  double average=0;
  for(uint16_t i=0; i<size; i++)
  {
    average+=array[i];
  }
 return average/size;
}
