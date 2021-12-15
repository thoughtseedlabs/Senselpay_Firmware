#include <Arduino.h>
#include <MAX30105.h>
#include <arduinoFFT.h>
#include <WiFiManager.h>
#include <OSCMessage.h>
#include <heartRate.h>
#include <NTPClient.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#define SIZE 1024
#define RSIZE 128
using namespace std;

typedef struct{
double powerVLF=0,powerLF=0,powerHF=0,BPM=0,coherence=0,phasic=0,tonic=0,phaiscTrend=0,tonicTrend=0;
} Data_t;

/*variable dec*/
IPAddress IP(172,20,10,2);
int OSCPort=8000;
WiFiUDP timeUDP;
NTPClient timeClient(timeUDP,"pool.ntp.org",19800);
const char version[]="1.0.0.3";
const char apiKey[]="191032aa-3103-406d-b7d8-7cbe27516aff";



/*Func declaration*/
void getDatafromSensor(double rea[],double imag[],double &BPM,uint16_t size);
void getGSRData(double rea[],double imag[],uint16_t size);
void askCredentials(IPAddress &IPADDR,int &P);
void FFT(double rea[],double imag[],unsigned int size,unsigned short int dir);
void Magnitude(double rea[],double imag[],unsigned int size);
void powerSpectrum(double mag[],double res[],unsigned int size);
void OSC(Data_t data,IPAddress &IPADDR,int &P,NTPClient &client);
void trend(Data_t &data);
double average(double arr[],uint16_t size);
double average(vector<uint16_t> &vec);
double average(vector<double> &vec);
String getChipID();
void updateFirmware();

 //ADC_MODE(VDD33);
void setup()
{
 Serial.begin(115200);
 WiFi.setOutputPower(20.5);
 askCredentials(IP,OSCPort)
 timeClient.begin();
 updateFirmware();
 Serial.println(version);
}
void loop()
{
  static uint32_t previousFFTTime=0;
  static uint32_t GSRTime=millis();
  uint32_t entryTime;
  static Data_t data;
  static double BPM=0;
  static double real[SIZE];
  static double imaginary[SIZE];
  static double power[SIZE];
  static double resisReal[RSIZE];
  static double resisImag[RSIZE];
  static double resisPower[RSIZE];
  double totalpower=0;
  double resisTotalPower=0;
  if(WiFi.status()!=WL_CONNECTED)
  {
    ESP.reset();
  }
  getDatafromSensor(real,imaginary,BPM,SIZE);
  if(millis()-GSRTime>=250)
  {
     GSRTime=millis();
     getGSRData(resisReal,resisImag,RSIZE);
  }
  
  if(millis()-previousFFTTime>=2000)
  {
    entryTime=millis();
    previousFFTTime=millis();
    FFT(real,imaginary,SIZE,FFT_FORWARD);
    Magnitude(real,imaginary,SIZE);
    powerSpectrum(real,power,SIZE);
    FFT(resisReal,resisImag,RSIZE,FFT_FORWARD);
    Magnitude(resisReal,resisImag,RSIZE);
    powerSpectrum(resisReal,resisPower,RSIZE);
    data.BPM=0;
    data.coherence=0;
    data.phasic=0;
    data.tonic=0;
    data.powerHF=0;
    data.powerVLF=0;
    data.powerLF=0;
    for(int i=1; i<SIZE;i++)
    {
      totalpower+=power[i];
    }
    for(int i=0;i<RSIZE;i++)
    {
      resisTotalPower+=resisPower[i];
    }
    for(int i=1; i<6; i++)
    {
      data.powerVLF+=power[i];
    }
    Serial.printf("power vof %f\n",data.powerVLF);
    data.powerVLF=isnan(data.powerVLF)?0:data.powerVLF/=totalpower;
    for(int i=6; i<84; i++)
    {
      data.powerLF+=power[i];
    }
    data.powerLF=isnan(data.powerLF)? 0:data.powerLF/=totalpower;
    for(int i=84; i<SIZE; i++)
    {
      data.powerHF+=power[i];
    }
    data.powerHF=isnan(data.powerHF)? 0:data.powerHF/totalpower;
    for(int i=0; i<3;i++)
    {
      data.tonic+=resisPower[i];
    }
    data.tonic=isnan(data.tonic)? 0:data.tonic/resisTotalPower;
    for(int i=3;i<RSIZE;i++)
    {
      data.phasic+=resisPower[i];
    }
    data.phasic=isnan(data.phasic)? 0:data.phasic/resisTotalPower;
    data.powerHF/=totalpower;
    data.phasic=
    data.tonic=
    data.coherence=(data.powerVLF&&data.powerLF&&data.powerHF)?0:data.powerLF/(data.powerVLF+data.powerHF);
    data.BPM=BPM;
    Serial.println(BPM);
    OSC(data,IP,OSCPort,timeClient);
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
   for(unsigned int i=0;i<size;i++)
   {
     imag[i]=0;
     rea[i]=0;
   }
   for(unsigned int i=0;i<datavec.size();i++)
   {
     rea[i]=datavec[i];
     //Serial.println(rea[i]);
   }
   
}

void getGSRData(double rea[],double imag[],uint16_t size)
{
 static bool init=false;
 static vector<uint16_t> resisVec;
 uint16_t sample;
 if(!init)
 {
   resisVec.reserve(128);
   init=true;
 }
 sample=analogRead(A0);
 for(int i=0;i<size;i++)
 {
   rea[i]=0;
   imag[i]=0;
 }
 if(sample>20)
 {
   resisVec.push_back(sample);
   if(resisVec.size()>128)
   {
     resisVec.erase(resisVec.begin());
   }
  for(int i=0;i<resisVec.size();i++)
  {
    rea[i]=resisVec[i];
  }
 }
 else
 {
   resisVec.clear();
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
    wifiManager.startConfigPortal("TOM Flow Compass");
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
      res[j]=pow(abs(mag[j]),2)/(2*PI);
  }
}
void OSC(Data_t data,IPAddress &IPADDR,int &P,NTPClient &client)
{
    
    WiFiUDP Udp;
    client.update();
    OSCMessage msg[10]={OSCMessage("/BPM"),OSCMessage("/powerVLF"),OSCMessage("/powerLF"),OSCMessage("/powerHF"),OSCMessage("/coherence"),OSCMessage("/phasic"),OSCMessage("/tonic"),OSCMessage("/phasicTrend"),OSCMessage("/tonicTrend"),OSCMessage("/time")};
    msg[0].add(float(data.BPM));
    msg[1].add(float(data.powerVLF));
    msg[2].add(float(data.powerLF));
    msg[3].add(float(data.powerHF));
    msg[4].add(float(data.coherence));
    msg[5].add(float(data.phasic));
    msg[6].add(float(data.tonic));
    msg[7].add(float(data.phaiscTrend));
    msg[8].add(float(data.tonicTrend));
    msg[9].add(client.getFormattedTime().c_str());
    Serial.printf("POWER VLF %lf",data.powerVLF);
    Serial.printf("power lf %f\n",data.powerLF);
    Serial.printf("power HF %f\n",data.powerHF);
    Serial.printf("Coherence =%f\n",data.coherence);
    for(int i=0; i<10; i++)
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
double average(vector<uint16_t> &vec)
{
  double avg=0;
  for(auto itr=vec.begin();itr!=vec.end();++itr)
  {
    avg+=*itr;
  }
  return avg/vec.size();
}
double average(vector<double> &vec)
{
  double avg=0;
  for(auto itr=vec.begin();itr!=vec.end();++itr)
  {
    avg+=*itr;
  }
  return avg/vec.size();
}
String getChipID()
{
  String chipIDHex;
  chipIDHex+=String(WiFi.macAddress());
  return chipIDHex;
}
void updateFirmware()
{
  String url="http://otadrive.com/deviceapi/update?";
  url+="k="+String(apiKey);
  url+="&v="+String(version);
  url+="&s="+getChipID();
  Serial.println(url);
  ESP8266HTTPUpdate httpUpdate;
  WiFiClient Client;
  int respCode;
  httpUpdate.rebootOnUpdate(true);
  respCode=httpUpdate.update(Client,url,version);
  switch(respCode)
  {
    case HTTP_UPDATE_NO_UPDATES:
    Serial.println("Already on latest version");
    break;

    case HTTP_UPDATE_OK:
    Serial.println("Update successfull");
    break;
  }

}
void trend(Data_t &data)
{
  static bool init=false;
  static vector<double> trendPhasicvec,trendTonicvec;
  vector<double> gradientVec;
  if(!init)
  {
    trendPhasicvec.reserve(32);
    trendTonicvec.reserve(32);
    init=true;
  }
  trendTonicvec.push_back(data.tonic);
  trendPhasicvec.push_back(data.phasic);
  if(trendPhasicvec.size()>32)
  {
    trendPhasicvec.erase(trendPhasicvec.begin());
  }
  if(trendTonicvec.size()>32)
  {
    trendTonicvec.erase(trendTonicvec.begin());
  }
  if(trendTonicvec.size()>1)
  {
    gradientVec.clear();
    for(int i=0;i<trendTonicvec.size()-1;i++)
    {
      gradientVec[i]=trendTonicvec[i+1]-trendTonicvec[i];
    }
    data.phaiscTrend=isnan(average(gradientVec))? 0:average(gradientVec);
  }
  if(trendPhasicvec.size()>1)
  {
    gradientVec.clear();
    for(int i=0;i<trendPhasicvec.size()-1;i++)
    {
      gradientVec[i]=trendPhasicvec[i+1]-trendPhasicvec[i];
    }
    data.tonicTrend=isnan(average(gradientVec))? 0:average(gradientVec);
  }
  
}
