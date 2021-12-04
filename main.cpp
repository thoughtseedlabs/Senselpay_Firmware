#include <Arduino.h>
#include <MAX30105.h>
#include <arduinoFFT.h>
#include <WiFiManager.h>
#include <OSCMessage.h>
#include <heartRate.h>
#include <NTPClient.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#define SIZE 512
using namespace std;

typedef struct{
double powerVLF=0,powerLF=0,powerHF=0,BPM=0,coherence=0,phasic=0,tonic=0;
} Data_t;

/*variable dec*/
IPAddress IP(172,20,10,2);
int OSCPort=8000;
WiFiUDP timeUDP;
NTPClient timeClient(timeUDP,"pool.ntp.org",19800);


// DigiCert High Assurance EV Root CA


/*Func declaration*/
void getDatafromSensor(double rea[],double imag[],double &BPM,uint16_t size);
double getGSRData(bool getPhaic ,bool getValue);
void askCredentials(IPAddress &IPADDR,int &P);
void FFT(double rea[],double imag[],unsigned int size,unsigned short int dir);
void Magnitude(double rea[],double imag[],unsigned int size);
void powerSpectrum(double mag[],double res[],unsigned int size);
void OSC(Data_t data,IPAddress &IPADDR,int &P,NTPClient &client);
double average(double arr[],uint16_t size);
double average(vector<uint16_t> &vec);

 //ADC_MODE(VDD33);
void setup()
{
 Serial.begin(115200);
 WiFi.hostname("SensePlay");
 WiFi.setOutputPower(20.5);
 askCredentials(IP,OSCPort);
/* WiFi.begin("iPhone","mohan1711");
 while(WiFi.status()!=WL_CONNECTED)
 {
   delay(300);
 }*/
 timeClient.begin();
}
void loop()
{
  static uint32_t previousFFTTime=0;
  static uint32_t phasicTime=millis(),tonicTime=millis();
  uint32_t entryTime;
  static Data_t data;
  static double BPM=0;
  static double real[SIZE];
  static double imaginary[SIZE];
  static double power[SIZE];
  double totalpower=0;
  if(WiFi.status()!=WL_CONNECTED)
  {
    ESP.reset();
  }
  getDatafromSensor(real,imaginary,BPM,SIZE);
  if(millis()-phasicTime>=250)
  {
    getGSRData(true,false);
    phasicTime=millis();
  }
  if(millis()-tonicTime>=1000)
  {
    getGSRData(false,false);
    tonicTime=millis();
  }
  if(millis()-previousFFTTime>=2000)
  {
    entryTime=millis();
    FFT(real,imaginary,SIZE,FFT_FORWARD);
    Magnitude(real,imaginary,SIZE);
    powerSpectrum(real,power,SIZE);
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
    Serial.printf("total power %f\n",totalpower);
    for(int i=1; i<3; i++)
    {
      data.powerVLF+=power[i];
    }
    Serial.printf("power vof %f\n",data.powerVLF);
    data.powerVLF/=totalpower;
    for(int i=3; i<42; i++)
    {
      data.powerLF+=power[i];
    }
    Serial.printf("power lf %f\n",data.powerLF);
    data.powerLF/=totalpower;
    for(int i=42; i<SIZE; i++)
    {
      data.powerHF+=power[i];
    }
    Serial.printf("data power hf %f\n",data.powerHF);
    data.powerHF/=totalpower;
    data.phasic=getGSRData(true,true);
    data.tonic=getGSRData(false,true);
    Serial.printf("average value of phasic %f\n",data.phasic);
    Serial.printf("Average value of tonic %f\n",data.tonic);
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

double getGSRData(bool getPhasic=false,bool getValue=false)
{
 static bool init=false;
 static vector<uint16_t> phasic;
 static vector<uint16_t> tonic;
 uint16_t sample;
  if(!init)
 {
   phasic.reserve(64);
   tonic.reserve(256);
   init=true;
 }
 if(!getValue)
 {
 sample=analogRead(A0);
 if(getPhasic)
 {
   phasic.push_back(sample);
   if(phasic.size()>64)
   {
     phasic.erase(phasic.begin());
   }
 }
 else
 {
   tonic.push_back(sample);
   if(tonic.size()>256)
   {
     tonic.erase(tonic.begin());
   }
 }
 return NAN;
 }
 else
 {
   if(getPhasic)
   {
     return average(phasic);
   }
   
       return average(tonic);
  
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
    OSCMessage msg[8]={OSCMessage("/BPM"),OSCMessage("/powerVLF"),OSCMessage("/powerLF"),OSCMessage("/powerHF"),OSCMessage("/coherence"),OSCMessage("/phasic"),OSCMessage("/tonic"),OSCMessage("/time")};
    msg[0].add(float(data.BPM));
    msg[1].add(float(data.powerVLF));
    msg[2].add(float(data.powerLF));
    msg[3].add(float(data.powerHF));
    msg[4].add(float(data.coherence));
    msg[5].add(float(data.phasic));
    msg[6].add(float(data.tonic));
    msg[7].add(client.getFormattedTime().c_str());
    Serial.printf("POWER VLF %lf",data.powerVLF);
    Serial.printf("power lf %f\n",data.powerLF);
    Serial.printf("power HF %f\n",data.powerHF);
    Serial.printf("Coherence =%f\n",data.coherence);
    for(int i=0; i<8; i++)
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

