#include <DHT.h>
#include <internet.h>
#include <tcp.h>
#include <TEE_UC20.h>
#include <RunningAverage.h>
#include <gnss.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#define DHTPIN 49
#define DHTTYPE DHT22

#define LENG 31
unsigned char buf[LENG];

int PM10Value=0;
int PM2_5Value=0;
int PM1OValue=0;

byte readCO2[] = {0xFE, 0x44, 0x00, 0x08, 0x02, 0x9F, 0x25};
byte response[] = {0,0,0,0,0,0,0};

GNSS gps;
INTERNET net;
TCP tcp;

File myfile;
DHT dht(DHTPIN, DHTTYPE);

RunningAverage CO2(10);
RunningAverage PM1(10);
RunningAverage PM25(10);
RunningAverage PM10(10);
RunningAverage Temp(10);
RunningAverage Humid(10);

#define API "mypos"
#define USER ""
#define PASS ""

int i=1;
double lat = 0.00000;
double lon = 0.00000;
int numsat = 0;
double sp = 0.00;
int h = 0;
int m = 0;
int s = 0;
int day = 0;
int month = 0;
int years = 0;

char filename[] = "data.txt";

void debug(String data)
{
  Serial.println(data);
}
void setup()
{
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600);
  dht.begin();
  i=1;
  gsm.begin(&Serial3,9600);
  gsm.Event_debug = debug;
  Serial.println(F("UC20"));
  gsm.PowerOn();
  while(gsm.WaitReady()){
  Serial.print(F("GetOperator --> "));
  Serial.println(gsm.GetOperator());
  Serial.print(F("SignalQuality --> "));
  Serial.println(gsm.SignalQuality());
  gps.Start();
  Serial.println(F("GPS Start"));
  Serial.println(F("Disconnect net"));
  net.DisConnect();
  Serial.println(F("Set APN and Password"));
  net.Configure(APN,USER,PASS);
  Serial.println(F("Connect net"));
  net.Connect();
  delay(1000);
  Serial.println(F("Show My IP"));
  Serial.println(net.GetIP());
  Serial.println("Initializing SD card...");
  pinMode(53, OUTPUT);
  pinMode(SS, OUTPUT);
  if (!SD.begin(53)) {
    Serial.println("SD did not initialize");
    while (1) ;
  }
  Serial.println("SD initialized.");
  if (! SD.exists(filename))
  {
    myfile = SD.open(filename, FILE_WRITE);
    myfile.println("");
    myfile.flush();
  }
  else
  {
    myfile = SD.open(filename, FILE_WRITE);
    myfile.println("");
    myfile.flush();
  }
}

void loop()
{
  String gnss = gps.GetPosition();
  if (gnss.equals("Not fixed now.\r\n") && gnss.equals("Time Out"))
  {
    lat = 0;
    lon = 0;
    numsat = 0;
    sp =0;
    h = 0;
    m = 0;
    s = 0;
    day = 0;
    month = 0;
    years = 0;
  }
  else
  {
    int last= gnss.lastIndexOf(",")+1;
    int last2= gnss.lastIndexOf(",", last-2)+1;
    int last3= gnss.lastIndexOf(",", last2-2)+1;
    int last4= gnss.lastIndexOf(",", last3-2)+1;
    h = gnss.substring(10,12).toInt()+7;
    if (h>=24)
    {
      h=h-24; 
    }
    m = gnss.substring(12,14).toInt();
    s = gnss.substring(14,16).toInt();
    lat = gnss.substring(19,21).toDouble()+((gnss.substring(21,28).toDouble())/60);
    lon = gnss.substring(30,33).toDouble()+((gnss.substring(33,40).toDouble())/60);
    sp = gnss.substring(last4,last4+3).toDouble();
    numsat = gnss.substring(last,last+2).toInt();
    day = gnss.substring(last2,last2+2).toInt();
    month = gnss.substring(last2+2,last2+4).toInt();
    years = gnss.substring(last2+4,last2+6).toInt();
  }
  if(Serial1.find(0x42)){
    Serial1.readBytes(buf,LENG);
    if(buf[0] == 0x4d){
      if(checkValue(buf,LENG)){
        PM1.0Value=transmitPM1(buf); //count PM1.0 value of the air detector module
        PM2_5Value=transmitPM2_5(buf);//count PM2.5 value of the air detector module
        PM10Value=transmitPM10(buf); //count PM10 value of the air detector module
      }
    }
  }
  PM1.addValue(PM1Value);
  PM2.5.addValue(PM2_5Value);
  PM10.addValue(PM10Value);
  sendRequest(readCO2);
  unsigned long valCO2 = getValue(response);
  CO2.addValue(valCO2);
  double humid = dht.readHumidity();
  double temp = dht.readTemperature();
  Temp.addValue(temp);
  Humid.addValue(humid);
  Serial.print(F("Temp = "));
  Serial.println(Temp.getAverage(),2);
  Serial.print(F("Humid = "));
  Serial.println(Humid.getAverage(),2);
  Serial.print(F("CO2 ppm = "));
  Serial.println(CO2.getAverage(),2);
  Serial.print(F("PM1 ppm = "));
  Serial.println(PM1.getAverage(),2);
  Serial.print(F("PM2.5 ppm = "));
  Serial.println(PM25.getAverage(),2);
  Serial.print(F("PM10 ppm = "));
  Serial.println(PM10.getAverage(),2);
  if (i%10==0) {
    savetosdcard();
    send_tcp();
  }
  i++;
  delay(1500);
}

void sendRequest(byte packet[])
{
  while(!Serial2.available())
  {
    Serial2.write(readCO2,7);
    delay(50);
  }
  int timeout=0;
  while(Serial2.available() < 7 )
  {
    timeout++;
    if(timeout > 10)
    {
      while(Serial2.available())
        Serial2.read();
        break;
    }
    delay(50);
  }
  for (int i=0; i < 7; i++)
  {
    response[i] = Serial2.read();
  }
}

unsigned long getValue(byte packet[])
{
  int high = packet[3];
  int low = packet[4];
  unsigned long val = high256 + low;
  return val 1;
}

char checkValue(unsigned char *thebuf, char leng)
{
  char receiveflag=0;
  int receiveSum=0;

  for(int i=0; i<(leng-2); i++)
  {
   receiveSum=receiveSum+thebuf[i];
  }
  receiveSum=receiveSum + 0x42;

  if(receiveSum == ((thebuf[leng-2]<<8)+thebuf[leng-1]))
  {
    receiveSum = 0;
    receiveflag = 1;
  }
  return receiveflag;
}

int transmitPM01(unsigned char *thebuf)
{
  int PM01Val;
  PM01Val=((thebuf[3]<<8) + thebuf[4]);
  return PM01Val;
}

//transmit PM Value to PC
int transmitPM2_5(unsigned char *thebuf)
{
  int PM2_5Val;
  PM2_5Val= ((thebuf[5]<<8) + thebuf[6]);
  return PM2_5Val;
}

//transmit PM Value to PC
int transmitPM10(unsigned char *thebuf)
{
  int PM10Val;
  PM10Val=((thebuf[7]<<8) + thebuf[8]);
  return PM10Val;
}

void savetosdcard(void)
{
  if (! SD.exists(filename))
  {
    myfile = SD.open(filename, FILE_WRITE);
    myfile.println("Date Time Latitude Longitude numsat Speed TEMP HUMID PM1 PM2.5 PM10 CO2");
    myfile.print(day);
    myfile.print("/");
    myfile.print(month);
    myfile.print("/");
    myfile.print(years);
    myfile.print(" ");
    myfile.print(h);
    myfile.print(":");
    myfile.print(m);
    myfile.print(":");
    myfile.print(s);
    myfile.print(" ");
    myfile.print(lat,5);
    myfile.print(" ");
    myfile.print(lon,5);
    myfile.print(" ");
    myfile.print(numsat);
    myfile.print(" ");
    myfile.print(sp,1);
    myfile.print(" ");
    myfile.print(temp.getAverage(),2);
    myfile.print(" ");
    myfile.print(Humid.getAverage(),2);
    myfile.print(" ");
    myfile.print(PM1.getAverage(),2);
    myfile.print(" ");
    myfile.print(PM25.getAverage(),2);
    myfile.print(" ");
    myfile.print(PM10.getAverage(),2);
    myfile.print(" ");
    myfile.println(CO2.getAverage(),2);
    myfile.flush();
  }
  else
  {
    myfile = SD.open(filename, FILE_WRITE);
    myfile.print(day);
    myfile.print("/");
    myfile.print(month);
    myfile.print("/");
    myfile.print(year);
    myfile.print(" ");
    myfile.print(h);
    myfile.print(":");
    myfile.print(m);
    myfile.print(":");
    myfile.print(s);
    myfile.print(" ");
    myfile.print(lat,5);
    myfile.print(" ");
    myfile.print(lon,5);
    myfile.print(" ");
    myfile.print(alt);
    myfile.print(" ");
    myfile.print(sp1);
    myfile.print(" ");
    myfile.print(Temp.getAverage(),2);
    myfile.print(" ");
    myfile.print(Humid.getAverage(),2);
    myfile.print(" ");
    myfile.print(PM1.getAverage(),2);
    myfile.print(" ");
    myfile.print(PM2.5.getAverage(),2);
    myfile.print(" ");
    myfile.print(PM10.getAverage(),2);
    myfile.print(" ");
    myfile.println(CO2.getAverage(),2);
    myfile.flush();
  }
}

void open_tcp()
{
  Serial.println();
  Serial.println(F("Connect Server"));
  bool ret = tcp.Open("api.thingspeak.com","80");
}

void send_tcp(void)
{
  tcp.Close();
  delay(1000);
  open_tcp();
  delay(1000);
  if(tcp.StartSend())
  {
    Serial.println("HTTP GET");
    tcp.print("GET /");
    tcp.print("update?key=42M8NNBIBSDJG5YJ&field1=");
    tcp.print(String(lat,5));
    tcp.print("&field2=");
    tcp.print(String(lon,5));
    tcp.print("&field3=");
    tcp.print(String(sp,1));
    tcp.print("&field4=");
    tcp.print(String(numsat));
    tcp.print("&field5=");
    tcp.print(String(PM1.getAverage(),2));
    tcp.print("&field6=");
    tcp.print(String(PM25.getAverage(),2));
    tcp.print("&field7=");
    tcp.print(String(PM10.getAverage(),2));
    tcp.print("&field8=");
    tcp.print(String(CO2.getAverage(),2));
    tcp.println(" HTTP/1.1");
    tcp.println("Host: api.thingspeak.com");
    tcp.println("");
    tcp.println("");
    tcp.StopSend();
  }
  tcp.Close();
  delay(1000);
  open_tcp();
  delay(1000);
  if(tcp.StartSend())
  {
    tcp.print("GET /");
    tcp.print("update?api_key=MYYXNTZAKZ1Z1YRO&field1=");
    tcp.print(String(Temp.getAverage(),2));
    tcp.print("&field2=");
    tcp.print(String(Humid.getAverage(),2));
    tcp.println(" HTTP/1.1");
    tcp.println("Host: api.thingspeak.com");
    tcp.println("");
    tcp.println("");
    tcp.StopSend();
    Serial.println("Stop");
  }
}