//#include "EmonLib.h"   //https://github.com/openenergymonitor/EmonLib

//modified 16-04-2024 to send data to HOMEASSISTANT 
//modified 17-04-2024 to add command level to HOMEASSISTANT

#include <ArtnetWifi.h>

//MQTT
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/NanoMixFont.h>
#include <Fonts/NanoMixFontGel.h>

#include <Arduino.h>
#include <Time.h>
//#include "move.h"
#include <RBDdimmer.h> //https://github.com/RobotDynOfficial/RBDDimmer

#include <EEPROM.h>
#define EEPROM_SIZE 100  // 1 byte
#define ADRESS_MODE 1  //mode storage

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

//EnergyMonitor emon;

#define vCalibration 88.13
#define currCalibration 16.13


// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// setting PWM properties
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;

#define BP1   27  
#define BP2   14

//NTP
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600 * 1;
const int   daylightOffset_sec = 3600 * 1;

#define HC_Day_Start 12
#define HC_Day_End 14 //14h30

#define HC_Night_Start 2
#define HC_Night_End 7

bool Is_HC=0;

#define AUTO       0    //KEEP 0
#define AUTO25     1
#define AUTO27     2
#define SOLAR      3
#define FORCED     4
#define STOPPED    5    //KEEP LAST ONE


char DEBUG_TEMP=0;
char DEBUG_WIFI=1;
char DEBUG_MODE=0;
char DEBUG_TIME=0;

char DEBUG_DOSEND=1;

int Mode=AUTO;

//MQTT
// Add your MQTT Broker IP address, example:
//const char* mqtt_server = "192.168.1.12";
const char* mqtt_server = "192.168.1.35";
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

//TRIAC **********************************************
//Parameters
const int zeroCrossPin  = 26;   //GPIO26(11)
const int acdPin        = 13;   //GPIO13(16)

const int PinFan        = 12;

//ADC temperature
const int tempWaterPin = 34;

//DIMMER PARAMETER
int MIN_POWER  = 0;
int MAX_POWER  = 35; //Saturates to around 1kW.
int POWER_STEP  = 1;

//Variables
int power  = 0;

//Objects
dimmerLamp acd(acdPin,zeroCrossPin);
// TRIAC END **********************************************

// As in StepperDemo for Motor 1 on ESP32
#define dirPinStepper 18
#define enablePinStepper 26
#define stepPinStepper 17

//
unsigned int SP_Pos=0;
int SP_Accel=100;
int SP_Speed=7000;
int Prev_Speed=0;
int Prev_Accel=0;
int Prev_Pos=0;

//Wifi settings
const char* ssid = "xtracolors_BOX";
const char* password = "verdouillex";

unsigned long duration1;
unsigned long duration2;

unsigned int valueGrad=0;
unsigned int valueGradArtNet=0;

float tempWater=0;
float tempWaterFiltered=0;
int MQTT_tick=0;

char Disp_On=0;

/*const TickType_t delay_4ms =
      (DELAY_MS_BASE + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS;*/
 

WiFiUDP UdpSend;
ArtnetWifi artnet;

//FastAccelStepperEngine engine = FastAccelStepperEngine();
//FastAccelStepper *stepper = NULL;

TaskHandle_t Handle_aTask;

// connect to wifi – returns true if successful or false if not
bool ConnectWifi(void)
{
  bool state = true;
  int i = 0;

  WiFi.begin(ssid, password);
  if (DEBUG_WIFI) Serial.println("");
  if (DEBUG_WIFI) Serial.println("Connecting to WiFi");
  
  // Wait for connection
  if (DEBUG_WIFI) Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    if (DEBUG_WIFI) Serial.print(".");
    if (i > 20){
      state = false;
      break;
    }
    i++;
  }
  if (state) {
    if (DEBUG_WIFI) Serial.println("");
    if (DEBUG_WIFI) Serial.print("Connected to ");
    if (DEBUG_WIFI) Serial.println(ssid);
    if (DEBUG_WIFI) Serial.print("IP address: ");
    if (DEBUG_WIFI) Serial.println(IPAddress(WiFi.localIP()));
  } else {
    if (DEBUG_WIFI) Serial.println("");
    if (DEBUG_WIFI) Serial.println("Connection failed.");
  }
  
  return state;
}


//Reception ArtNet
void onDmxFrame(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data)
{
  bool tail = false;
  
  
  if (length > 16) 
  {
    length = 16;
    tail = true;
  }
  // send out the buffer
  for (int i = 0; i < length; i++)
  {
    //Serial.print(data[i], HEX);
    //Serial.print(" ");
  }
  if (tail) 
  {
    //Serial.print("...");
  }
  //Serial.println();
   SP_Pos=data[0];
   SP_Accel=1000+(data[2]/255.0)*10000.0;
   SP_Speed=100+(data[1]/255.0)*1000.0;
}



void setup() {

  // set-up serial for debug output
  Serial.begin(115200);
  
  // Allocate The Memory Size Needed
   EEPROM.begin(EEPROM_SIZE);

  //emon.voltage(33, vCalibration, 1.7); // Voltage: input pin, calibration, phase_shift
  //emon.current(25, currCalibration); // Current: input pin, calibration.

   if (Mode>STOPPED) Mode=AUTO;
   //restore stored mode
   Mode = EEPROM.read(ADRESS_MODE);
   
   if (DEBUG_MODE) Serial.print("Save mode was : ");
   if (DEBUG_MODE) 
   {
        if (Mode==AUTO) Serial.println("restore: AUTO");
        if (Mode==SOLAR) Serial.println("restore: SOLAR.");
        if (Mode==FORCED) Serial.println("restore: FORC.");
        if (Mode==STOPPED) Serial.println("restore: STOP");
   
   }
   
   //EEPROM.write(ADRESS_MODE,2);
   //EEPROM.commit();
   delay(1000);
 
  ConnectWifi();

  // this will be called for each packet received
  artnet.setArtDmxCallback(onDmxFrame);
  artnet.begin();

  acd.begin(NORMAL_MODE, ON);

  //to have a +3.3V somewhere for divider bridge with NTC
  pinMode(4,OUTPUT);
  digitalWrite(4,1);

  //to have a 0V
  pinMode(5,OUTPUT);
  digitalWrite(5,0);

   //to have a 5V
  pinMode(PinFan,OUTPUT);
  digitalWrite(PinFan,1);
  ledcSetup(ledChannel, freq, resolution);  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(PinFan, ledChannel);
  ledcWrite(ledChannel, 255);

  //ledcAttachPin(17, 1);
  //ledcSetup(1, 19000, 8); //PWM 19 KHz
  //ledcWrite(1, 0);

   //MQTT To send water temperature
   client.setServer(mqtt_server, 1883);
   
   client.setCallback(callback);
  //xTaskCreate(VerticalAsk,"Axe Vertical",1024, NULL, tskIDLE_PRIORITY + 2, &Handle_aTask);

    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3D))   //3D
    { // Address 0x3D for 128x64
      // Serial.println(F("SSD1306 allocation failed"));
      for(;;); // Don't proceed, loop forever
    }

   display.clearDisplay();
  
  display.setFont(&Roboto_Condensed_Bold_14);
  //display.setFont(&FreeSerif9pt7b);
  
  
   // text display tests
  display.setTextSize(1);
  display.setTextColor(WHITE);

  pinMode(BP1, INPUT);   
  pinMode(BP2, INPUT);   

   // On configure le seveur NTP
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
}



//MQTT
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    if (DEBUG_WIFI) Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client","mqtt-user","mqtt")) {
      if (DEBUG_WIFI) Serial.println("connected");
      // Subscribe
      client.subscribe("homeassistant/output");
      
      SendMqttSlaveConfig();

      //declare slave configuration
      
    } else {
      if (DEBUG_WIFI) Serial.print("failed, rc=");
      if (DEBUG_WIFI) Serial.print(client.state());
      if (DEBUG_WIFI) Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


char appui_1=0;
char appui_2=0;

void Manage_Heat()
{
 
  //Manage Dimmer control
  switch (Mode)
  {
    case AUTO:
    { 
        //minimum temp must be garanteed
        if ((Is_HC) & (tempWater<=23.0))
        {
          if (DEBUG_MODE) Serial.println("AUTO(23) rechauffe ");
          valueGrad=MAX_POWER;
        }
        //no Heure creuse, then regulate to solar
        else
        {
          //saturate
          if (DEBUG_MODE) Serial.println("EXEC. AUTO ");
          if (valueGradArtNet>MAX_POWER) valueGrad=MAX_POWER;
          else valueGrad=valueGradArtNet;
          //nothin to do regulate from SOLAR
        }
        break;
    }

    case AUTO25:
    { 
        //minimum temp must be garanteed
        if ((Is_HC) & (tempWater<=25.0))
        {
          if (DEBUG_MODE) Serial.println("AUTO25 rechauffe ");
          valueGrad=MAX_POWER;
        }
        //no Heure creuse, then regulate to solar
        else
        {
          //saturate
          if (DEBUG_MODE) Serial.println("EXEC. AUTO ");
          if (valueGradArtNet>MAX_POWER) valueGrad=MAX_POWER;
          else valueGrad=valueGradArtNet;
          //nothin to do regulate from SOLAR
        }
        break;
    }

    case AUTO27:
    { 
        //minimum temp must be garanteed
        if ((Is_HC) & (tempWater<=27.0))
        {
          if (DEBUG_MODE) Serial.println("AUTO27 rechauffe ");
          valueGrad=MAX_POWER;
        }
        //no Heure creuse, then regulate to solar
        else
        {
          //saturate
          if (DEBUG_MODE) Serial.println("EXEC. AUTO ");
          if (valueGradArtNet>MAX_POWER) valueGrad=MAX_POWER;
          else valueGrad=valueGradArtNet;
          //nothin to do regulate from SOLAR
        }
        break;
    }

    case SOLAR:
    { 
        //minimum temp must be garanteed
        /*if ((Is_HC) & (tempWater<=23.0))
        {
          Serial.println("auto rechauffe ");
          valueGrad=MAX_POWER;
        }
        //no Heure creuse, then regulate to solar
        else
        {*/
          //saturate
          if (DEBUG_MODE) Serial.println("EXEC. SOLAR");
          if (valueGradArtNet>MAX_POWER) valueGrad=MAX_POWER;
          else valueGrad=valueGradArtNet;
          //nothin to do regulate from SOLAR
        //}
        break;
    }
    case FORCED:
    {
       if (DEBUG_MODE) Serial.println("EXEC. FORCED");
       //full power
       valueGrad=MAX_POWER;
       break;
    }
    case STOPPED:
    {
      if (DEBUG_MODE) Serial.println("EXEC. STOPPED");
        valueGrad=0;
     
        break;
    }
    
  }

  //adjust Dimmer control with values updated above
  if (valueGrad>5)
  {
       acd.begin(NORMAL_MODE, ON);
        ledcWrite(ledChannel, 255);
  }
  else
  {
       acd.begin(NORMAL_MODE, OFF);
        ledcWrite(ledChannel, 0);
  }
  
  acd.setPower(valueGrad);

}

void debounce()
{
  //Manage Button
  if ((!digitalRead(BP1)) & (appui_1==0)) appui_1=1;
  
  if (!digitalRead(BP2)) appui_2=1;
}

int cnt;

void loop() 
{
struct tm timeinfo;

  //GET TIME
  if (!getLocalTime(&timeinfo)) 
  {
    if (DEBUG_TIME) Serial.println("Failed to obtain time");
  }
  if (DEBUG_TIME) Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  //detect HC
  if (
      ((timeinfo.tm_hour>=HC_Day_Start) & (timeinfo.tm_hour<HC_Day_End))
      || ((timeinfo.tm_hour>=HC_Night_Start) & (timeinfo.tm_hour<HC_Night_End))
      ) 
      {
        Is_HC=1;
        if (DEBUG_TIME) Serial.println("HC");
      }
      else 
      {
        Is_HC=0;
        if (DEBUG_TIME) Serial.println("HP");

      }
    
      
  
   //MANAGE BUTTON
   debounce();

   if (appui_1) 
   {
      Mode++;
      if (Mode>STOPPED) Mode=AUTO;
      
      if (Mode==AUTO) Serial.println("now: AUTO");
      if (Mode==AUTO25) Serial.println("now: AUTO-25");
      if (Mode==AUTO27) Serial.println("now: AUTO-27");
      if (Mode==SOLAR) Serial.println("now: SOLAR.");
      if (Mode==FORCED) Serial.println("now: FORC.");
      if (Mode==STOPPED) Serial.println("now: STOP");
      cnt++;
      appui_1=0;
      // Write MODE To EEPROM
      EEPROM.write(ADRESS_MODE, Mode);
      // Call commit() Function To Actually Save The Data You've Written Before
      if (DEBUG_MODE) Serial.println("SAVING ...");
      EEPROM.commit();
      delay(500);
      if (DEBUG_MODE) Serial.println("SAVING DONE");
      //delay(00);
      
   }

   if (appui_2)
   {
    if (Disp_On) Disp_On=0;
    else Disp_On=1;
    
    appui_2=0;
   }

  //MANAGE SCREEN
  display.clearDisplay();
  
/*  display.setCursor(0,53);
  display.println(cnt);*/

  
  if (Disp_On)
  {
      display.setCursor(0,26);
      display.print("Temp: ");
      display.print(tempWater); 
      display.print("°C");
    
      display.setCursor(0,39);
      display.print("Grad: ");
      display.print(valueGrad);
      display.print("%");
    
      display.setCursor(0,13);
      //display.println(&timeinfo,"%A, %B %d %Y %H:%M");
      display.print(&timeinfo,"%H:%M:%S");
      if (Is_HC) display.print(" HC");
      else display.print(" HP");
      if (Mode==AUTO) display.print(" AUTO");
      if (Mode==AUTO25) display.print(" AUTO25");
      if (Mode==AUTO27) display.print(" AUTO27");
      if (Mode==SOLAR) display.print(" SOLA");
      if (Mode==FORCED) display.print(" FORC.");
      if (Mode==STOPPED) display.print(" STOP");
  }
  //update display
  display.display();
  

  //Manage set point from artnet
  artnet.read();
  
  valueGradArtNet = SP_Pos/100.0*100.0;

  // read temp
  tempWaterFiltered = tempWaterFiltered*0.9 + analogRead(tempWaterPin)*0.1;
  if (DEBUG_TEMP) Serial.println(tempWater);
  tempWater = tempWaterFiltered*tempWaterFiltered*0.000014 - 0.0671*tempWaterFiltered + 97.532;
  if (DEBUG_TEMP) Serial.println(tempWater);
  
 
  //Serial.print("temperature : ");
  //Serial.println(tempWater);    

  //PCB val 2100 -> 19°C
  //PROTO
  //1800 = 25°C
  //2900 estime 50°C
  
  delay(100);

  Manage_Heat(); 
  
  if (MQTT_tick++>100)
  {
    //MQTT connect
    if (!client.connected()) 
    {
      reconnect();
    }
    MQTT_tick=0;
    SendMqtt(); //TO UNCOMMENT
  }
}


void testDimmer(){/* function testDimmer */ 
////Sweep light power to test dimmer

  for(power=MIN_POWER;power<=MAX_POWER;power+=POWER_STEP){
    acd.setPower(power); // setPower(0-100%);
      Serial.print("lampValue -> ");
      Serial.print(acd.getPower());
      Serial.println("%");
    delay(100);
  }

  for(power=MAX_POWER;power>=MIN_POWER;power-=POWER_STEP){
    acd.setPower(power); // setPower(0-100%);
      Serial.print("lampValue -> ");
      Serial.print(acd.getPower());
      Serial.println("%");
    delay(100);
  }
}


//MQTT CALLBACK
void callback(char* topic, byte* message, unsigned int length) 
{
  if (DEBUG_WIFI) Serial.print("Message arrived on topic: ");
  if (DEBUG_WIFI) Serial.print(topic);
  if (DEBUG_WIFI) Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    if (DEBUG_WIFI) Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  if (DEBUG_WIFI) Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  if (String(topic) == "homeassistant/output") {
    if (DEBUG_WIFI) Serial.print("Changing output to ");
    if(messageTemp == "on"){
      if (DEBUG_WIFI) Serial.println("on");
      //digitalWrite(ledPin, HIGH);
    }
    else if(messageTemp == "off"){
      if (DEBUG_WIFI) Serial.println("off");
      //digitalWrite(ledPin, LOW);
    }
  }
}

void SendMqttSlaveConfig()
{
    String publishstr;
    String publishstr2;

    String TempECS;
    char publishArray[200];
    
    //{"name":"MQTT HEATER TEMP","state_topic":"stat/mydevice/temperature","unique_id":"heater123","unit_of_measurement":"°C",value_template: "{{ value_json.temperature }}"}

    //temperature
    //publishstr="{\"name\":\"MQTT HEATER TEMP\",\"state_topic\":\"homeassistant/sensor/heater/temperature\",\"unique_id\":\"heater123\",\"unit_of_measurement\":\"°C\",\"value_template\":\"{{ value_json.temperature }}\"} ";
    publishstr="{\"name\":\"MQTT HEATER TEMP\",\"state_topic\":\"homeassistant/sensor/heater/temperature\",\"unique_id\":\"heater123\",\"unit_of_measurement\":\"°C\"} ";


    //command
    publishstr2="{\"name\":\"MQTT HEATER CMD\",\"state_topic\":\"homeassistant/sensor/boiler/value\",\"unique_id\":\"heatercmd123\"} "; //space at the end is important if not curly brace is not send...
    
    if (DEBUG_DOSEND) Serial.println(publishstr);
    publishstr.toCharArray(publishArray, publishstr.length());
    client.publish("homeassistant/sensor/heater/config",publishArray);


    publishstr2.toCharArray(publishArray, publishstr2.length());
    if (DEBUG_DOSEND) Serial.println(publishstr2);
    client.publish("homeassistant/sensor/boiler/config",publishArray);
}


void SendMqtt()
{
    String publishstr;
    String TempECS,Command;
    char publishArray[70];


    // ------------------------------------------- SEND TEMPERATURE
    TempECS=String( tempWater);

    publishstr=TempECS;
    if (DEBUG_WIFI) Serial.println(publishstr);
    
    publishstr.toCharArray(publishArray, publishstr.length());
    
    //the topic where we publish is dependant of the configuration done above
    if (DEBUG_DOSEND) client.publish("homeassistant/sensor/heater/temperature",publishArray,true);  //true means retained message

    // ------------------------------------------ SEND COMMAND

    Command=String(valueGrad);

    publishstr=Command;
    if (DEBUG_WIFI) Serial.println(publishstr);
    
    publishstr.toCharArray(publishArray, publishstr.length()+1);
    
    //the topic where we publish is dependant of the configuration done above
    if (DEBUG_DOSEND) client.publish("homeassistant/sensor/boiler/value",publishArray,true);  //true means retained message

}
