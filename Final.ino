#include "ThingsBoard.h"
#include "CTBot.h"
#include "WiFi.h"
#include <SPI.h> // call library
#include <MCP3XXX.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <genieArduino.h>  // MODIFIED new genieArduino library

#include <stdio.h>
#include <stdint.h>
#include <ctype.h>



Genie genie;
#define RESETLINE 32  // Change this if you are not using an Arduino Adaptor Shield Version 2 (see code below)

#define RXD2 16
#define TXD2 17


#define CS 4 //chip select pin
#define RELAY 2 //solid state relay
#define FILTER true //enable/disable digital filter
#define AVG_NUM 100 //number of readings to average (if filter is on)
MCP3008 adc;

#define WIFI_AP             "Esp32"
#define WIFI_PASSWORD       "prueba123"

// See https://thingsboard.io/docs/getting-started-guides/helloworld/
// to understand how to obtain an access token
#define TOKEN               "Thks2i1teEhpNE7HOgk9"
#define THINGSBOARD_SERVER  "demo.thingsboard.io"

// Baud rate for debug serial
#define SERIAL_DEBUG_BAUD   115200

// Initialize ThingsBoard client
WiFiClient espClient;
// Initialize ThingsBoard instance
ThingsBoard tb(espClient);
//Initialize telegram bot
CTBot myBot;
// the Wifi radio's status 
int status = WL_IDLE_STATUS;
int i = 0;
// Calculator globals

float offset = 0; //variable to save measurement offset
float Amp = 0;
float ISNS20_get_mA(bool is_offset = false);  //function prototype
float Volt = 0;

WiFiMulti wifiMulti;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "es.pool.ntp.org",+7200,6000);


String ver="1.1" ;
int Hora=16;
int BAT = 0;


void setup() {
  // initialize serial for debugging
  
  Serial.begin(SERIAL_DEBUG_BAUD);
  
   Serial.print(ver); //OTA
  for(uint8_t t=4; t > 0; t--){
    Serial.printf("[SETUP] WAIT %d...\n", t);
    Serial.flush();
    delay(1000);
  }
  
  //Voltios
  adc.begin();

  //Amperios
  
  init_Current();
  InitWiFi();
  initPantalla();

 pinMode(0, INPUT);    // sets the digital pin 0 as input
  
  
  myBot.setTelegramToken("2069998038:AAGjP7UnG02xPmjicoTO5879PIXznfC7lYk"); // set the telegram bot token

 delay(1000);



 

}

void loop() {

 if (WiFi.status() != WL_CONNECTED) {
    reconnect();
  }

  else {
    CheckHora();
       //CheckVers();
    }

  if (!tb.connected()) {
    // Connect to the ThingsBoard
    Serial.print("Connecting to: ");
    Serial.print(THINGSBOARD_SERVER);
    Serial.print(" with token ");
    Serial.println(TOKEN);
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN)) {
      Serial.println("Failed to connect");
      return;
    }
  }

  Serial.println("Sending data...");

   //Voltios
  float result = adc.analogRead(i)*5.0/ 1024.0;
  Volt = result;
  Serial.print("V =");
  Serial.print(Volt);

  //Amperios
  Serial.print("I = "); //display measured current
  Amp=ISNS20_get_mA();
  Serial.print(Amp);
  Serial.println("mA");

  delay(1000); //arbitrary delay, to make the values readable


  // Uploads new telemetry to ThingsBoard using MQTT.
  // See https://thingsboard.io/docs/reference/mqtt-api/#telemetry-upload-api
  // for more details
  
  String bateria;
  int auxiliar=0;
  BAT = digitalRead(0);   // read the input pin

  if(BAT == LOW){ 
     bateria = "Bateria Baja";
     auxiliar=100;
    }
  else{ 
    bateria = " Bateria Alta";
    auxiliar= 400;
    }

    escribirAmp(Amp*1000);
    escribirV(Volt*10);
    escribirBateria(auxiliar);
  
  tb.sendTelemetryInt("voltaje", Volt);
  tb.sendTelemetryFloat("intensidad", Amp);
  //datos para el bot
  telemetria_bot(Volt,Amp,bateria);

  delay(1000);

 
 
}

void telemetria_bot (int v,float i,String b) 
{

  
  String string1=""; //PONER VALOR VOLTAJE
  String string2=""; //PONER VALOR CORRIENTE

  string1.concat(v);
  string2.concat(i);
  
  myBot.sendMessage(-674339035, "Voltaje producido: " + string1 + "\n" + "Intensidad producida: " + string2 + "\n" + "Estado de Batería: " + b + "\n" );
  
  delay(1000); // wait ten sec between two messages
}

void InitWiFi()
{
  Serial.println("Connecting to AP ...");
  // attempt to connect to WiFi network

  WiFi.begin(WIFI_AP, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}

void reconnect() {
  // Loop until we're reconnected
  status = WiFi.status();
  if ( status != WL_CONNECTED) {
    WiFi.begin(WIFI_AP, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("Connected to AP");
  }
}

void init_Current(){        //Inicia la recogida del Amperaje
    pinMode(RELAY, OUTPUT);   //initialize pin connected to relay
  digitalWrite(RELAY, LOW); //turn the relay off (load not connected)

  SPI.begin(); // initialization of SPI interface
  SPI.setDataMode(SPI_MODE0); // configuration of SPI communication in mode 0
  SPI.setClockDivider(SPI_CLOCK_DIV16); // configuration of clock at 1MHz
  pinMode(CS, OUTPUT);  //set chip select pin as output

  Serial.println("Disconnecting load to measure the offset");
  Serial.println("Measuring offset...");
  offset = ISNS20_get_mA(true);  //measure and save the offset+++
  Serial.println("Connecting the load");
  delay(500);
  digitalWrite(RELAY, HIGH);   //turn relay on

}

//measure with the sensor (send true as parameter to measure the offset)
float ISNS20_get_mA(bool is_offset) {
  float sum = 0;
  for (int i = 0; i < AVG_NUM; i++) {
    int temporal = 0;
    digitalWrite(CS, LOW);      //begin SPI transfer
    delay(1);
    temporal = SPI.transfer(0x00);   //transfer first byte in
    temporal <<= 8; //shift msb to place
    delay(1);
    temporal |= SPI.transfer(0x00);   //transfer and append second byte in
    delay(1);
    digitalWrite(CS, HIGH);     //end transfer
    delay(1);
    float result = temporal / 4096.0 * (Volt);  //convert raw value to mA: bit result / 12 bits * 5.0V reference

    //if the measurement isn't for offset, correct the error
    if (!is_offset) {
      result = (result - offset) / 0.066; //correct offset: (result-offset)/(0.066V per Amp ratio)
    }

    //if no filtering is needed, return the result
    if (!FILTER) {
      return result;
    }                               

    sum += result;  //add partial results for averaging
  }
  sum /= AVG_NUM; //divide sum to get the average
  return sum;
}


void CheckVers(){
      WiFiClient client;
      HTTPClient http;

       Serial.print("[HTTP] begin...\n");
       http.begin("http://192.168.72.12/test/version.txt");
       Serial.print("[HTTP] GET...\n");
       int httpCode = http.GET();
       if(httpCode > 0) {
        // HTTP header has been send and Server response header has been handled
            Serial.printf("[HTTP] GET... code: %d\n", httpCode);

            // file found at server
            if(httpCode == HTTP_CODE_OK) {
                String payload = http.getString();
               Serial.println(payload);
               if(payload!=ver){
                 UpdateOTA();
                }
            }   
        } else {
            Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
        }
}
void UpdateOTA(){
                        WiFiClient client;
                  t_httpUpdate_return ret = httpUpdate.update(client, "http://192.168.72.12/test/ard.bin");
                  switch (ret) {
                      case HTTP_UPDATE_FAILED:
                         Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
                         break;
    
                      case HTTP_UPDATE_NO_UPDATES:
                        Serial.println("HTTP_UPDATE_NO_UPDATES");
                        break;

                      case HTTP_UPDATE_OK:
                        Serial.println("HTTP_UPDATE_OK");
                        break;
                   } 
}
void CheckHora(){
  timeClient.update();
  if(Hora==timeClient.getHours()){
        CheckVers();
      }
}

void initPantalla()
{
   Serial2.begin(9600,SERIAL_8N1,RXD2,TXD2);
  
  genie.Begin(Serial2);   // Use Serial0 for talking to the Genie Library, and to the 4D Systems display

 
  pinMode(RESETLINE, OUTPUT);  // Set D4 on Arduino to Output (4D Arduino Adaptor V2 - Display Reset)
  digitalWrite(RESETLINE, 0);  // Reset the Display via D4
  delay(100);
  digitalWrite(RESETLINE, 1);  // unReset the Display via D4

  delay (3500); //let the display start up after the reset (This is important)

  genie.DoEvents();
  }

  void escribirBateria(int valor)
{
  // Bateria el valor es de 100 para baja y de 400 para alta
  genie.WriteObject(GENIE_OBJ_IGAUGE, 0, valor); //Amperimetro
  
}


void escribirAmp(int valor)
{
  //3 decimales y el numero tienes que ser entero
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0, valor); //Amperimetro
  
  
}

void escribirV(int valor)
{
  //1 decimales y el numero tienes que ser entero
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 1, valor); //Amperimetro
  
  
}
