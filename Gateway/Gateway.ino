#include <WiFi.h>
#include <Ethernet.h>
#include <WiFiClient.h>
#include <LoRa.h>
#include <SPI.h>
#include "String.h"

// SPI Lora
#define LORA_RST      14
#define LORA_SS       18
#define LORA_MOSI     27
#define LORA_MISO     19
#define LORA_SCK      5
#define LORA_DIO0     26

int freq = 2400;
int channel = 0;
int id_temp = 1;
char ctrl_device[10];
char ctrl_device_temp[10];

unsigned int id[10];
String  stringData;
String  tstr;
char    node_data[20];
char    node_id[10];
char    temp[10];
char    humi[10];
char    soil[10];
char    relay_1[10];
char    relay_2[10];

#ifndef STASSID
#define STASSID   "NOkia_X6"
//#define STASSID   "DRK"
#define STAPSK    "35354935qq"
//#define STAPSK   "home123456"
#endif

const char* ssid      = STASSID;
const char* password  = STAPSK;

const   char*         host = "tvkhanh.000webhostapp.com";
const   uint16_t      port = 80;

WiFiClient client;

void setup()
{
    Serial.begin(115200);

    // Ket noi voi WiFi
    Serial.print("Dang ket noi voi WiFi:");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.println(".");
    }
    Serial.println("Da ket noi voi WiFi");
    Serial.print("Dia chi IP: ");
    Serial.println(WiFi.localIP());
    
    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
    if(!LoRa.begin(434E6))
    {
      Serial.println("Connect to LoRa Failed!");
      while(1);
    }
    Serial.println("Configuration for LoRa module");
    LoRa.setSpreadingFactor(10);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
    LoRa.setPreambleLength(10);
    LoRa.enableCrc();
    Serial.println("Initialize Done!");
}

void loop()
{
 //Send id to node
 Serial.print("Send to node: ");
 Serial.println(id_temp);
 LoRa.beginPacket();
 LoRa.print(id_temp);
 LoRa.endPacket();
 delay(1000);
  //Nhan du lieu tu node
  int packetSize = LoRa.parsePacket();
  if(packetSize)
  {
    while (LoRa.available())
    {
      stringData = LoRa.readString();
      Serial.print("String: ");
      Serial.println(stringData);
    }
  
    //Cat chuoi stringData
    stringData.toCharArray(node_data, 20);
   
    strncpy(node_id, node_data +0, 1);
    node_id[1] = '\0';
    Serial.print("Dia chi node: ");
    Serial.println(node_id[0]);
    id[0] = node_id[0] - 48;
    
    strncpy(temp, node_data + 1, 4);
    temp[4] = '\0';

    strncpy(humi, node_data + 5, 4);
    humi[4] = '\0';

    strncpy(soil, node_data +9, 5);
    soil[5] = '\0';

    strncpy(relay_1, node_data + 14, 3);
    relay_1[3] = '\0';

    strncpy(relay_2, node_data + 17, 3);
    relay_2[3] = '\0';
       
    switch (id[0])
    {
      case 1:
      send2node_1(temp, humi, soil, relay_1, relay_2);
      break;
    
      case 2:
      send2node_2(temp, humi, soil, relay_1, relay_2);
      break;
    }
    id_temp += 1;
    if(id_temp == 3)
    {
      id_temp = 1;
    }
  }
}


void send2node_1(String temp, String humi, String soil, String relay_1, String relay_2)
{
    if (client.connect(host, port))
    {
        String httpData = "/node_1/write_data.php?temp=" + String(temp) + "&humi=" + String(humi) +
        "&soil=" + String(soil) + "&relay_1=" + String(relay_1) + "&relay_2=" + String(relay_2) ;
        
        client.print(String("GET ") + httpData + " HTTP/1.1\r\n" + "Host: " + host + "\r\n" + "Connection: Close\r\n\r\n");
    }
    else
    {
        Serial.println("Ket noi voi may chu that bai");
        return;
    }
}

void send2node_2(String temp, String humi, String soil, String relay_1, String relay_2)
{
    if (client.connect(host, port))
    {
        String httpData = "/node_2/write_data.php?temp=" + String(temp) + "&humi=" + String(humi) +
        "&soil=" + String(soil) + "&relay_1=" + String(relay_1) + "&relay_2=" + String(relay_2) ;
        
        client.print(String("GET ") + httpData + " HTTP/1.1\r\n" + "Host: " + host + "\r\n" + "Connection: Close\r\n\r\n");
    }
    else
    {
        Serial.println("Ket noi voi may chu that bai");
        return;
    }
}

/*
void call_data(String node)
{
  if(client.connect(host,port))
  {
    String httpData = "/get_relay_node.php?node=" + String(node);
    client.print(String("GET ") + httpData + " HTTP/1.1\r\n" + "Host: " + host + "\r\n" + "Connection: Close\r\n\r\n");    
  }
  else
  {
    Serial.println("Ket noi voi may chu that bai!");
    return;
  }
}
 */
 
