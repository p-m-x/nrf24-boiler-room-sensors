/*
 *************************************************************************
   RF24Ethernet Arduino library by TMRh20 - 2014-2015

   Automated (mesh) wireless networking and TCP/IP communication stack for RF24 radio modules

   RF24 -> RF24Network -> UIP(TCP/IP) -> RF24Ethernet
                       -> RF24Mesh

        Documentation: http://tmrh20.github.io/RF24Ethernet/

 *************************************************************************
 *
 **** EXAMPLE REQUIRES: PubSub MQTT library: https://github.com/knolleary/pubsubclient ***
 * 
 * Install using the Arduino library manager
 * 
 *************************************************************************
  Basic MQTT example

 This sketch demonstrates the basic capabilities of the library.
 It connects to an MQTT server then:
  - publishes "hello world" to the topic "outTopic"
  - subscribes to the topic "inTopic", printing out any messages
    it receives. NB - it assumes the received payloads are strings not binary

 It will reconnect to the server if the connection is lost using a blocking
 reconnect function. See the 'mqtt_reconnect_nonblocking' example for how to
 achieve the same result without blocking the main loop.
 
*/

#include <SPI.h>
#include <RF24.h>
#include <RF24Network.h>
#include <RF24Mesh.h>
#include <RF24Ethernet.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 2
#define TEMPERATURE_PRECISION 9
#define MQTT_PORT 1883
#define MESH_CONNECTION_CHECK_INTERVAL_MS 3000
#define TEMP_CONVERSION_INTERVAL_MS 1000
#define SEND_DATA_INTERVAL_MS 5000

OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature tempSensors(&oneWire);
unsigned char ds18B20deviceCount = 0;

RF24 radio(9,10);
RF24Network network(radio);
RF24Mesh mesh(radio,network);
RF24EthernetClass RF24Ethernet(radio,network,mesh);

IPAddress ip(10,10,1,2);
IPAddress gateway(10,10,1,1); //Specify the gateway in case different from the server
IPAddress server(10,10,1,1);
bool isGatewayReady = false;
char *clientID = {"BoilerRoomClient"};

EthernetClient ethClient;
PubSubClient client(ethClient);

uint32_t mesh_timer = 0;
uint32_t temp_conversion_timer = 0;
uint32_t send_data_timer = 0;
String topicTemperatureSensor = "sensors/boiler-room/temperature/t";


void printDS18B20Address(DeviceAddress deviceAddress);
void mqttCallbackFunc(char* topic, byte* payload, unsigned int length);

void reconnect() {
  // Loop until we're reconnected
  if (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(clientID)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic","hello world");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup()
{
  Serial.begin(9600);

  // locate devices on the bus
  Serial.print(F("Locating DS18B20 devices..."));
  tempSensors.begin();
  tempSensors.setCheckForConversion(false);
  tempSensors.setResolution(10);
  ds18B20deviceCount = tempSensors.getDS18Count();
  Serial.print(F(" Found "));
  Serial.print(ds18B20deviceCount, DEC);
  Serial.println(F(" device(s)."));

  // Print found devices addresses
  DeviceAddress termometerAddr;
  for (int i = 0; i < ds18B20deviceCount; i++) {
    tempSensors.getAddress(termometerAddr, i);
    Serial.print(F(" #1 "));
    printDS18B20Address(termometerAddr);
    Serial.println();
  }
  Serial.println();

  // Start gateway connection
  Serial.print("Starting gateway...");
  client.setServer(server, MQTT_PORT);
  client.setCallback(mqttCallbackFunc);
  Ethernet.begin(ip);
  Ethernet.set_gateway(gateway);
  if (mesh.begin()) {
    Serial.println(" OK");
    isGatewayReady = true;
  } else {
    Serial.println(" Failed");
  }
  clientID[13] = ip[3] + 48; //Convert last octet of IP to ascii & use in clientID
}



void loop()
{
  // Test mesh connectivity
  if(millis() - mesh_timer > MESH_CONNECTION_CHECK_INTERVAL_MS) { 
    mesh_timer = millis();
    if (!mesh.checkConnection()) {
      mesh.renewAddress();
    }
  }  
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Send temperatures
  if (client.connected()) {
    if (millis() - temp_conversion_timer > TEMP_CONVERSION_INTERVAL_MS) {
      temp_conversion_timer = millis();
      tempSensors.requestTemperatures();      
    }

    if (millis() - send_data_timer > SEND_DATA_INTERVAL_MS) {      
      if (tempSensors.isConversionComplete()) {
        send_data_timer = millis();
        char payload[10];
        char topic[50];

        for (int i = 0; i < ds18B20deviceCount; i++) {
          dtostrf(tempSensors.getTempCByIndex(i), 4, 2, payload);
          sprintf(topic, "%s%d", "sensors/boiler-room/temperature/t", i);

          Serial.print(F("Publishing "));
          Serial.print(topic);
          Serial.print(F(": "));   
          Serial.println(payload);

          client.publish(topic, payload);
        }
      } else {
        Serial.println(F("ERROR - temp conversion did not finish yet"));
      }
    }
  }
  
}

// function to print a device address
void printDS18B20Address(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

void mqttCallbackFunc(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}