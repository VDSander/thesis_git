#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include "PubSubClient.h"

//#define SSID          "TP-Link_5980"
//#define PWD           "sander2001"
//#define MQTT_SERVER   "192.168.0.102"
//#define SSID          "telenet-412D4EB"
//#define PWD           "QR8raunyjzya"
// #define MQTT_SERVER "192.168.0.149"
#define SSID          "Techtile"
#define PWD           "Techtile229"
#define MQTT_SERVER   "192.108.0.79"

#define MQTT_PORT 1883

#define LED_2 14

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];

int D2 = 4;
int D3 = 0;
SoftwareSerial uartSerial(D2, D3);
float value = 10.0;
float previousValue = value;
int number = 10;
int previousNumber = number;


void callback(char *topic, byte *message, unsigned int length);
void handleMessage(String message);

void setup_wifi() {
  delay(10);
  Serial.println("Connecting to WiFi..");

  WiFi.begin(SSID, PWD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(9600);
  uartSerial.begin(4800);

  setup_wifi();
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(callback);

  pinMode(D2, INPUT);
  pinMode(D3, OUTPUT);

  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);

  previousValue = value;
  previousNumber = number;
}

void callback(char *topic, byte *message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp = "";

  digitalWrite(2, !digitalRead(2));

  for (int i = 0; i < length; i++) {
    messageTemp += (char)message[i];
  }
  Serial.println();
  Serial.print("'" + messageTemp + "'");
  Serial.println(" ontvangen");
  handleMessage(messageTemp);
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("zendstation")) {
      Serial.println("connected");
      client.subscribe("sander/botToZendstation");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  value = uartSerial.parseFloat();
  if (uartSerial.read() == '\n') {
    if (value != 404) {
      Serial.print("\n");
      Serial.print("message received: ");
      Serial.println(value);
      client.publish("sander/zendstationToBot", "number was delivered");
    } else {
      Serial.print("\n");
      Serial.println("Error occurred, check zendstation");
      client.publish("sander/zendstationToUser", "Error occurred, check zendstation");
    }
  }
}

void handleMessage(String message) {
  if (message == "start transmission") {
    Serial.println("recognized start transmission");
    uartSerial.print(0);
    uartSerial.print("\n");
  } else if (message == "stop transmission") {
    Serial.println("recognized stop transmission");
    uartSerial.print(-1);
    uartSerial.print("\n");
  } else if (message == "more power") {
    Serial.println("recognized more power");
    uartSerial.print(1);
    uartSerial.print("\n");
  } else if (message == "less power") {
    Serial.println("recognized less power");
    uartSerial.print(2);
    uartSerial.print("\n");
  }
}