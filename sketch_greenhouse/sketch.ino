#include <DHT.h>
#include <Servo.h> 
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
//#include "DHT.h"
#define DHTPIN 5 
const int relPin = 4;
unsigned long timing;
uint32_t myTimer1;

int value = 0;
int value1 = 0;
int sense_Pin = 0;
int led1 = 14;
int led2 = 12;
int led3 = 13;
int led4 = 15;

int fan = 0;

bool p = false;
// Пин для сервопривода

bool modee = false;
int poliv = 0;

DHT dht(DHTPIN, DHT22);

const char *ssid = "ROSTELECOM_97CF"; // Имя вайфай точки доступа
const char *pass = "MP7PC44J"; // Пароль от точки доступа

const char *mqtt_server = "m2.wqtt.ru"; // Имя сервера MQTT
const int mqtt_port = 5631; // Порт для подключения к серверу MQTT
const char *mqtt_user = "u_2CWPYH"; // Логи от сервер
const char *mqtt_pass = "716Q9lLe"; // Пароль от сервера
int otkat = 1000;

WiFiClient wclient;
PubSubClient client(wclient, mqtt_server, mqtt_port);

int servoPin = 4;
// Создаем объект
Servo Servo1;

void callback(const MQTT::Publish& pub)
{
Serial.print(pub.topic()); // выводим в сериал порт название топика
Serial.print(" => ");
Serial.print(pub.payload_string()); // выводим в сериал порт значение полученных данных

String payload = pub.payload_string();
  if(String(pub.topic()) == "test/pomp") // проверяем из нужного ли нам топика пришли данные
  {
    String stled = payload; // преобразуем полученные данные в тип integer
    //Serial.println("pomp:" + payload);// включаем или выключаем светодиод в зависимоти от полученных значений данных
    
    
    if(stled == "1"){
      digitalWrite(led1, HIGH);
      digitalWrite(led2, LOW);
      digitalWrite(led3, LOW);
      }
         
    if(stled == "2"){
      digitalWrite(led1, LOW);
      digitalWrite(led2, HIGH);
      digitalWrite(led3, LOW);
      }
    if(stled == "3"){
      digitalWrite(led1, LOW);
      digitalWrite(led2, LOW);
      digitalWrite(led3, HIGH);
      }
    if(stled == "4"){
        p = true;
        digitalWrite(led4, LOW);
      }

    if(stled == "p0"){

        digitalWrite(fan, HIGH);
      }
    if(stled == "p1"){

        digitalWrite(fan, LOW);
      }
      
  }
  else if(String(pub.topic()) == "test/mode") {
      
      if (getValue(payload,'.',0) == "hum"){
          modee = false;
          Serial.println(" f; " +getValue(payload,'.',1));
          poliv = getValue(payload,'.',1).toInt();
        }
    }
}



void setup() {
  dht.begin();
  Serial.begin(9600);
  delay(10);
  Serial.println();
  Serial.println();
  pinMode(DHTPIN, INPUT);
  pinMode(4, INPUT);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  pinMode(led4, OUTPUT);
  pinMode(fan, OUTPUT);
  digitalWrite(led4, HIGH);
  digitalWrite(fan, HIGH);
  Servo1.attach(servoPin);
  Servo1.write(-180);
  delay(500);
  Servo1.write(0);
}

void loop() {
  Serial.println("hum " + String(analogRead(A0)));
  try_connect();
  if (client.connected() ){
    client.loop();
    send_data();
    Serial.println("connected");
    client.set_callback(callback);
    client.subscribe("test/pomp");
     client.subscribe("test/mode");
     
  }
   client.set_callback(callback);
   client.subscribe("test/pomp");
    int water = analogRead(A0);

    value1 = map(water, 1048, 350, 0, 100);
    if(modee == false){
      Serial.println( " val " + String(value1) + " poliv :" +  String(poliv)); 
      if(value1 < poliv){
          p = true;
          
          digitalWrite(led4, LOW);
        }
    }

   if(p == true){
      if (millis() - myTimer1 >= 5000) {   
          myTimer1 = millis();
          digitalWrite(led4, HIGH);
          p = false;
          
       }
    }
 
}


void send_data(){
  if (millis() - timing > otkat){ 
    float t = dht.readTemperature();    
    float h = dht.readHumidity();    
    int water = analogRead(A0);

    value = map(water, 1048, 350, 0, 100);
    client.publish("test/temp",String(String(t) + " " + String(h)) + " " + String(value)); 
    

    //Serial.println(t);   
    //Serial.println(h);
    timing = millis();
  }
      
}

void check_data(){
  
  client.subscribe("test/pomp");
  }


void try_connect(){if (WiFi.status() != WL_CONNECTED) {
    Serial.print("Connecting to ");
    Serial.print(ssid);
    Serial.println("...");
    WiFi.begin(ssid, pass);

  if (WiFi.waitForConnectResult() != WL_CONNECTED)
  //return;
  Serial.println("WiFi connected");
  }

  if (WiFi.status() == WL_CONNECTED) {
    if (!client.connected()) {
      Serial.println("Connecting to MQTT server");
      if (client.connect(MQTT::Connect("arduinoClient2")
      .set_auth(mqtt_user, mqtt_pass))) {
        Serial.println("Connected to MQTT server");
      } else {
        Serial.println("Could not connect to MQTT server");
      }
    }
  }
}



String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}
