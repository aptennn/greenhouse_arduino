#include <AHTxx.h>
#include <Wire.h>
#include <GyverOLED.h>
#include <DHT.h> // USE LIBRARY FROM ADAFRUIT LIBARY AND USE ADAFRUIT DEPENCY
#include <ESP8266WiFi.h>
#include <PubSubClient.h> // USE LIBRARY FROM IMROY
// https://github.com/Imroy/pubsubclient

#define DHTPIN D6 
#define DHTTYPE DHT22

const int scale = 1;

DHT dht(DHTPIN, DHTTYPE); 
GyverOLED<SSH1106_128x64> oled;



String code = "12346"; // Код теплицы
const char *ssid = "ROSTELECOM_97CF"; // Имя вайфай точки доступа
const char *pass = "MP7PC44J"; // Пароль от точки доступа

int otkat = 1000;
unsigned long timing;
uint32_t myTimer1;
int value = 0;
int fan = D8;
int nasos = D4;
bool modee = false;
int poliv = 25;
bool p = false;



const char *mqtt_server = "m2.wqtt.ru"; // Имя сервера MQTT
const int mqtt_port = 5631; // Порт для подключения к серверу MQTT
const char *mqtt_user = "u_2CWPYH"; // Логи от сервер
const char *mqtt_pass = "716Q9lLe"; // Пароль от сервера

const int AirValue = 855;             // Максимальное значение сухого датчика
const int WaterValue = 450;           // Минимальное значение погруженного датчика
int soilMoistureValue = 0;            // Создаем переменную soilMoistureValue
int soilmoisturepercent = 0;  
uint8_t pinSensor = A0;

float ahtValue;                               //to store T/RH result

AHTxx aht10(AHTXX_ADDRESS_X38, AHT1x_SENSOR); //sensor address, sensor type


WiFiClient wclient;
PubSubClient client(wclient, mqtt_server, mqtt_port);


void callback(const MQTT::Publish& pub)
{
Serial.print(pub.topic()); // выводим в сериал порт название топика
Serial.print(" => ");
Serial.print(pub.payload_string()); // выводим в сериал порт значение полученных данных

String payload = pub.payload_string();
  String stled = payload;
  Serial.println("msg: " + payload);
  if(String(pub.topic()) == "test"+code+"/pomp") // проверяем из нужного ли нам топика пришли данные
  {
    //String stled = payload; // преобразуем полученные данные в тип integer
    //// включаем или выключаем светодиод в зависимоти от полученных значений данных
    
    
    if(stled == "p1"){

        digitalWrite(fan, HIGH);
      }
    if(stled == "p0"){

        digitalWrite(fan, LOW);
      }
      
  }
  else if(String(pub.topic()) == "test"+code+"/mode") {
      
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
  aht10.begin();
  pinMode(fan, OUTPUT);
  pinMode(nasos, OUTPUT);
  pinMode(pinSensor, INPUT);
  oled.init();  // инициализация

  oled.clear();  
  oled.update(); 

  oled.setCursor(1, 1);   // курсор в (пиксель X, строка Y)
  oled.setScale(scale);
  oled.print("Temp:");
  oled.setCursor(1, 2);   // курсор в (пиксель X, строка Y
  oled.print("Hum:");
  oled.setCursor(1, 4);   // курсор в (пиксель X, строка Y)
  oled.setScale(scale);
  oled.print("Temp:");
  oled.setCursor(1, 5);   // курсор в (пиксель X, строка Y
  oled.print("Hum:");
  oled.update();
  Serial.print("Setup init");

}

void loop() {
  try_connect();
  if (client.connected() ){
    client.loop();
    float h = dht.readHumidity();
    // Read temperature as Celsius (the default)
    float t = dht.readTemperature();
    soilMoistureValue = analogRead(pinSensor);   // Считываем данные с порта A0 и записываем их в переменную
    soilmoisturepercent = map(soilMoistureValue, AirValue, WaterValue, 0, 100);
    
    oled.setCursor(64, 1);   // курсор в (пиксель X, строка Y)
    oled.print(String(t));
    oled.setCursor(64, 2);   // курсор в (пиксель X, строка Y)
    oled.print(String(h));

    ahtValue = aht10.readTemperature();
    oled.setCursor(64, 4);   // курсор в (пиксель X, строка Y)
    oled.print(String(ahtValue));
    oled.setCursor(64, 5);   // курсор в (пиксель X, строка Y)
    ahtValue = aht10.readHumidity();
    oled.print(String(ahtValue));
    oled.update();
    send_data(t, h, soilmoisturepercent);
    Serial.println("connected");

    
    client.set_callback(callback);
    client.subscribe("test"+code+"/pomp");
    client.subscribe("test"+code+"/mode");


    if(modee == false){
    if(soilmoisturepercent < poliv){
          p = true;
          digitalWrite(D4, HIGH);
        }
    }

    if(p == true){
      if (millis() - myTimer1 >= 5000) {   
          myTimer1 = millis();
          p = false;
          digitalWrite(D4, LOW);
          
       }
    }
     
  }
  
}



void send_data(float t, float h, float soilmoisturepercent){
  if (millis() - timing > otkat){ 
    //float t = dht22.readTemperature();    
    //float h = dht22.readHumidity();    
    int water = 400;


    

    value = map(water, 1048, 350, 0, 100);
    client.publish("test"+code+"/temp",String(String(t) + " " + String(h)) + " " + String(soilmoisturepercent)); 
    

    Serial.println(" temp " + String(t) + " hum " + String(h));   
    Serial.println(t);
    timing = millis();
  }
      
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
