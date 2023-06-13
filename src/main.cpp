#include <WiFi.h>
#include <FirebaseESP32.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <DHT_U.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>



#define FIREBASE_HOST "https://finalsmartparking-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define FIREBASE_AUTH "BlJSSAFJWP9oIIv30HHnmE3rQlfawqF6bit3BZxO"

#define SENSOR1_PIN 13
#define SENSOR2_PIN 14
#define SENSOR3_PIN 15

#define RED_LED_PIN1 5
#define GREEN_LED_PIN1 4
#define RED_LED_PIN2 18
#define GREEN_LED_PIN2 19
#define RED_LED_PIN3 21
#define GREEN_LED_PIN3 22

#define DHT_PIN 25
#define DHT_TYPE DHT11

#define RAINDROP_PIN 34

DHT dht(DHT_PIN, DHT_TYPE);

#define LED_ON HIGH
#define LED_OFF LOW

FirebaseData firebaseData;

struct WifiCredentials {
  const char* ssid;
  const char* password;
};

WifiCredentials wifiCredentials[] = {
  {"S22 Ultra", "Daniel12"},
  {"B29 2.4GHz", "68205713"}
};

void connectToWiFi(int index) {
  WiFi.begin(wifiCredentials[index].ssid, wifiCredentials[index].password);
  Serial.print("Connecting to Wi-Fi");
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 5000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Connected to Wi-Fi");
  } else {
    Serial.println("pindah ke wifi selanjutnya...");
    delay(2000);
    int nextIndex = (index + 1) % (sizeof(wifiCredentials) / sizeof(wifiCredentials[0]));
    connectToWiFi(nextIndex);
  }
}
void setup() {
  Serial.begin(115200);
  connectToWiFi(0); 
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  dht.begin();
  pinMode(SENSOR1_PIN, INPUT);
  pinMode(SENSOR2_PIN, INPUT);
  pinMode(SENSOR3_PIN, INPUT);
  pinMode(RED_LED_PIN1, OUTPUT);
  pinMode(GREEN_LED_PIN1, OUTPUT);
  pinMode(RED_LED_PIN2, OUTPUT);
  pinMode(GREEN_LED_PIN2, OUTPUT);
  pinMode(RED_LED_PIN3, OUTPUT);
  pinMode(GREEN_LED_PIN3, OUTPUT);
}

void updateFirebaseData(const char* sensorId, bool hasObstacle, float temperature, float humidity, int raindropValue) {
  FirebaseJson json;
  json.add(sensorId, hasObstacle);
  json.add("temperature", temperature);
  json.add("humidity", humidity);
  json.add("raindropValue", raindropValue);

  String rainDescription;
  if (raindropValue >= 3000 && raindropValue <= 3999) {
    rainDescription = "There is light rain";
  } else if (raindropValue >= 2000 && raindropValue <= 2999) {
    rainDescription = "There is rain outside";
  } else if (raindropValue >= 1000 && raindropValue <= 1999) {
    rainDescription = "There is heavy rain";
  } else {
    rainDescription = "There is no rain";
  }
  
  json.add("temperature", temperature);
  json.add("humidity", humidity);
  json.add("raindropValue", raindropValue);
  json.add("rainDescription", rainDescription);
  
  Firebase.updateNode(firebaseData, "/sensors", json);
  
  if (firebaseData.dataPath() == "/sensors") {
    Serial.println("Data kekirim");
  } else {
    Serial.println("Failed ");
    Serial.println(firebaseData.errorReason());
  }
}

void updateLEDState(int redLedPin, int greenLedPin, bool hasObstacle) {
  digitalWrite(redLedPin, hasObstacle ? LED_ON : LED_OFF);
  digitalWrite(greenLedPin, hasObstacle ? LED_OFF : LED_ON);
}

void loop() {
  bool hasObstacle1 = digitalRead(SENSOR1_PIN) == HIGH;
  bool hasObstacle2 = digitalRead(SENSOR2_PIN) == HIGH;
  bool hasObstacle3 = digitalRead(SENSOR3_PIN) == HIGH;
  

  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();


  int raindropValue = analogRead(RAINDROP_PIN);


  updateFirebaseData("sensor1", hasObstacle1, temperature, humidity, raindropValue);
  updateFirebaseData("sensor2", hasObstacle2, temperature, humidity, raindropValue);
  updateFirebaseData("sensor3", hasObstacle3, temperature, humidity, raindropValue);

  updateLEDState(RED_LED_PIN1, GREEN_LED_PIN1, hasObstacle1);
  updateLEDState(RED_LED_PIN2, GREEN_LED_PIN2, hasObstacle2);
  updateLEDState(RED_LED_PIN3, GREEN_LED_PIN3, hasObstacle3);

  delay(500); 
}
