#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <HX711.h>
#include <WiFi.h>

const char * WIFI_SSID = "Galaxy A15";
const char * WIFI_PASS = "72726315";
// const char * WIFI_SSID = "Flia. VIGABRIEL";
// const char * WIFI_PASS = "71717805ELI";
//const char * WIFI_SSID = "UCB-PREMIUM";//nombre del router
//const char * WIFI_PASS = "lacatoucb";//contraseña

// const char * WIFI_SSID = "TP-Link_65A0";
// const char * WIFI_PASS = "P@trick.72238400";

const char * MQTT_BROKER_HOST = "a10nhk4kc31osm-ats.iot.us-east-2.amazonaws.com";
const int MQTT_BROKER_PORT = 8883;

const char * MQTT_CLIENT_ID = "ESP-32";                                         // Unique CLIENT_ID

const char * UPDATE_TOPIC = "$aws/things/thing/shadow/update";              // publish
const char * SUBSCRIBE_TOPIC = "$aws/things/thing/shadow/update/delta";     // subscribe

const char * UPDATE_GET_TOPIC = "$aws/things/thing/shadow/get";              // publish
const char * SUBSCRIBE_GET_TOPIC = "$aws/things/thing/shadow/get/accepted"; // subscribe

const int PIN_MOTOR = 13;
const int TRIGGER_PIN = 4;
const int ECHO_PIN = 2;

const int DT = 25;
const int sck = 26;

const char AMAZON_ROOT_CA1[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----
)EOF";

const char CERTIFICATE[] PROGMEM = R"KEY(
-----BEGIN CERTIFICATE-----
MIIDWTCCAkGgAwIBAgIUFF6WG+U7ka2Oc5ctOka8LpgrKPYwDQYJKoZIhvcNAQEL
BQAwTTFLMEkGA1UECwxCQW1hem9uIFdlYiBTZXJ2aWNlcyBPPUFtYXpvbi5jb20g
SW5jLiBMPVNlYXR0bGUgU1Q9V2FzaGluZ3RvbiBDPVVTMB4XDTI0MDYwMTIzMzQx
N1oXDTQ5MTIzMTIzNTk1OVowHjEcMBoGA1UEAwwTQVdTIElvVCBDZXJ0aWZpY2F0
ZTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALf72blxdujzCHeUUBRr
7ZZbGM7RsM2CCqVk7suxlq9vuqP1TtOz7JnvDHWdltM/6VpqZ6yEeFI4hC8WjDhL
27RjaxFUQHlMjISJw7W3B9y2ns2p0pwtl+IGYJOLLCfctaWHiI1a4dV2mNQFBf46
NbVDuu17EMyx7qmIHst20wGTvz3Lq0Vg5mbuxmJyXcTl1xdjj7tmKL8aKcuNWjeb
7xm2xWXpKm5XcrCB8Eg2OLYT2Hfax2qP5JelOUyFHswHux7QmUVNkks7ugql/l3f
tzeI6W5FKhsTC7FGSk2dDzvxHOoamigZENJZoLoOnGVwgndyPdIRVcXVD4ckOJrH
wA8CAwEAAaNgMF4wHwYDVR0jBBgwFoAUR2hq15vQqWHpgd5j+Bo9plGB6g8wHQYD
VR0OBBYEFGNYmoed5cVFRomuFLBGoNcC8suhMAwGA1UdEwEB/wQCMAAwDgYDVR0P
AQH/BAQDAgeAMA0GCSqGSIb3DQEBCwUAA4IBAQAKK3cSz3MuBECDj+UDZSZGiHa6
gZrTW4p7ZEOpmPHEgxw1mlqfbjdNev8ZeCpuxdmBj1bpabRYXjO/KutSuFs4FVE1
npneo2WMGBgLnJA9Z2WiTB1841E4F7v0smJgj6QIspdl0RfvQSWkUUa6r8Kom6Oe
+XE1p3IUZj87BDnX5P57CoePMBLzAEDfNXDuSndQNcdxspMhvdguzN66q/b19tE+
JjZgl9HoBTpNZh8EKSYeE6XG95kBwklDujWVZyiUrYwxz8E1zZaNdWpozPpdko07
MEjnukFZFz8ASVJE2GL5wjzfp/RH6pOrvAq5mmyIYLgguS5VsNps2g1HpIRq
-----END CERTIFICATE-----
)KEY";

const char PRIVATE_KEY[] PROGMEM = R"KEY(
-----BEGIN RSA PRIVATE KEY-----
MIIEpAIBAAKCAQEAt/vZuXF26PMId5RQFGvtllsYztGwzYIKpWTuy7GWr2+6o/VO
07Psme8MdZ2W0z/pWmpnrIR4UjiELxaMOEvbtGNrEVRAeUyMhInDtbcH3LaezanS
nC2X4gZgk4ssJ9y1pYeIjVrh1XaY1AUF/jo1tUO67XsQzLHuqYgey3bTAZO/Pcur
RWDmZu7GYnJdxOXXF2OPu2Yovxopy41aN5vvGbbFZekqbldysIHwSDY4thPYd9rH
ao/kl6U5TIUezAe7HtCZRU2SSzu6CqX+Xd+3N4jpbkUqGxMLsUZKTZ0PO/Ec6hqa
KBkQ0lmgug6cZXCCd3I90hFVxdUPhyQ4msfADwIDAQABAoIBAGps/xbscZJy81To
sCIABKFS9Ni+J33ZWXsvFYsyoS2+fcJWJlED5TtuMlDe5d4xPDUXE/2Ra2B0XNI4
QshV6TNBxy9xAxKnmykX3+P/ika8RMRHFhNNtGwEsFzoxpQQcercqbrK9ZPbVLBg
b4UN+v9tvHWfGKqvypS6rsWXaJuh9bgCaqhV+fPC7/zVMTj+S7LyOK3iKvY0xVOq
Yr75SAJ6H7VPuDeCNnVRDgGv+IgesuUXs+a086+tZ2N1jiQGeP6C3JGRlNnMDmoA
cSMDBamelIwFwdlaK8eIWyQ04TEMT7gaq/wBDCPoiY2TnizsFO7lhMlgX0IE3qoR
S8YzKYECgYEA61vcfYlLT6OZPTR+GoFWbdD6SxaESq3+TSghXYkX2Q7ZpR86Fyny
b4878nmdFGWoJYfLthzxCPXOTaJvrxENsPk04Q5ppTVjvdXflzkWCpevk1EIjj8N
OxSJcwANNJhyUU1CbmrLRIBZIkij7A3Gp5G8B14BdKopqrQOqA8Vs+8CgYEAyB6L
/Q4gg1f4tovuoWki3/65QFJQ//ofvbwEgPldPLX3WlPJyuwovJRbx9jLjppo81IX
rXCDcFNL9Bl/yGXfkzyAWYzdziJwOD0MV1GZOZQqOIF11U4a4Efk5PCdokescHk9
puEudshOmdF5hQ2Yvo5JWHtLEgXiTaR594QuFeECgYEAmqnsNEbTycrteY4UTGCw
JkxNHqj2WIkMczGq82eS113t4pepue0j1vHeaBJJCk1feQJK+Sr9rnCxmMzk7buc
Lq8lc7vf6uQx3l88poiqFl3l0D5RDAEdNbTiOkNPaj+/5/OIcz0UxLg/Wss3hXjs
EdQwJvs8o6jq5rWaaBA5p9sCgYAIMuAvXO3OE8OXzwHW0RThsnDqhrTU1+2G+3X/
xregLiBVLgudabF9kGJ2PJaSYBceqOVZcLuh1XfOJ2FJ14qiYJ8tjAzmThjk5PNl
fG1Xo49bQ0qNk6acO7XP/1+l72PMM9tnw+AAc9JskN12qwjv/apZmquHsIAGCFAK
6PpuYQKBgQDDEBDlmu4Kr3CBb+CllnrsbGR6XG1Xe9h4ioNjRyQezbMFi4dPXX8v
YZF9cxLTYS70xO3mW/v3U0KzW2jmJcUAJwrUb/jsBf/TK+7cp6u8vGMky21AiZ0y
XyPs/G9q6DyVLo6QSmGBZZWqndSIUZEGmeH/8emyw54ozqBh4SDOEg==
-----END RSA PRIVATE KEY-----
)KEY";

StaticJsonDocument<JSON_OBJECT_SIZE(64)> inputDoc;

StaticJsonDocument<JSON_OBJECT_SIZE(4)> outputDoc;

char outputBuffer[128];

unsigned char builtInMotor = 0;

static bool voidReported = false;
static bool fullReported = false;
static bool catCloseReported = false;
static bool catAwayReported = false;

class Wifi {
private:
    String wifi_SSID;
    String wifi_PASS;

public:
    // Constructor que recibe el nombre y la contraseña del wifi
    Wifi(String ssid, String pass) : wifi_SSID(ssid), wifi_PASS(pass) {}

    // Método para conectar al wifi
    void connectWifi() {
        WiFi.begin(wifi_SSID.c_str(), wifi_PASS.c_str());
        while (WiFi.status() != WL_CONNECTED) {
          delay(200);
          Serial.print(".");
        }
        Serial.println(" Connected to WiFi!");
    }
};

class Actuator {
  public:
      Actuator(int pin) : pin_(pin) {
          pinMode(pin_, OUTPUT);
      }

      void turnOn() {
          digitalWrite(pin_, HIGH);
      }

      void turnOff() {
          digitalWrite(pin_, LOW);
      }

  protected:
      int pin_;
};

class Motor : public Actuator {
  public:
    Motor(int pin) : Actuator(pin) {}

    void turnOnMotor() {
      turnOn();
    }

    void turnOffMotor() {
      turnOff();
    }

    void turnOnMotorForTime() {
      turnOn();
      delay(5000);
      turnOff();
      delay(200);
    }
};

class Sensor {
  protected:
    int sensorPin;
    float sensorValue;

  public:
    Sensor(int pin) : sensorPin(pin), sensorValue(0.0) {
        pinMode(sensorPin, INPUT);
    }

    virtual void readSensor() {}

    float getValue() const {
        return sensorValue;
    }
};

class UltrasonicSensor : public Sensor {
  private:
    int triggerPin;
    int echoPin;

  public:
    UltrasonicSensor(int triggerPin, int echoPin) : Sensor(triggerPin), triggerPin(triggerPin), echoPin(echoPin) {}

    long readUltrasonicDistance() {
      pinMode(triggerPin, OUTPUT);  // Clear the trigger
      digitalWrite(triggerPin, LOW);
      delayMicroseconds(20);
      // Sets the trigger pin to HIGH state for 10 microseconds
      digitalWrite(triggerPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(triggerPin, LOW);
      pinMode(echoPin, INPUT);
      // Reads the echo pin, and returns the sound wave travel time in microseconds
      return pulseIn(echoPin, HIGH);
    }
};

//////////////////////////////////////////////////////////////////////////////////
WiFiClientSecure wiFiClient;
PubSubClient mqttClient(wiFiClient);

Motor motor(PIN_MOTOR);
Wifi wifi(WIFI_SSID, WIFI_PASS);
UltrasonicSensor ultrasonicSensor(TRIGGER_PIN, ECHO_PIN);

HX711 balance;

void reportBuiltInMotor(String dataType, int data)
{
  outputDoc["state"]["reported"][dataType] = data;
  serializeJson(outputDoc, outputBuffer);
  mqttClient.publish(UPDATE_TOPIC, outputBuffer);
}

///////////////////////////////////////////////////////////////////////

void reportInTopic(String stateDish, int stateMotor)
{
  outputDoc["state"]["reported"]["stateDish"] = stateDish;
  outputDoc["state"]["reported"]["builtInMotor"] = stateMotor;
  outputDoc["state"]["desired"]["builtInMotor"] = 0;
  Serial.println("Se envio el reporte");
  serializeJson(outputDoc, outputBuffer);
  mqttClient.publish(UPDATE_TOPIC, outputBuffer);
}

void setBuiltInMotor()
{
  if (builtInMotor == 1) {
    Serial.println("Motor Prendido");
    motor.turnOnMotor();
    delay(5000);
    motor.turnOffMotor();
    builtInMotor = 0;
  }

  Serial.println(builtInMotor);
  reportInTopic("Full", builtInMotor);
}
///////////////////////////////////////////////////////////////////////
void restaurarValores() {
  voidReported = false;
  fullReported = false;
  catCloseReported = false;
  catAwayReported = false;
}
///////////////////////////////////////////////////////////////////////

void callback(const char * topic, byte * payload, unsigned int length) 
{
  String message;
  for (int i = 0; i < length; i++) message += String((char) payload[i]);
  Serial.println("Message from topic " + String(topic) + ":" + message);
  DeserializationError err = deserializeJson(inputDoc, payload);
  
  if (!err) 
  {
    if (String(topic) == SUBSCRIBE_TOPIC) {
      builtInMotor = inputDoc["state"]["builtInMotor"].as<int8_t>();
      if (builtInMotor == 2) {
        builtInMotor = 1;
        restaurarValores();
      }
      Serial.println(builtInMotor);
      setBuiltInMotor();
      delay(200);
    }
  }
}

String medirRangos(int peso) {
  if (peso >= 20)
    return "Full";
  else
    return "Void";
}

void setup() 
{
  Serial.begin(115200);

  wifi.connectWifi();

  balance.begin(DT,sck);
  balance.tare();
  balance.set_scale();

  // SETEANDO CERTIFICADOS XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
  wiFiClient.setCACert(AMAZON_ROOT_CA1);
  wiFiClient.setCertificate(CERTIFICATE);
  wiFiClient.setPrivateKey(PRIVATE_KEY);
  // XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

  // CONECTANDO AL BROKER DE AWS WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW
  mqttClient.setServer(MQTT_BROKER_HOST, MQTT_BROKER_PORT);
  mqttClient.setCallback(callback);

  Serial.print("Connecting to " + String(MQTT_BROKER_HOST));
  if (mqttClient.connect(MQTT_CLIENT_ID)) {
    mqttClient.subscribe(SUBSCRIBE_TOPIC);
    Serial.println("Subscribed to " + String(SUBSCRIBE_TOPIC));

    mqttClient.subscribe(SUBSCRIBE_GET_TOPIC);
    Serial.println("Subscribed to " + String(SUBSCRIBE_GET_TOPIC));

    delay(100);
    reportInTopic("Full", builtInMotor);
  }
}

void reportState(const char* stateDish, const char* catState) {
    outputDoc["state"]["reported"]["stateDish"] = stateDish;
    outputDoc["state"]["reported"]["cat"] = catState;
    serializeJson(outputDoc, outputBuffer);
    mqttClient.publish(UPDATE_TOPIC, outputBuffer);
}

unsigned long previousPublishMillis = 0;

void loop() {
  float distance = 0.01723 * ultrasonicSensor.readUltrasonicDistance();
  unsigned long now = millis();
  int Time = 5000;

  int reading = balance.get_units(10);
  int weight = (reading*100/10500)-1035;

  String stateDish = medirRangos(weight);

  if (!isnan(distance) && now - previousPublishMillis >= 5000) {
    previousPublishMillis = now;
    
    Serial.println("Peso: " + String(weight));
    Serial.println("Distancia: " + String(distance));
    Serial.println("Estado Plato: " + String(stateDish));

    if (distance <= 30) {
      if (stateDish == "Void") {
        outputDoc["state"]["reported"]["stateDish"] = "Void";
        outputDoc["state"]["reported"]["cat"] = "close";
        serializeJson(outputDoc, outputBuffer);
        mqttClient.publish(UPDATE_TOPIC, outputBuffer);

      } else if (stateDish == "Full" && !fullReported) {
        voidReported = false;
        outputDoc["state"]["reported"]["stateDish"] = "Full";
        outputDoc["state"]["reported"]["cat"] = "close";
        serializeJson(outputDoc, outputBuffer);
        mqttClient.publish(UPDATE_TOPIC, outputBuffer);
        fullReported = true;

      } else if (!catCloseReported) {
        catAwayReported = false;
        outputDoc["state"]["reported"]["cat"] = "close";
        serializeJson(outputDoc, outputBuffer);
        mqttClient.publish(UPDATE_TOPIC, outputBuffer);
        catCloseReported = true;
      }
      delay(5000);

    } else if (stateDish == "Void" && !voidReported) {
      fullReported = false;
      outputDoc["state"]["reported"]["stateDish"] = "Void";
      outputDoc["state"]["reported"]["cat"] = "away";
      serializeJson(outputDoc, outputBuffer);
      mqttClient.publish(UPDATE_TOPIC, outputBuffer);
      voidReported = true;

    } else if (stateDish == "Full" && !fullReported) {
      voidReported = false;
      outputDoc["state"]["reported"]["stateDish"] = "Full";
      outputDoc["state"]["reported"]["cat"] = "away";
      serializeJson(outputDoc, outputBuffer);
      mqttClient.publish(UPDATE_TOPIC, outputBuffer);
      fullReported = true;

    } else if (!catAwayReported) {
      catCloseReported = false;
      outputDoc["state"]["reported"]["cat"] = "away";
      serializeJson(outputDoc, outputBuffer);
      mqttClient.publish(UPDATE_TOPIC, outputBuffer);
      catAwayReported = true;
    }
    /*
    presentCat = estadoAnteriorGato();
    presentStateDish = estadoAnteriorPlato();
    estadoAnteriorMotor();
    */
    delay(5000);
  }

  if (mqttClient.connected()) {
    mqttClient.loop();
  } 
  else {
    Serial.println("MQTT broker not connected. Reconnecting...");
    if (mqttClient.connected()) {
      Serial.println("Reconnected to MQTT broker");
      mqttClient.subscribe(SUBSCRIBE_TOPIC);
      mqttClient.subscribe(SUBSCRIBE_GET_TOPIC);
      //mqttClient.setServer(MQTT_BROKER_HOST, MQTT_BROKER_PORT);
      //mqttClient.setCallback(callback);
      
    } 
    else {
      Serial.println("Failed to reconnect. Waiting to retry...");
      delay(2000);
    }
  }
}