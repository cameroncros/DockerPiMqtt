#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#define DEV_ADDR 0x17
#define DATA_LEN 0x0D
uint8_t data[DATA_LEN + 1] = {0};

#define TEMP_REG 0x01
#define LIGHT_REG_L 0x02
#define LIGHT_REG_H 0x03
#define STATUS_REG 0x04
#define ON_BOARD_TEMP_REG 0x05
#define ON_BOARD_HUMIDITY_REG 0x06
#define ON_BOARD_SENSOR_ERROR 0x07
#define BMP280_TEMP_REG 0x08
#define BMP280_PRESSURE_REG_L 0x09
#define BMP280_PRESSURE_REG_M 0x0A
#define BMP280_PRESSURE_REG_H 0x0B
#define BMP280_STATUS 0x0C
#define HUMAN_DETECT 0x0D

// Replace with your network credentials
const char* ssid = "AirportBest";
const char* password = "Yamaha4Life";
const char* mqtt_server = "192.168.1.100";

// Change room, must be unique
#define ROOM "Kitchen"
#define DEBUG 1
#define PREFIX "homeassistant"

WiFiClient espClient;
PubSubClient client(espClient);

void PrintHex8(uint8_t *data, uint8_t length) // prints 8-bit data in hex with leading zeroes
{
  for (int i = 0; i < length; i++) {
    if (data[i] < 0x10) Serial.print("0");
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void setup() {
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(115200);  // start serial for output

  // Connect to your wi-fi modem
  WiFi.begin(ssid, password);

  // Check wi-fi is connected to wi-fi network
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected successfully");
  Serial.print("Got IP: ");
  Serial.println(WiFi.localIP());  //Show ESP32 IP on serial

  client.setServer(mqtt_server, 1883);
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client" ROOM, "mqttuser", "mqttpassword")) {
      Serial.println("connected");
      client.setBufferSize(512);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

#define CONFIG_SENSOR(TYPE, ID, NAME, DC, UNIT) { \
    StaticJsonDocument<500> doc; \
    doc["name"] =  NAME; \
    doc["device_class"] = DC; \
    doc["state_topic"] = PREFIX "/" TYPE "/" ROOM "/" ID "/state"; \
    doc["unit_of_measurement"] = UNIT; \
    doc["unique_id"] = ROOM "_" ID; \
    JsonObject device = doc.createNestedObject("device"); \
    device["name"] = "Dockerpi Sensorhub " ROOM; \
    device["model"] = "Sensorhub"; \
    device["manufacturer"] = "dockerpi"; \
    device["sw_version"] = "0.1"; \
    JsonArray ids = device.createNestedArray("identifiers"); \
    ids.add("dockerpi_sensorhub_" ROOM); \
    String configStr; \
    serializeJson(doc, configStr); \
    Serial.println(configStr); \
    client.publish(PREFIX "/" TYPE "/" ROOM "/" ID "/config", "", true); \
    bool ret = client.publish(PREFIX "/" TYPE "/" ROOM "/" ID "/config", configStr.c_str(), true); \
    Serial.print("Configured " NAME ":"); \
    Serial.println(ret); \
}


#define PUBLISH_DATA(TYPE, ID, VAR) { \
    client.publish(PREFIX "/" TYPE "/" ROOM "/" ID "/state", String(VAR).c_str()); \
}

void setupSensors()
{
  CONFIG_SENSOR("sensor", "temp1", "Temp 1", "temperature", "ºC");
  CONFIG_SENSOR("sensor", "temp2", "Temp 2", "temperature", "ºC");
  CONFIG_SENSOR("sensor", "temp_ext", "Temp Ext", "temperature", "ºC");
  CONFIG_SENSOR("sensor", "light", "Light", "illuminance", "Lx");
  CONFIG_SENSOR("sensor", "humidity", "Humidity", "humidity", "%");
  CONFIG_SENSOR("sensor", "pressure", "Pressure", "pressure", "mPa");
  CONFIG_SENSOR("binary_sensor", "motion", "Motion", "motion", "");
}

void loop() {
  if (!client.connected()) {
    reconnect();
    setupSensors();
  }
  client.loop();

  Wire.beginTransmission(DEV_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.requestFrom(DEV_ADDR, DATA_LEN);    // request DATA_LEN bytes from slave device DEV_ADDR
  int i = 0;
  while (Wire.available()) { // slave may send less than requested
    assert(i < DATA_LEN + 1);
    data[i++] = Wire.read(); // receive a byte as character
  }

  uint32_t temp1 = data[TEMP_REG];
  uint32_t temp2 = data[ON_BOARD_TEMP_REG];
  uint32_t temp3 = data[BMP280_TEMP_REG];
  uint32_t light = data[LIGHT_REG_H] << 8 | \
                   data[LIGHT_REG_L];
  uint32_t humidity = data[ON_BOARD_HUMIDITY_REG];
  uint32_t pressure = data[BMP280_PRESSURE_REG_H] << 16 | \
                      data[BMP280_PRESSURE_REG_M] << 8 | \
                      data[BMP280_PRESSURE_REG_L];
  uint32_t motion = data[HUMAN_DETECT];


  PUBLISH_DATA("sensor", "temp1", temp1);
  PUBLISH_DATA("sensor", "temp2", temp2);
  PUBLISH_DATA("sensor", "temp_ext", temp3);
  PUBLISH_DATA("sensor", "light", light);
  PUBLISH_DATA("sensor", "humidity", humidity);
  PUBLISH_DATA("sensor", "pressure", pressure);
  PUBLISH_DATA("binary_sensor", "temp2", motion);

#if DEBUG
  Serial.print("Temp1: ");
  Serial.print(temp1);
  Serial.println("*c");

  Serial.print("Temp2: ");
  Serial.print(temp2);
  Serial.println("*c");

  Serial.print("Temp3: ");
  Serial.print(temp3);
  Serial.println("*c");

  Serial.print("Light: ");
  Serial.print(light);
  Serial.println(" lux");

  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println("%");

  Serial.print("Pressure: ");
  Serial.print(pressure);
  Serial.println("mPa");

  Serial.print("Motion: ");
  Serial.println(motion);

  PrintHex8(data, DATA_LEN + 1);
#endif

  delay(5000);
}
