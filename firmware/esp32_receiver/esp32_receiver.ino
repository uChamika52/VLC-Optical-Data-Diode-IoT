#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <PubSubClient.h>

#define SENSOR_PIN 35
#define THRESHOLD 1350

Adafruit_SSD1306 display(128, 64, &Wire, -1);

// WIFI CONFIGURATION
const char* ssid = "FOE_Students";
const char* password = "FOE@30st";

// MQTT BROKER DETAILS
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_topic = "ee3363/vlc/sensordata"; 
const char* mqtt_client_id = "ee3363-vlc-receiver"; 

WiFiClient espClient;
PubSubClient client(espClient);

// MQTT Connection
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(mqtt_client_id)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  analogSetAttenuation(ADC_11db);

  // OLED Display Initialization
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println("VLC Receiver Booting...");
  display.display();
  delay(1000);

  // WIFI Connection
  display.clearDisplay();
  display.setCursor(0,0);
  display.printf("Connecting to %s\n", ssid);
  display.display();
  Serial.printf("Connecting to %s ", ssid);
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println(" WiFi Connected!");
  display.println("WiFi Connected!");
  display.display();
  delay(1000);

  client.setServer(mqtt_server, mqtt_port);
  
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("READY: WAITING SYNC");
  display.display();
}

uint8_t readPPMByte() {
  uint8_t result = 0;
  for (int i = 7; i >= 0; i--) {
    uint32_t timeout = millis();
    while(analogRead(SENSOR_PIN) < THRESHOLD && (millis() - timeout < 500));
    
    uint32_t start = millis();
    while(analogRead(SENSOR_PIN) > THRESHOLD && (millis() - start < 100));
    uint32_t duration = millis() - start;
    
    if (duration > 10) result |= (1 << i);
  }
  return result;
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  if (analogRead(SENSOR_PIN) > THRESHOLD) {
    uint32_t hStart = millis();
    while(analogRead(SENSOR_PIN) > THRESHOLD && (millis() - hStart < 200));
    uint32_t hDuration = millis() - hStart;

    if (hDuration > 40 && hDuration < 80) {
      delay(20);
      
      uint8_t humi = readPPMByte();
      uint8_t temp = readPPMByte();

      if (temp > 0 && temp < 80) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(0,0);
        display.println("VLC REAL-TIME DATA");
        display.drawFastHLine(0, 10, 128, WHITE);
        display.setTextSize(2);
        display.setCursor(0, 20);
        display.printf("TEMP: %d C", temp);
        display.setCursor(0, 45);
        display.printf("HUMI: %d %%", humi);
        display.display();
        
        Serial.printf("T:%d H:%d\n", temp, humi);

        char json_payload[100];
        snprintf(json_payload, sizeof(json_payload), "{\"temperature\":%d, \"humidity\":%d}", temp, humi);

        Serial.print("Publishing message to topic: ");
        Serial.println(mqtt_topic);
        Serial.println(json_payload);
        
        client.publish(mqtt_topic, json_payload);
        
        delay(2000); 
      }
    }
  }
}