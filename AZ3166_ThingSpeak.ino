/*
Copyright (C) 2021  Lubos Kuzma

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/


#include <AZ3166WiFi.h>
#include "RGB_LED.h"
#include "EEPROMInterface.h"
#include "MQTTClient.h"
#include "MQTTNetwork.h"
#include "HTS221Sensor.h"
#include "LPS22HBSensor.h"

RGB_LED rgbled;                 // RGB Led used for special functions
EEPROMInterface secure_mem;     // Secure Memory holds SSID and password 

// WiFi buffers (from Secure Mem)
char ssid_buf[64];
char pswd_buf[64];
int wifi_status = WL_IDLE_STATUS;
bool hasWifi = false;

char PRINT_BUFFER[20];          // OLED screen buffer
char dat_buf[64];
int mem_read_res;
int arrivedcount = 0;

// measurement variables
float temperature;
float humidity;
float pressure;
float LPS_temp;

HTS221Sensor *HT_sensor;
LPS22HBSensor *LPS_sensor;
DevI2C *i2c;


void setup() {
  Screen.clean();
  rgbled.setColor(0,0,0);
  rgbled.turnOff();
  Serial.begin(115200);

  // Setup LEDs
  pinMode(LED_WIFI, OUTPUT);    // WiFi status
  pinMode(LED_AZURE, OUTPUT);   // IoT Portal connection status
  pinMode(LED_USER, OUTPUT);    // Connecyed to MQTT Server status

  //Humidity Temp sensor
  i2c = new DevI2C(D14, D15);   // D14 and D15 are I2C GPIOs
  HT_sensor = new HTS221Sensor(*i2c);
  HT_sensor -> init(NULL);

  // LPS Sensor
  LPS_sensor = new LPS22HBSensor(*i2c);
  LPS_sensor -> init(NULL);

  // Connect to WiFi
  init_Wifi();
}

// WIFI Connection

void init_Wifi() {
    // attempt to connect to Wifi network:
    while (wifi_status != WL_CONNECTED) {
        // Pull WiFi credential from Secure IC
        get_Wifi_credentials();
        Serial.print("Attempting to connect to WPA SSID: ");
        Serial.println(ssid_buf);
        // Connect to WPA/WPA2 network:
        wifi_status = WiFi.begin(ssid_buf, pswd_buf);
        // Wait 10 seconds for connection:
        delay(10000);
    } 
    
    if (wifi_status != WL_CONNECTED) {
        // Turn off Wifi LED
        digitalWrite(LED_WIFI, LOW);
        hasWifi = false;
           
    } else {
        sprintf(PRINT_BUFFER, "WiFi: %s", ssid_buf);
        Screen.print(0, PRINT_BUFFER);
        Serial.println("Connected to Wifi");
        digitalWrite(LED_WIFI, HIGH);
        hasWifi = true;
        // Only connect MQTT Network if Wifi is on
        connect_mqtt_network();
    }

}

void get_Wifi_credentials() {
  // Pull SSID and Pass from secure IC
  mem_read_res = secure_mem.read((uint8_t*)ssid_buf, 64, 0x00, STSAFE_ZONE_3_IDX);
     
  if (mem_read_res == -1) {
      // if memory read was unsuccesful
      Screen.print(1, "Can't read WiFi SSID");
      Serial.println("Can't read SSID from Secure Memory");
      digitalWrite(LED_WIFI, LOW);
  }

  mem_read_res = secure_mem.read((uint8_t*)pswd_buf, 64, 0x00, STSAFE_ZONE_10_IDX);
  
  if (mem_read_res == -1) {
      // if memory read was unsuccesful
      Screen.print(1, "Can't read WiFi PSWD");
      Serial.println("Can't read WiFi Password from Secure Memory");
      digitalWrite(LED_WIFI, LOW);
  }


}

// MQTT Server on ThingSpeak

const char* mqttServer = "mqtt3.thingspeak.com";
int port = 1883;
const char* topic = "channels/1555948/publish";
char RCmsgBuf[100];               // MQTT Return Code Message buffer
bool mqtt_connected = false;
bool last_publish_failed = true;

MQTTNetwork mqttNetwork;
MQTT::Client<MQTTNetwork, Countdown> mqtt_client = MQTT::Client<MQTTNetwork, Countdown>(mqttNetwork);

int connect_mqtt_network() {
  
  sprintf(RCmsgBuf, "Connecting to MQTT server %s:%d", mqttServer, port);
  Serial.println(RCmsgBuf);

  int rc = mqttNetwork.connect(mqttServer, port);
  if (rc != 0) {
    Serial.println("Connection to MQTT server failed");
    digitalWrite(LED_AZURE, LOW);
    sprintf(RCmsgBuf, "Return Code: %d", rc);
    Serial.println(RCmsgBuf);

  } else {
    Serial.println("Connected to MQTT server");
    digitalWrite(LED_AZURE, HIGH);
  }
  
  // ThingSpeak Device settings:
  // Client ID - LwgJMyg7FxA3BTYFAy46Iyk
  // Username - LwgJMyg7FxA3BTYFAy46Iyk
  // Password - SjyOt4NoGJCMDVN1gzw+fTQ1

  MQTTPacket_connectData mqtt_data = MQTTPacket_connectData_initializer;
  mqtt_data.MQTTVersion = 4;
  mqtt_data.clientID.cstring = (char*)"LwgJMyg7FxA3BTYFAy46Iyk";
  mqtt_data.username.cstring = (char*)"LwgJMyg7FxA3BTYFAy46Iyk";
  mqtt_data.password.cstring = (char*)"SjyOt4NoGJCMDVN1gzw+fTQ1";
  mqtt_data.cleansession = true;
  mqtt_data.willFlag = 1;

  rc = mqtt_client.connect(mqtt_data);
  if (rc != 0) {
    Serial.println("MQTT client connection failed");
    sprintf(RCmsgBuf, "Retrun code: %d", rc);
    Serial.println(RCmsgBuf);
    mqtt_connected = false;
  } else {
    digitalWrite(LED_USER, HIGH);
    mqtt_connected = true;
  }
  
}

int disconnect_mqtt_network() {
  int rc = mqtt_client.disconnect();
  if (rc != 0) {
      Serial.println("MQTT client disconnect from server failed");
      sprintf(RCmsgBuf, "Retrun Code: %d", rc);
      Serial.println(RCmsgBuf);
  } else {
      digitalWrite(LED_USER, LOW);
      mqtt_connected = false;
  }
}


int publish_mqtt(char* f1_payload, char* f2_payload, char* f3_payload) {

  int rc;
  MQTT::Message message;

  // assemble message
  Serial.print("Publish function - ");
  char msg_buf[100];
  sprintf(msg_buf, "field1=%s&field2=%s&field3=%s", f1_payload, f2_payload, f3_payload);
  Serial.println(msg_buf);
  message.qos = MQTT::QOS0;
  message.retained = false;
  message.dup = false;
  message.payload = (void*)msg_buf;
  message.payloadlen = strlen(msg_buf)+1;
  rc = mqtt_client.publish(topic, message);
  
  if (rc != 0) {
    Serial.println("Failed to publish");
    sprintf(RCmsgBuf, "Return Code: %d", rc);
    Serial.println(RCmsgBuf);
    last_publish_failed = true;
  } else {
    Serial.println("Published correctly");
    last_publish_failed = false;
  }
  
}

// HT Sensor

void update_HT_sensor() {

  HT_sensor -> enable();
  HT_sensor -> getTemperature(&temperature);
  delay(1000);
  HT_sensor -> getHumidity(&humidity);
  delay(1000);
  HT_sensor -> disable();
  HT_sensor -> reset();

}

// LPS Sensor

void update_LPS_sensor() {

  LPS_sensor -> getPressure(&pressure);
 
}

void loop() {

  char hum_buf[50];
  char temp_buf[50];
  char press_buf[50];

  update_HT_sensor();
  update_LPS_sensor();

  sprintf(temp_buf, "%.1f", temperature);
  sprintf(hum_buf, "%.1f", humidity);
  sprintf(press_buf, "%.1f", pressure);
  
  
  if (hasWifi){
    publish_mqtt(temp_buf, hum_buf, press_buf);
   } else {
    init_Wifi();
  }
  sprintf(temp_buf, "Temperature: %.0f", temperature);
  sprintf(hum_buf, "Humidity: %.0f", humidity);
  sprintf(press_buf, "Pressure: %.0f", pressure);
  Screen.print(1, temp_buf);
  Screen.print(2, hum_buf);
  Screen.print(3, press_buf);
  Serial.println(temp_buf);
  Serial.println(hum_buf);
  Serial.println(press_buf);

  delay(20000);
  
}
