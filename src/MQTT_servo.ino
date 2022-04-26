#include <Arduino.h>
#ifdef ESP32
#include <WiFi.h>
#include <AsyncTCP.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Servo.h>
#include "max6675.h"
#include <Wire.h>               // Only needed for Arduino 1.6.5 and earlier
#include "SSD1306Wire.h"        // legacy: #include "SSD1306.h"
#include <AutoPID.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <ESP8266WiFiMulti.h>

#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>

#endif
#include <ESPAsyncWebServer.h>

#include "RestAPI.h";


StaticJsonDocument<200> doc;

// Thermocouple 
int thermoDO = D8;
int thermoCS = D7;
int thermo2CS = D0;
int thermoCLK = D6;
MAX6675 thermocouple1(thermoCLK, thermoCS, thermoDO);
MAX6675 thermocouple2(thermoCLK, thermo2CS, thermoDO);
#define J_TYPE_thermo

// Servo
Servo myServo;  // create a servo object
int servo_pin = D5;
int current_pos = 0;
#define valve_off 180
#define valve_max 90
#define valve_min 0

// OLED
int SDA_pin = D2;
int SCL_pin = D1;
SSD1306Wire display(0x3c, SDA_pin, SCL_pin);   // ADDRESS, SDA, SCL  -  SDA and SCL usually populate automatically based on your board's pins_arduino.h e.g. https://github.com/esp8266/Arduino/blob/master/variants/nodemcu/pins_arduino.h

// buttons
// buttons are connected to analog input A0 using a resistor ladder.
// BTN_1 voltage = 0V (GND)
// BTN_2 voltage = 1.65V (R-R)
// BTN_3 voltage = 2.2V (R-2R)
// no buttons - 3.3V
int BUTTON_pin = A0;
int button = 0;

// WiFi
const char *ssid1 = "********"; // Enter your WiFi name
const char *pwd1 = "********";  // Enter WiFi password
const char *ssid2 = "********"; // Enter your WiFi name
const char *pwd2 = "********";  // Enter WiFi password
const char *ssid3 = "********"; // Enter your WiFi name
const char *pwd3 = "********";  // Enter WiFi password
WiFiClient espClient;
ESP8266WiFiMulti wifiMulti;
unsigned int connect_retries = 0;

// Time keeping
long last_millis=0;
long last_state_change_millis=0;
long last_reconnect_millis=0;
const long wifi_reconnect_interval_ms = 20000;
char str[100];

// MQTT Broker
const char *mqtt_broker = "10.0.0.10";
const char *topic = "smoker";
const char *temp_topic = "smoker/temp";     // used for publishing temp and setpoint
const char *set_topic = "smoker/setpoint";  //set goal temp
const char *pid_topic = "smoker/pid";       // set PID coefficients
const char *servo_topic = "smoker/servo";   // direct control of servo command 0-255
const char *valve_topic = "smoker/valve";   // direct control of valve pos off/min/max
const char *inj_topic = "smoker/inj";       // inject temp value for testing
const char *ctrl_topic = "smoker/control";  // control loop on/off

const char *mqtt_username = "STRSO";
const char *mqtt_password = "1011040311037";
const int mqtt_port = 1883;
PubSubClient client(espClient);

//AutoPID
float Kp = 50;
float Ki = 2;
float Kd = 10;
unsigned long lastTempUpdate; //tracks clock time of last temp update
double setpoint_temp, current_temp, current_temp2, valve_cmd;
bool control_active = false;
AutoPID myPID(&current_temp, &setpoint_temp, &valve_cmd, 0, 255, Kp, Ki, Kd);

unsigned int UI_state = 0;  //control by buttons. 0 - idle, 1 - change setpoint, 2- set control on/off  
int rc;
bool display_IP = false;

void setup() {

  ArduinoOTA.onStart([]() {
    String type;
    display.clear();
    display.drawString(0,0,"Firmware update");
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
    display.drawString(0, 20, "Start updating " + type);
    display.display();
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    display.clear();
    display.drawString(0,0,"Firmware update");
    sprintf(str, "Progress: %u%%\r", (progress / (total / 100)));
    display.drawString(0, 20, str);
    display.display();

  });
  ArduinoOTA.onError([](ota_error_t error) {
    display.clear();
    display.drawString(0,0,"Firmware update");
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
      sprintf(str, "Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
      sprintf(str, "Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
      sprintf(str, "Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
      sprintf(str, "Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
      sprintf(str, "End Failed");
    }

    display.drawString(0, 20, str);
    display.display();
  });

  // init multi Wifi
    WiFi.mode(WIFI_STA);
    wifiMulti.addAP(ssid1, pwd1);
    wifiMulti.addAP(ssid2, pwd2);
    wifiMulti.addAP(ssid3, pwd3);

  // init servo
  myServo.attach(servo_pin); // attaches the servo on pin 7 to the servo object
  myServo.write(valve_off);
  
  // init OLED display
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_24);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  sprintf(str, "Smoker \n Controller");
  display.drawString(5, 0, str);
  display.display();
  delay(2000);

  //initialize the variables we're linked to
  current_temp = 25;  // replace with real thermocouple data
  setpoint_temp = 35.5;

  // Set software serial baud to 115200;
  Serial.begin(115200);
  delay(1000);

  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.clear();

  // connecting to a WiFi network
  rc = connect_wifi(true);
  //rc = connect_mqtt(true);

  //if temperature is more than 4 degrees below or above setpoint, OUTPUT will be set to min or max respectively
  myPID.setBangBang(4);
  //set PID update interval to 2000ms
  myPID.setTimeStep(2000);

  Serial.println("Starting Server.."); 
  setupServerEvents();
}

int connect_wifi(bool force_connect)
{
  int retval = 0;
  if ((millis()-last_reconnect_millis>wifi_reconnect_interval_ms)||force_connect)
  {
    last_reconnect_millis = millis();
    connect_retries = 0;
    display.clear();
    // WiFi.begin(ssid, password);
    // while (WiFi.status() != WL_CONNECTED && connect_retries<=10) {
    while (wifiMulti.run() != WL_CONNECTED && connect_retries<=10) 
    {
        delay(500);
        connect_retries++;
        Serial.println("Connecting to WiFi..");
        display.drawString(5, 0, "Connecting to \nWiFi");
        display.display();
    }
    if(!WiFi.isConnected())
    {
      Serial.println("Wifi connection failed");
      display.clear();
      display.drawString(0, 0, "No connection");
      display.display();
      delay(3000);
      retval = -1;
    }
    else
    {
      Serial.println("Connected to the WiFi network");
      display.clear();  
      display.drawString(5, 0, "Connected!");
      display.drawString(5, 16, WiFi.localIP().toString());
      display.display();

      ArduinoOTA.begin();
      Serial.println("Ready");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
      delay(3000);
    }
  }
  return retval;

}

int connect_mqtt(bool force_connect)
{
  int retval = 0;
  
      //connecting to a mqtt broker
    client.setServer(mqtt_broker, mqtt_port);
    client.setCallback(callback);
    while (!client.connected()) 
    {
        String client_id = "esp8266-client-";
        client_id += String(WiFi.macAddress());
        Serial.printf("The client %s connects to the public mqtt broker\n", client_id.c_str());
        if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
            Serial.println("HA mqtt broker connected");
            display.clear();
            display.drawString(5, 0, "MQTT Connected!");
            display.display();
            delay(1000);
        } else {
            Serial.print("failed with state ");
            Serial.print(client.state());
            delay(2000);
        }
      
      // publish and subscribe
      client.subscribe(set_topic);
      client.subscribe(pid_topic);
      client.subscribe(servo_topic);
      client.subscribe(valve_topic);
    //  client.subscribe(inj_topic);
      client.subscribe(ctrl_topic);

      retval = 0;
    }
  return retval;
}

void callback(char *topic, byte *payload, unsigned int length) {
    String payload_str = String("");
    for (int i = 0; i < length; i++) {
      payload_str += (char) payload[i];
    }

  if (strcmp(topic, set_topic) == 0){
    setpoint_temp = payload_str.toFloat();
    Serial.print("Got new setpoint: ");
    Serial.println(setpoint_temp);
  }
  if (strcmp(topic, servo_topic) == 0){
    int servo_pos = payload_str.toInt();
    Serial.print("Got servo pos: ");
    Serial.println(servo_pos);
    int servo_cmd = map(servo_pos,0,255,valve_min,valve_max);
    Serial.print("servo angle:");
    Serial.println(servo_cmd);
    myServo.attach(servo_pin);
    myServo.write(servo_cmd);
    
  }
  else if (strcmp(topic, ctrl_topic) == 0){
    Serial.print("Control is ");
    if (payload_str == "on"){
      Serial.println("on");
      control_active = true;
    }
    else if (payload_str == "off"){
      Serial.println("off");
      myPID.stop();
      control_active = false;
      myServo.write(valve_off);
      delay(500);
      myServo.detach();
    }
  }
  else if (strcmp(topic, valve_topic) == 0){
    Serial.print("Valve set to ");
    if (payload_str == "off"){
      Serial.println(payload_str);
      myServo.write(valve_off);
      delay(500);
      myServo.detach();
    }
    else if (payload_str == "min"){
      myServo.attach(servo_pin);
      Serial.println(payload_str);
      myServo.write(valve_min);
    }
    else if (payload_str == "max"){
      myServo.attach(servo_pin);
      Serial.println(payload_str);
      myServo.write(valve_max);
    }
  }
  else if (strcmp(topic, pid_topic) == 0){
    int val1_ind = payload_str.indexOf(',',0);
    int val2_ind = payload_str.indexOf(',',val1_ind + 1);
    Kp = payload_str.substring(1,val1_ind).toFloat(); 
    Ki = payload_str.substring(val1_ind +1, val2_ind).toFloat(); 
    Kd = payload_str.substring(val2_ind + 1).toFloat(); 
    String temp_str = String("Got new PID values: Kp=" + String(Kp) + ", Ki=" + String(Ki) +", Kd=" + String(Kd));
    Serial.println(temp_str);
    myPID.setGains(Kp,Ki,Kd);
  }
  
 }

float convert_J_K_thermo(float K_reading){
  float a0 = -1.93;
  float a1 = 1.59;
  float a2 = -1.66e-3;
  float a3 = 1.98e-6;

  return a0 + K_reading*a1 + pow(K_reading,2)*a2 + pow(K_reading,3)*a3;
}

void valve_control() {
  int servo_cmd = map(constrain(valve_cmd,0,255),0,255,valve_min,valve_max);
  Serial.print("servo angle:");
  Serial.println(servo_cmd);
  myServo.attach(servo_pin);
  myServo.write(servo_cmd);
}

int button_decoder(){
  int adc_val = analogRead(BUTTON_pin);
  Serial.print("ADC: " + String(adc_val));
  if(adc_val < 50){return 3;}
  else if(adc_val < 580){return 2;}
  else if(adc_val < 800){return 1;}
  else{return 0;}
}

void loop() {
  ArduinoOTA.handle();
  client.loop();
  if (control_active){
    myPID.run();
  }

  // reconnect wifi
  if(!WiFi.isConnected())
  {
    connect_wifi(false);
    //connect_mqtt(false);
  }

  // change state to idle after 10 sec of no activity
  if (millis() - last_state_change_millis >=10000){UI_state = 0;};

  // publish temp every 2 sec
  if (millis() - last_millis >= 2000){
    last_millis = millis();

    // get temp measurement
    #ifdef J_TYPE_thermo
      current_temp = convert_J_K_thermo(thermocouple1.readCelsius());
      current_temp2 = convert_J_K_thermo(thermocouple2.readCelsius());
    #else
      current_temp = thermocouple.readCelsius();
      current_temp2 = thermocouple2.readCelsius();
    #endif

    // JSON generator
    doc["temp1"] = current_temp;
    doc["temp2"] = current_temp2;
    doc["setpoint"] = setpoint_temp;
    doc["active"] = control_active;

    // Generate the minified JSON and send it to the str variable.
    serializeJson(doc, str);
    client.publish(temp_topic, str);  
    Serial.println(str);
    
    if (control_active){
      valve_control();
    }
  }

  // handle buttons press
  button = button_decoder();
  switch (button){
    case 0:  //no button pressed
      display_IP = false;
      break;
    case 1:  //button 1 pressed (Left button) - change state
      UI_state+=1;
      UI_state%=3;
      if (UI_state!=0){last_state_change_millis=millis();};
      break;
    case 2: //DOWN
      last_state_change_millis = millis();
      if(UI_state==1) //modify setpoint
      {
        setpoint_temp-=5;
        if (setpoint_temp<30){setpoint_temp=30;}
      }
      else if(UI_state==2) //set control ON/OFF
      {
        control_active=0;
      }
      break;
    case 3:  //UP
      last_state_change_millis = millis();
      if(UI_state==1)  //modify setpoint
      {
        setpoint_temp+=5;
        if (setpoint_temp>250){setpoint_temp=250;}
      }
      else if(UI_state==2) //set control ON/OFF
      {
        control_active=1;
      }
      else  //show IP
      {
          display_IP = true;
      }
      break;
    default:
      break;
  }

  // display update
    display.clear();
    display.setFont(ArialMT_Plain_16);
    display.setTextAlignment(TEXT_ALIGN_LEFT);

    if (display_IP)
    {
       display.drawString(0, 0, "WiFi settings:");
       display.drawString(0, 20, "SSID: " + WiFi.SSID());
       display.drawString(0, 40, "IP: " +  WiFi.localIP().toString());
    }
    else  //display controller state (temp, setpoint, active)
    {
      if(control_active){
        sprintf(str, "SP: %3.1f (ON)", setpoint_temp);
      }
      else{
        sprintf(str, "SP: %3.1f (OFF)", setpoint_temp);
      }
      display.drawString(0, 0, str);
      sprintf(str, "Temp1:  %3.1f C", current_temp);
      display.drawString(0, 20, str);
      sprintf(str, "Temp2:  %3.1f C", current_temp2);
//      sprintf(str, "ADC:  %d", analogRead(BUTTON_pin));
      display.drawString(0, 40, str);

      switch(UI_state){
        case 1:
          display.drawRect(0,0,22,17);
          break;
        case 2:
          display.drawRect(65,0,40,17);
          break;
      }
    }

    // write the buffer to the display
    display.display();
  

  delay(200);
}
