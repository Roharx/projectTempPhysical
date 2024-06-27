#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <LiquidCrystal_I2C.h>
#include <ArduinoJson.h>

Adafruit_BME280 bme; 
LiquidCrystal_I2C lcd(0x27, 16, 2);
WiFiClient espClient;
PubSubClient client(espClient);

// Inputs/outputs
int blueButtonPin = 26;  
int redButtonPin = 25;   
int greenButtonPin = 16; 
int heaterPin = 17;

// Motor
const int DIR = 12;
const int STEP = 15;
const int ENABLE = 14;
const int steps_per_rev = 200;

// --------Temperature control-------
unsigned long lastPress = 0;
unsigned long debounceDelay = 250;

float targetTemperature = 22.0;
float currentTemperature = 0;
float currentHumidity = 0;
float humidityTreshold = 45;
float humidityMax = 55;
float highHumidityTempAdjuster = 2;

bool windowOpen = false;
const int windowOpenDelay = 10000; // Time in milliseconds to decide if the window should open automatically (10sec for testing and demonstration purposes, normally I'd set it at 30m-1.5hr)
unsigned long timeAboveTargetTemp = 0;

int maxHeatOnTime = 4500;
int minHeatOnTime = 500;
int cycleTime = 5000;
int thresholdTemp = 5;
//--------End of temperature control-----------

// WiFi settings
const char* ssid = "Cookiespot";
const char* password = "KecskeSegg69!";

// MQTT settings
const char* mqttServer = "mqtt.flespi.io";
const int mqttPort = 1883;
const char* mqttClientId = "Temperature";
const char* mqttUsername = "GgRdHT6zYQHZ25mBLwRnACVnw2tfjZlnaDq49L37oKpkG7Pyv1uX5MNcCosNVRiL";
const char* mqttPassword = "";
String officeName = "office1";
String deviceName = "room1";
int deviceId = 1;
char fullTopicName[256];
bool receivedFromServer = false;

// Other variables
volatile bool blueButtonPressed = false;
volatile bool redButtonPressed = false;
volatile bool greenButtonPressed = false;
unsigned long lastInputTime = 0; 
bool pendingUpdate = false;
bool toggleWindowOpener = true;

// Non-blocking delay variables
unsigned long lastMotorStepTime = 0;
const unsigned long motorStepInterval = 2000; // Microseconds

void setup() {
  Serial.begin(115200); 
  pinMode(blueButtonPin, INPUT_PULLUP);
  pinMode(redButtonPin, INPUT_PULLUP);
  pinMode(greenButtonPin, INPUT_PULLUP);
  pinMode(DIR, OUTPUT);
  pinMode(STEP, OUTPUT);
  pinMode(ENABLE, OUTPUT);

  pinMode(heaterPin, OUTPUT);

  lcd.init();                      
  lcd.backlight();                 

  if (!bme.begin(0x76)) {          
    lcd.print("No BME280 detected");
    while (1);
  }

  attachInterrupt(digitalPinToInterrupt(blueButtonPin), bluePressed, FALLING);
  attachInterrupt(digitalPinToInterrupt(redButtonPin), redPressed, FALLING);
  attachInterrupt(digitalPinToInterrupt(greenButtonPin), greenPressed, FALLING);

  setFullTopicName();

  // WiFi
  connectWiFi();
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

  reconnect();
}

void loop() {  
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  currentTemperature = bme.readTemperature();
  currentHumidity = bme.readHumidity();
  updateScreen();
  temperatureAction();
  if(windowOpen){
    digitalWrite(heaterPin, LOW);
  }

  handleButtons();

  delay(50); // Adjust loop delay as needed
}

#pragma region WiFi
void connectWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
}
#pragma endregion

#pragma region MQTT
void reconnect() {
  while (!client.connected()) {
    Serial.prwint("Attempting MQTT connection...");
    if (client.connect(mqttClientId, mqttUsername, mqttPassword)) {
      Serial.println("Connected to MQTT");
      subscribeToTopics();
      sendConnectedMessage();
      sendStartupMessage();
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" trying again in 5 seconds");
      delay(5000);
    }
  }
}

void sendInfo() {
  if (!receivedFromServer) {
    String jsonPayload = "{\"source\": \"" + "device" +
                         "\", \"targetTemperature\": " + String(targetTemperature) +
                         ", \"humidityTreshold\": " + String(humidityTreshold) +
                         ", \"humidityMax\": " + String(humidityMax) +
                         ", \"toggle\": " + String(toggleWindowOpener) + "}";
    if (client.publish(fullTopicName, jsonPayload.c_str())) {
      Serial.println("Data sent to MQTT broker from " + String(deviceName));
    } else {
      Serial.println("Failed to send data from " + String(deviceName));
    }
  } else {
    receivedFromServer = false;
  }
}



void subscribeToTopics() {
  String specificConfigTopic = "config/" + officeName + "/" + deviceName;
  String tempTopic = "temp/" + officeName + "/" + deviceName;
  
  // Subscribe to both specific and wildcard config topics
  client.subscribe(specificConfigTopic.c_str());
  client.subscribe("config/#");
  client.subscribe(tempTopic.c_str());

  Serial.println("Subscribed to specific config topic: " + specificConfigTopic);
  Serial.println("Subscribed to wildcard config topic: config/#");
  Serial.println("Subscribed to temp topic: " + tempTopic);
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  
  String messageTemp;
  for (int i = 0; i < length; i++) {
    messageTemp += (char)payload[i];
  }
  Serial.println(messageTemp);

  String specificConfigTopic = "config/" + officeName + "/" + deviceName;
  String tempTopic = "temp/" + officeName + "/" + deviceName;

  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, messageTemp);

  if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.f_str());
    return;
  }

  if (String(topic).startsWith("config/")) {
    if (doc["message"] == "response" && doc["device"] == deviceId) {
      Serial.println("Handling config response...");
      handleConfigResponse(doc);
    } else if (String(topic) == specificConfigTopic) {
      Serial.println("Handling specific config update...");
      handleConfigUpdate(doc);
    }
  } 
  else if (String(topic) == tempTopic) {
    Serial.println("Handling temp update...");
    handleTempUpdate(doc);
  }
}

void handleConfigUpdate(DynamicJsonDocument& doc) {
  String newOfficeName = doc["office"].as<String>();
  String newRoomName = doc["room"].as<String>();

  if (newOfficeName.length() > 0 && newRoomName.length() > 0) {
      officeName = newOfficeName;
      deviceName = newRoomName;
      setFullTopicName();
      subscribeToSpecificTopics();
      Serial.println("Config updated: Office=" + officeName + ", Room=" + deviceName);
  }
}

void handleConfigResponse(DynamicJsonDocument& doc) {
  if (doc["device"] == deviceId) {
    officeName = doc["office"].as<String>();
    deviceName = doc["room"].as<String>();
    targetTemperature = doc["targetTemperature"].as<float>();
    humidityTreshold = doc["humidityTreshold"].as<float>();
    humidityMax = doc["humidityMax"].as<float>();
    toggleWindowOpener = doc["toggle"].as<bool>();

    setFullTopicName();
    subscribeToSpecificTopics();
    unsubscribeFromWildcardConfig();

    Serial.println("Config response applied: ");
    Serial.println("Office: " + officeName);
    Serial.println("Room: " + deviceName);
    Serial.println("Target Temperature: " + String(targetTemperature));
    Serial.println("Humidity Threshold: " + String(humidityTreshold));
    Serial.println("Humidity Max: " + String(humidityMax));
    Serial.println("Toggle Window Opener: " + String(toggleWindowOpener ? "Automatic" : "Manual"));
  } else {
    Serial.println("Config response ignored: device ID does not match.");
  }
}

void handleTempUpdate(DynamicJsonDocument& doc) {
  if (doc.containsKey("targetTemperature")) {
    targetTemperature = doc["targetTemperature"].as<float>();
    Serial.print("New target temperature received: ");
    Serial.println(targetTemperature);
  }
  if (doc.containsKey("toggle")) {
    toggleWindowOpener = doc["toggle"].as<bool>();
    Serial.print("Window opener mode updated to: ");
    Serial.println(toggleWindowOpener ? "Automatic" : "Manual");
    handleMotorEnable(); // Ensure motor enable state is updated immediately
  }
  if (doc.containsKey("humidityTreshold")) {
    humidityTreshold = doc["humidityTreshold"].as<float>();
    Serial.print("Humidity threshold updated to: ");
    Serial.println(humidityTreshold);
  }
  if (doc.containsKey("humidityMax")) {
    humidityMax = doc["humidityMax"].as<float>();
    Serial.print("Maximum humidity updated to: ");
    Serial.println(humidityMax);
  }
}

void subscribeToSpecificTopics() {
  String specificConfigTopic = "config/" + officeName + "/" + deviceName;
  client.subscribe(specificConfigTopic.c_str());
  String tempTopic = "temp/" + officeName + "/" + deviceName;
  client.subscribe(tempTopic.c_str());

  Serial.println("Subscribed to specific config topic: " + specificConfigTopic);
  Serial.println("Subscribed to temp topic: " + tempTopic);
}

void unsubscribeFromWildcardConfig() {
  client.unsubscribe("config/#");
  Serial.println("Unsubscribed from wildcard config topic: config/#");
}

void sendConnectedMessage() {
  String connectMessage = "{\"deviceId\": " + String(deviceId) + ", \"message\": \"connected\"}";
  String connectTopic = "status/" + officeName + "/" + deviceName;

  if (client.publish(connectTopic.c_str(), connectMessage.c_str())) {
      Serial.println("Connected message sent from " + String(deviceName));
  } else {
      Serial.println("Failed to send connected message from " + String(deviceName));
  }
}

void sendStartupMessage() {
  String startupMessage = "{\"message\": \"startup\", \"device\": " + String(deviceId) + "}";
  String configTopic = "config/";

  if (client.publish(configTopic.c_str(), startupMessage.c_str())) {
      Serial.println("Startup message sent: " + startupMessage);
  } else {
      Serial.println("Failed to send startup message");
  }
}
#pragma endregion

#pragma region Temperature_Control
void temperatureAction(){
  if (toggleWindowOpener) {
    if (currentTemperature > targetTemperature + 5 - (currentHumidity > humidityTreshold ? highHumidityTempAdjuster : 0) || currentHumidity > humidityMax) {
      openWindow(3, 500); // Open window fast if temp is 5 degrees above target
      windowOpen = true;
    } else if (currentTemperature > targetTemperature - (currentHumidity > humidityTreshold ? highHumidityTempAdjuster : 0)) {
      if (millis() - timeAboveTargetTemp > windowOpenDelay) {
        openWindow(3, 1000); // Open window slowly if temp is constantly above target
      }
    } else {
      timeAboveTargetTemp = millis(); // Reset timer as temperature is not above target      
      closeWindow(3, 1000); // Close window if the temperature is normal or below
    }
  }

  if (!windowOpen) { // Ensure heating is disabled when window is open
    heatRoom();
  }

  // Check if it's time to send the update
  if (pendingUpdate && millis() - lastInputTime > 5000) {
    sendInfo();
    pendingUpdate = false; // Reset the update flag
  }
}

void openWindow(int amount, int speed) {
  if(!windowOpen){
    Serial.println("Opening window...");
    digitalWrite(DIR, HIGH);
    for (int i = 0; i < steps_per_rev * 3; i++) {  // Open window 3 revolutions      
      digitalWrite(STEP, HIGH);
      delayMicroseconds(1000);  // Adjust delay as needed
      digitalWrite(STEP, LOW);
      delayMicroseconds(1000);  // Adjust delay as needed
    }
    windowOpen = true;
  }
}

void closeWindow(int amount, int speed) {
  if(windowOpen){
    Serial.println("Closing window...");
    digitalWrite(DIR, LOW);
    for (int i = 0; i < steps_per_rev * 3; i++) {  // Close window 3 revolutions
      digitalWrite(STEP, HIGH);
      delayMicroseconds(1000);  // Adjust delay as needed
      digitalWrite(STEP, LOW);
      delayMicroseconds(1000);  // Adjust delay as needed
    }
    windowOpen = false;
  }  
}

void heatRoom() {
  if (windowOpen == false){  
    unsigned long currentTime = millis();
    static unsigned long lastHeatCycleTime = 0;
    static bool heaterState = false;

    float tempDifference = targetTemperature - (currentHumidity > humidityTreshold ? highHumidityTempAdjuster : 0) - currentTemperature;

    if (tempDifference > thresholdTemp) { // if 5 degrees below the target, we continuously heat
      digitalWrite(heaterPin, HIGH);
      heaterState = true;
    } else if (tempDifference > 0) { //
      int heatOnTime = (int)map(tempDifference * 100, 0, thresholdTemp * 100, 0, cycleTime * 0.9);
      heatOnTime = constrain(heatOnTime, 0, cycleTime * 0.9);

      if (currentTime - lastHeatCycleTime >= cycleTime) {
        lastHeatCycleTime = currentTime;
        heaterState = false;
      }

      if (!heaterState && currentTime - lastHeatCycleTime < heatOnTime) {
        digitalWrite(heaterPin, HIGH);
        heaterState = true;
      } else if (heaterState && currentTime - lastHeatCycleTime >= heatOnTime) {
        digitalWrite(heaterPin, LOW);
        heaterState = false;
      }
  } else {
    digitalWrite(heaterPin, LOW);
    heaterState = false;
  }
  }
}

void updateScreen(){
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(currentTemperature);
  lcd.print(" C");
  lcd.setCursor(0, 1);
  lcd.print("Target: ");
  lcd.print(targetTemperature);
  lcd.print(" C");
  lcd.setCursor(15, 0);
  lcd.print(toggleWindowOpener ? "A" : "M");
}

void setFullTopicName() {
  fullTopicName[0] = '\0';
  strcpy(fullTopicName, "temp/");
  strcat(fullTopicName, officeName.c_str());
  strcat(fullTopicName, "/");
  strcat(fullTopicName, deviceName.c_str());
}
#pragma endregion

#pragma region Button_Handlers
void bluePressed() {
  blueButtonPressed = true;
}

void redPressed() {
  redButtonPressed = true;
}

void greenPressed() {
  greenButtonPressed = true;
}

void handleButtons() {
  if (blueButtonPressed) {
    blueButtonPressed = false;
    unsigned long currentTime = millis();
    if (currentTime - lastPress > debounceDelay) {
      lastPress = currentTime;
      targetTemperature--;
      pendingUpdate = true;
      lastInputTime = millis();
      receivedFromServer = false;
    }
  }

  if (redButtonPressed) {
    redButtonPressed = false;
    unsigned long currentTime = millis();
    if (currentTime - lastPress > debounceDelay) {
      lastPress = currentTime;
      targetTemperature++;
      pendingUpdate = true;
      lastInputTime = millis();
      receivedFromServer = false;
    }
  }

  if (greenButtonPressed) {
    greenButtonPressed = false;
    unsigned long currentTime = millis();
    if (currentTime - lastPress > debounceDelay) {
      lastPress = currentTime;
      toggleWindowOpener = !toggleWindowOpener;
      pendingUpdate = true;
      lastInputTime = millis();
      receivedFromServer = false;
      Serial.print("Window opener toggled to: ");
      Serial.println(toggleWindowOpener ? "Automatic" : "Manual");
      handleMotorEnable();
    }
  }
}

void handleMotorEnable() {
  if (toggleWindowOpener) {
    digitalWrite(ENABLE, LOW); // Enable the motor
  } else {
    digitalWrite(ENABLE, HIGH); // Disable the motor
  }
}
#pragma endregion
