#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// MQTT variables
const char* ssid = "Monster 2.4G";
const char* password = "startup1";
const char* mqtt_server = "192.168.0.115";  // Replace with Raspberry Pi IP address
const int mqtt_port = 1883;
const char* command_topic = "robot/commands";
bool mqttEnabled = true; // Keep track of MQTT status

WiFiClient espClient;
PubSubClient client(espClient);

// Pin assignments
const int EmoteButton = 2;  // Yellow button
const int PayloadDrop = 12; // Green button
const int GaitButton = 4;   // Blue button
const int ModeButton = 5;   // Red button

const int MoveHexaX = 32;   // Analog input (valid)
const int MoveHexaY = 33;   // Analog input (valid)
const int MoveSwitch = 14;  // Digital input (valid)

const int LookHexaX = 26;   // Analog input (valid)
const int LookHexaY = 25;   // Analog input (valid)
const int LookSwitch = 27;  // Digital input (valid)

const int KillSwitch = 13;  // Kill switch pin (you can change this to any available pin)

// Button states
int b1 = 0, b2 = 0, b3 = 0, b4 = 0;
int moveSwitchState = 0, lookSwitchState = 0;
int moveHexaXValue = 0, moveHexaYValue = 0;
int lookHexaXValue = 0, lookHexaYValue = 0;

// Debouncing variables
unsigned long lastDebounceTime[7] = {0, 0, 0, 0, 0, 0, 0};  // Stores last debounce time for each button and kill switch
unsigned long debounceDelay = 50;  // Debounce delay in milliseconds

// Timing variables
unsigned long lastMQTTSendTime = 0;
const int mqttSendInterval = 100; // Send MQTT messages every 100ms

// Deadzone range
const int DEADZONE_MIN = 450;
const int DEADZONE_MAX = 550;

// Function for deadzone logic
int applyDeadzone(int value) {
  if (value > DEADZONE_MAX) {
    return value - DEADZONE_MAX;
  } else if (value < DEADZONE_MIN) {
    return value - DEADZONE_MIN;
  } else {
    return 0;
  }
}

// Button debounce function
int debounceButton(int pin, int index) {
  int reading = digitalRead(pin);
  unsigned long currentTime = millis();

  // If the button state has changed and enough time has passed since the last debounce
  if (reading != (lastDebounceTime[index] > currentTime - debounceDelay)) 
  {
    lastDebounceTime[index] = currentTime;
    return reading;  // Return the new state
  }

  return (lastDebounceTime[index] > currentTime - debounceDelay) ? reading : 0;  // Ignore changes too quick
}

// MQTT setup
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  // Wait for connection with timeout
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    mqttEnabled = true;
  } else {
    Serial.println("");
    Serial.println("WiFi connection failed");
    mqttEnabled = false;
  }
}

void reconnect() {
  // Loop until we're reconnected
  int attempts = 0;
  while (!client.connected() && attempts < 3) { // Limit attempts to avoid blocking too long
    attempts++;
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32Remote-";
    clientId += String(random(0xffff), HEX);
    
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      return;
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" trying again in 1 second");
      delay(1000);
    }
  }
  
  if (!client.connected()) {
    Serial.println("MQTT connection failed after multiple attempts");
  }
}

// not compressed yet
void workingSend() {
  // Check MQTT connection
  if (!client.connected()) {
    reconnect();
    if (!client.connected()) return; // If still not connected, exit
  }
  
  // Create JSON document
  StaticJsonDocument<256> doc;
  
  // Add joystick raw values
  doc["move_x"] = moveHexaXValue;
  doc["move_y"] = moveHexaYValue;
  doc["look_x"] = lookHexaXValue;
  doc["look_y"] = lookHexaYValue;
  
  // Add button states
  doc["move_switch"] = moveSwitchState;
  doc["look_switch"] = lookSwitchState;
  doc["emote_button"] = b1;
  doc["payload_drop"] = b2;
  doc["gait_button"] = b3;
  doc["mode_button"] = b4;
  
  // Add normalized values (0-100) for analog inputs
  if (moveHexaXValue != 0) {
    doc["move_x_percent"] = map(moveHexaXValue + (moveHexaXValue > 0 ? DEADZONE_MAX : DEADZONE_MIN), 
                               0, 4095, 0, 100);
  } else {
    doc["move_x_percent"] = 50; // Center position
  }
  
  if (moveHexaYValue != 0) {
    doc["move_y_percent"] = map(moveHexaYValue + (moveHexaYValue > 0 ? DEADZONE_MAX : DEADZONE_MIN), 
                               0, 4095, 0, 100);
  } else {
    doc["move_y_percent"] = 50; // Center position
  }
  
  if (lookHexaXValue != 0) {
    doc["look_x_percent"] = map(lookHexaXValue + (lookHexaXValue > 0 ? DEADZONE_MAX : DEADZONE_MIN), 
                               0, 4095, 0, 100);
  } else {
    doc["look_x_percent"] = 50; // Center position
  }
  
  if (lookHexaYValue != 0) {
    doc["look_y_percent"] = map(lookHexaYValue + (lookHexaYValue > 0 ? DEADZONE_MAX : DEADZONE_MIN), 
                               0, 4095, 0, 100);
  } else {
    doc["look_y_percent"] = 50; // Center position
  }
  
  // Add timestamp
  doc["timestamp"] = millis();
  
  // Serialize and send
  char jsonBuffer[256];
  serializeJson(doc, jsonBuffer);
  
  // Publish to MQTT topic
  client.publish(command_topic, jsonBuffer);
  
  // For debugging
  Serial.print("MQTT Sent: ");
  Serial.println(jsonBuffer);
}

void sendCommand() {
  // Check MQTT connection
  if (!client.connected()) {
    reconnect();
    if (!client.connected()) return; // If still not connected, exit
  }
  
  // Create a binary buffer instead of JSON
  // Format: [4x2-byte joystick values][1-byte button states][4x1-byte percentages][4-byte timestamp]
  // Total: 17 bytes instead of ~200+ bytes for JSON
  uint8_t buffer[17];
  int index = 0;
  
  // Pack joystick values as 2-byte integers (8 bytes total)
  // Values are typically -3645 to +3645 after deadzone, so 16 bits is sufficient
  buffer[index++] = moveHexaXValue & 0xFF;
  buffer[index++] = (moveHexaXValue >> 8) & 0xFF;
  
  buffer[index++] = moveHexaYValue & 0xFF;
  buffer[index++] = (moveHexaYValue >> 8) & 0xFF;
  
  buffer[index++] = lookHexaXValue & 0xFF;
  buffer[index++] = (lookHexaXValue >> 8) & 0xFF;
  
  buffer[index++] = lookHexaYValue & 0xFF;
  buffer[index++] = (lookHexaYValue >> 8) & 0xFF;
  
  // Pack all button states into a single byte (1 byte total)
  uint8_t buttonByte = 0;
  buttonByte |= (moveSwitchState ? 1 : 0) << 0;
  buttonByte |= (lookSwitchState ? 1 : 0) << 1;
  buttonByte |= (b1 ? 1 : 0) << 2;
  buttonByte |= (b2 ? 1 : 0) << 3;
  buttonByte |= (b3 ? 1 : 0) << 4;
  buttonByte |= (b4 ? 1 : 0) << 5;
  // Bits 6-7 are unused
  buffer[index++] = buttonByte;
  
  // Pack percentage values (0-100) as single bytes (4 bytes total)
  uint8_t mxPercent = 50; // Default to center
  if (moveHexaXValue != 0) {
    mxPercent = map(moveHexaXValue + (moveHexaXValue > 0 ? DEADZONE_MAX : DEADZONE_MIN), 
                    0, 4095, 0, 100);
  }
  buffer[index++] = mxPercent;
  
  uint8_t myPercent = 50; // Default to center
  if (moveHexaYValue != 0) {
    myPercent = map(moveHexaYValue + (moveHexaYValue > 0 ? DEADZONE_MAX : DEADZONE_MIN), 
                    0, 4095, 0, 100);
  }
  buffer[index++] = myPercent;
  
  uint8_t lxPercent = 50; // Default to center
  if (lookHexaXValue != 0) {
    lxPercent = map(lookHexaXValue + (lookHexaXValue > 0 ? DEADZONE_MAX : DEADZONE_MIN), 
                    0, 4095, 0, 100);
  }
  buffer[index++] = lxPercent;
  
  uint8_t lyPercent = 50; // Default to center
  if (lookHexaYValue != 0) {
    lyPercent = map(lookHexaYValue + (lookHexaYValue > 0 ? DEADZONE_MAX : DEADZONE_MIN), 
                    0, 4095, 0, 100);
  }
  buffer[index++] = lyPercent;
  
  // Pack timestamp (4 bytes total)
  unsigned long timestamp = millis();
  buffer[index++] = timestamp & 0xFF;
  buffer[index++] = (timestamp >> 8) & 0xFF;
  buffer[index++] = (timestamp >> 16) & 0xFF;
  buffer[index++] = (timestamp >> 24) & 0xFF;
  
  // Publish binary data to MQTT topic with minimal topic name
  client.publish("r", buffer, sizeof(buffer));
  
  // For debugging (optional - can be removed in production)
  Serial.print("Binary data sent: ");
  for (int i = 0; i < sizeof(buffer); i++) {
    Serial.print(buffer[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting ESP32 MQTT Controller!");

  // Set up pin modes
  pinMode(EmoteButton, INPUT);
  pinMode(PayloadDrop, INPUT);
  pinMode(GaitButton, INPUT);
  pinMode(ModeButton, INPUT);
  pinMode(MoveSwitch, INPUT_PULLUP);
  pinMode(LookSwitch, INPUT_PULLUP);
  pinMode(KillSwitch, INPUT);  // Kill switch input

  // Set up the buffer size for MQTT (increase if needed for larger messages)
  client.setBufferSize(512);

  // Initialize MQTT
  Serial.println("Starting MQTT...");
  setup_wifi();
  if (mqttEnabled) {
    client.setServer(mqtt_server, mqtt_port);
    reconnect();
    if (client.connected()) {
      Serial.println("MQTT is connected");
    }
  }
}

void loop() {
  // Check the Kill Switch
  bool killSwitchActive = (debounceButton(KillSwitch, 6) == HIGH);
  
  if (killSwitchActive) {
    // If kill switch is active, disconnect MQTT
    if (client.connected()) {
      client.disconnect();
      Serial.println("MQTT disconnected due to kill switch");
    }
    
    delay(100); // Short delay while kill switch is active
    return; // Skip the rest of the loop
  }
  
  // If kill switch was just released, reconnect MQTT
  static bool lastKillSwitchState = false;
  if (lastKillSwitchState && !killSwitchActive) {
    // Restart MQTT
    if (mqttEnabled && !client.connected()) {
      setup_wifi();
      client.setServer(mqtt_server, mqtt_port);
      reconnect();
      Serial.println("MQTT reconnected after kill switch release");
    }
  }
  lastKillSwitchState = killSwitchActive;
  
  // Read inputs
  // Read the analog values for joysticks and apply deadzone
  moveHexaXValue = applyDeadzone(analogRead(MoveHexaX));
  moveHexaYValue = applyDeadzone(analogRead(MoveHexaY));
  lookHexaXValue = applyDeadzone(analogRead(LookHexaX));
  lookHexaYValue = applyDeadzone(analogRead(LookHexaY));

  // Read the digital values for buttons with debouncing
  b1 = debounceButton(EmoteButton, 0);
  b2 = debounceButton(PayloadDrop, 1);
  b3 = debounceButton(GaitButton, 2);
  b4 = debounceButton(ModeButton, 3);
  moveSwitchState = debounceButton(MoveSwitch, 4);
  lookSwitchState = debounceButton(LookSwitch, 5);
  
  // Handle MQTT
  if (mqttEnabled) {
    // MQTT client loop to process callbacks
    client.loop();
    
    // Send data at regular intervals
    unsigned long currentTime = millis();
    if (currentTime - lastMQTTSendTime >= mqttSendInterval) {
      lastMQTTSendTime = currentTime;
      sendCommand();
    }
  }
  
  // Small delay to stabilize readings
  delay(20);
}