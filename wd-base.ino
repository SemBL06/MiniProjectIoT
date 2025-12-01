//S168772 Dries Van Bouwel
//S171282 Lukas Van Dyck
//S173062 Karol Czaplejewicz
//S166965 Sem Block



// Necessary Libraries
#include <LiquidCrystal_I2C.h>  // https://github.com/johnrickman/LiquidCrystal_I2C (install as zip)
#include <WiFi.h>               // Standard ESP32 Library
#include <PubSubClient.h>       // https://github.com/knolleary/pubsubclient (intall with library Manager)

#define CLASSGROUP "1it2"
#define LABGROUP "7"

const char WIFI_SSID[] = "bletchley";
const char WIFI_PASS[] = "laptop!internet";
const char MQTT_SERVER[] = "mqtt.iote-ap.be";

//UPGRADE AUTOMATIC GENERATION OF THIS VARIABLES BASED
const char MQTT_ID[] = CLASSGROUP "-" LABGROUP;
const char SPEED_TOPIC[] = "mp/" CLASSGROUP "/" LABGROUP "/wd/speed";
const char MOVEMENT_TOPIC[] = "mp/" CLASSGROUP "/" LABGROUP "/wd/movement";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

char msg[32];

const int PWM_FREQUENCY = 500;
const int PWM_RESOLUTION = 8;
const int PWM_MAX_DC = 255;

const int MOTOR_PIN_COUNT = 4;
const int MOTOR_COUNT = 2;

int MOTOR_PINS[MOTOR_PIN_COUNT] = { 18, 5, 2, 15 };

const int LED_COUNT = 4;

int LED_PINS[LED_COUNT] = { 13, 12, 14, 27 };

int LED_STATE[] = { 0, 0, 0, 0 };

const int PL_PIN = 25;
const int TRIG_PIN = 19;
const int ECHO_PIN = 23;

int LCD_COLUMNS = 16;
int LCD_ROWS = 2;

enum COMMANDS {
  LEFT_TURN_FORWARD,
  FORWARD,
  RIGHT_TURN_FORWARD,
  LEFT,
  STOP,
  RIGHT,
  LEFT_TURN_BACKWARDS,
  BACKWARDS,
  RIGHT_TURN_BACKWARDS,
};

const char* directionNames[] = {
  "LEFT_TURN_FORWARD",  //0
  "FORWARD",            //1
  "RIGHT_TURN_FORWARD",
  "LEFT",
  "STOP",
  "RIGHT",
  "LEFT_TURN_BACKWARDS",
  "BACKWARDS",
  "RIGHT_TURN_BACKWARDS",
};

enum MOTOR_DIRECTION {
  MOTOR_BACKWARDS = -1,
  MOTOR_STOP = 0,
  MOTOR_FORWARD = 1
};

LiquidCrystal_I2C lcd(0x27, LCD_COLUMNS, LCD_ROWS);

int drivingSpeed = 128;

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect_to_mqtt() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect(MQTT_ID)) {
      Serial.println("connected");
      // Subscribe
      mqttClient.subscribe(SPEED_TOPIC);
      mqttClient.subscribe(MOVEMENT_TOPIC);
      mqttClient.publish(MOVEMENT_TOPIC, "connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void callback(char* topic, byte* message, unsigned int length) {
  memset(msg, '\0', sizeof(msg));
  memcpy(msg, (char*)message, length);
  Serial.print("Recieved message from broker");
  Serial.print("Topic: ");
  Serial.println(topic);
  Serial.print("msg: ");
  Serial.println(msg);
  Serial.println();

  if (strcmp(topic, SPEED_TOPIC) == 0) {
    handle_speed_command(msg);
    return;
  }

  if (strcmp(topic, MOVEMENT_TOPIC) == 0) {
    handle_movement_command(msg);
    return;
  }
}

void handle_movement_command(const char* msg) {
  static int lastCommand = -1;
  int command = atoi(msg);
    lcd.clear();
    Serial.print("Command: ");
    Serial.println(command);
    lcd.print(directionNames[command]);
    lcd.setCursor(0,1);

    lcd.print("Speed: " + String(drivingSpeed));
    lcd.setCursor(0,0);
  if (command != lastCommand) {
    switch (command) {
      case LEFT_TURN_FORWARD:
        driveMotors(MOTOR_FORWARD, drivingSpeed, MOTOR_FORWARD, drivingSpeed / 2);
        break;
      case FORWARD:
        driveMotors(MOTOR_FORWARD, drivingSpeed, MOTOR_FORWARD, drivingSpeed);
        break;
      case RIGHT_TURN_FORWARD:
        driveMotors(MOTOR_FORWARD, drivingSpeed / 2, MOTOR_FORWARD, drivingSpeed);
        break;
      case RIGHT:
        driveMotors(MOTOR_BACKWARDS, drivingSpeed, MOTOR_FORWARD, drivingSpeed);
        break;
      case RIGHT_TURN_BACKWARDS:
        driveMotors(MOTOR_BACKWARDS, drivingSpeed / 2, MOTOR_BACKWARDS, drivingSpeed);
        break;
      case BACKWARDS:
        driveMotors(MOTOR_BACKWARDS, drivingSpeed, MOTOR_BACKWARDS, drivingSpeed);
        break;
      case LEFT_TURN_BACKWARDS:
        driveMotors(MOTOR_BACKWARDS, drivingSpeed, MOTOR_BACKWARDS, drivingSpeed / 2);
        break;
      case LEFT:
        driveMotors(MOTOR_FORWARD, drivingSpeed, MOTOR_BACKWARDS, drivingSpeed);
        break;
      case STOP:
        driveMotors(MOTOR_STOP, 0, MOTOR_STOP, 0);
        break;
    }
    lastCommand = command;
  }
}

void handle_speed_command(const char* msg) {
  int speed = atoi(msg);
  if (speed < 0 || speed > 255) {
    Serial.println("Invalid speed received. Ignoring.");
    return;
  }
  drivingSpeed = speed;
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < MOTOR_PIN_COUNT; i++) {
    pinMode(MOTOR_PINS[i], OUTPUT);
    ledcAttach(MOTOR_PINS[i], PWM_FREQUENCY, PWM_RESOLUTION);  // Attach pin to channel i
  }

  for (int i = 0; i < LED_COUNT; i++) {
    pinMode(LED_PINS[i], OUTPUT);
  }
  pinMode(TRIG_PIN, OUTPUT);  // Sets the trigPin as an Output
  pinMode(ECHO_PIN, INPUT);   // Sets the echoPin as an Input
  lcd.init();
  lcd.backlight();
  setup_wifi();
  mqttClient.setServer(MQTT_SERVER, 1883);
  mqttClient.setCallback(callback);
}



void loop() {
  if (!mqttClient.connected()) {
    reconnect_to_mqtt();
  }
  mqttClient.loop();
}

void driveMotors(int leftMotorDirection, int leftMotorSpeed, int rightMotorDirection, int rightMotorSpeed) {
  // Control Left Motor (PINS 0 and 1)
  if (leftMotorDirection == MOTOR_FORWARD) {
    ledcWrite(MOTOR_PINS[0], leftMotorSpeed);  // Forward channel active
    ledcWrite(MOTOR_PINS[1], 0);                // Backward channel off
  } 
  else if (leftMotorDirection == MOTOR_BACKWARDS) {
    ledcWrite(MOTOR_PINS[0], 0);                // Forward channel off
    ledcWrite(MOTOR_PINS[1], leftMotorSpeed);  // Backward channel active
  } 
  else { // MOTOR_STOP
    ledcWrite(MOTOR_PINS[0], 0);                // Both PINS off
    ledcWrite(MOTOR_PINS[1], 0);
  }

  // Control Right Motor (PINS 2 and 3)
  if (rightMotorDirection == MOTOR_FORWARD) {
    ledcWrite(MOTOR_PINS[2], rightMotorSpeed); // Forward channel active
    ledcWrite(MOTOR_PINS[3], 0);                // Backward channel off
  } 
  else if (rightMotorDirection == MOTOR_BACKWARDS) {
    ledcWrite(MOTOR_PINS[2], 0);                // Forward channel off
    ledcWrite(MOTOR_PINS[3], rightMotorSpeed); // Backward channel active
  } 
  else { // MOTOR_STOP
    ledcWrite(MOTOR_PINS[2], 0);                // Both PINS off
    ledcWrite(MOTOR_PINS[3], 0);
  }
}
