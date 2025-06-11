// 1. Підключаємо необхідні бібліотеки
#include <WiFi.h>
#include "lds_all_models.h"
#include <math.h>

// =======================================================================
// 2. НАЛАШТУВАННЯ
// =======================================================================
const char* ssid = "Hitm@n"; // ім'я WIFI
const char* password = "22446688"; // Пароль від WIFI
const char* PC_IP = "192.168.0.104";   // IP комп'ютера
const uint16_t LIDAR_DATA_PORT = 8888; // Порт для НАДАННЯ даних лідара
const uint16_t CONTROL_PORT = 8889;    // Порт для ПРИЙОМУ команд управління

// Піни моторів
#define ENA 23
#define IN1 13
#define IN2 12
#define ENB 22
#define IN3 14
#define IN4 27

// Піни та модель лідара
const uint8_t LIDAR_GPIO_EN = 19; 
const uint8_t LIDAR_GPIO_RX = 16; 
const uint8_t LIDAR_GPIO_TX = 17; 
const uint8_t LIDAR_GPIO_PWM = 15;
#define XIAOMI_LDS02RR

// Загальні налаштування
const uint32_t SERIAL_MONITOR_BAUD = 115200;
const uint32_t LIDAR_PWM_FREQ = 10000;
const uint8_t LIDAR_PWM_BITS = 11;
const uint8_t LIDAR_PWM_CHANNEL = 2;

// =======================================================================
// 3. ОБ'ЄКТИ ТА ГЛОБАЛЬНІ ЗМІННІ
// =======================================================================
WiFiClient lidar_data_client; // Клієнт для надсилання даних на ПК
WiFiServer control_server(CONTROL_PORT); // Сервер для прийому команд
WiFiClient control_client;  // Підключений пульт керування

LDS *lidar;
HardwareSerial LidarSerial(1);
String scanBuffer = "";
int motorSpeed = 200;
unsigned long lastLidarConnectionAttempt = 0;
const long connectionInterval = 2000;

// =======================================================================
// 4. ДЕКЛАРАЦІЯ ВСІХ ФУНКЦІЙ
// =======================================================================
void stopRobot(); void moveForward(); void moveBackward(); void turnLeft(); void turnRight();
void pivotLeft(); void pivotRight(); void setupWifi(); void setupLidar();
int lidar_serial_read_callback(); size_t lidar_serial_write_callback(const uint8_t * buffer, size_t length);
void lidar_motor_pin_callback(float value, LDS::lds_pin_t lidar_pin);
void lidar_scan_point_callback(float angle_deg, float distance_mm, float quality, bool scan_completed);

// =======================================================================
// 5. ГОЛОВНА ФУНКЦІЯ SETUP()
// =======================================================================
void setup() {
  Serial.begin(SERIAL_MONITOR_BAUD);
  while(!Serial);
  Serial.println("\nWiFi Dual-Port Robot starting up...");
  
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopRobot();

  setupWifi();
  
  control_server.begin();
  Serial.print("Control server started on port: "); Serial.println(CONTROL_PORT);
  
  setupLidar();
  LDS::result_t result = lidar->start();
  if (result != LDS::RESULT_OK) {
    Serial.println("Lidar start FAILED. Halting."); while(1);
  }
  Serial.println("Lidar started. System ready.");
}

// =======================================================================
// 6. ГОЛОВНА ФУНКЦІЯ LOOP()
// =======================================================================
void loop() {
  // --- ЗАВДАННЯ 1: Прийняти нового клієнта для управління ---
  if (control_server.hasClient()) {
    if (control_client && control_client.connected()) {
      control_client.stop();
      Serial.println("Old control client disconnected.");
    }
    control_client = control_server.available();
    Serial.println("New control client connected!");
  }
  
  // --- ЗАВДАННЯ 2: Перевірити команди від підключеного пульта ---
  if (control_client && control_client.connected() && control_client.available()) {
    char command = control_client.read();
    Serial.print("Control command received: "); Serial.println(command);
    switch (command) {
      case 'w': moveForward(); break; case 's': moveBackward(); break;
      case 'a': turnLeft(); break;    case 'd': turnRight(); break;
      case 'q': pivotLeft(); break;   case 'e': pivotRight(); break;
      case 'x': case 'k': stopRobot(); break;
      case '1': motorSpeed = 50; break; case '2': motorSpeed = 75; break;
      case '3': motorSpeed = 100; break; case '4': motorSpeed = 125; break;
      case '5': motorSpeed = 150; break; case '6': motorSpeed = 175; break;
      case '7': motorSpeed = 200; break; case '8': motorSpeed = 225; break;
      case '9': motorSpeed = 255; break;
    }
  }
  
  // --- ЗАВДАННЯ 3: Надсилати дані лідара на ПК ---
  if (!lidar_data_client.connected()) {
    unsigned long currentMillis = millis();
    if (currentMillis - lastLidarConnectionAttempt >= connectionInterval) {
      lastLidarConnectionAttempt = currentMillis;
      Serial.print("Connecting to Lidar PC server... ");
      if (lidar_data_client.connect(PC_IP, LIDAR_DATA_PORT)) {
        Serial.println("Connected!");
      } else {
        Serial.println("Failed.");
      }
    }
  }
  lidar->loop();
}

// =======================================================================
// 7. ДОПОМОЖНІ ФУНКЦІЇ
// =======================================================================
void setupWifi() {
  Serial.print("Connecting to WiFi: "); Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP address: "); Serial.println(WiFi.localIP());
}

void lidar_scan_point_callback(float angle_deg, float distance_mm, float quality, bool scan_completed) {
  if (!lidar_data_client.connected()) return;
  if (distance_mm <= 10.0 || distance_mm > 12000.0) return;
  float angle_rad = angle_deg * M_PI / 180.0;
  int x_mm = round(distance_mm * cos(angle_rad));
  int y_mm = round(distance_mm * sin(angle_rad));
  String pointString = "p," + String(x_mm) + "," + String(y_mm) + "\n";
  scanBuffer += pointString;
  if (scan_completed) {
    if (scanBuffer.length() > 0) { lidar_data_client.print(scanBuffer); }
    lidar_data_client.println("s");
    scanBuffer = "";
  }
}

void lidar_motor_pin_callback(float value, LDS::lds_pin_t lidar_pin) {
  int pin = (lidar_pin == LDS::LDS_MOTOR_EN_PIN) ? LIDAR_GPIO_EN : LIDAR_GPIO_PWM;
  if (value <= (float)LDS::DIR_INPUT) {
    if (value == (float)LDS::DIR_OUTPUT_PWM) {
      #if ESP_IDF_VERSION_MAJOR < 5
      ledcSetup(LIDAR_PWM_CHANNEL, LIDAR_PWM_FREQ, LIDAR_PWM_BITS);
      ledcAttachPin(pin, LIDAR_PWM_CHANNEL);
      #else
      if (!ledcAttachChannel(pin, LIDAR_PWM_FREQ, LIDAR_PWM_BITS, LIDAR_PWM_CHANNEL))
        Serial.println("lidar_motor_pin_callback() ledcAttachChannel() error");
      #endif
    } else pinMode(pin, (value == (float)LDS::DIR_INPUT) ? INPUT : OUTPUT);
    return;
  }
  if (value < (float)LDS::VALUE_PWM) {
    digitalWrite(pin, (value == (float)LDS::VALUE_HIGH) ? HIGH : LOW);
  } else {
    #ifdef INVERT_PWM_PIN
    value = 1 - value;
    #endif
    int pwm_value = ((1<<LIDAR_PWM_BITS)-1)*value;
    #if ESP_IDF_VERSION_MAJOR < 5
    ledcWrite(LIDAR_PWM_CHANNEL, pwm_value);
    #else
    ledcWriteChannel(LIDAR_PWM_CHANNEL, pwm_value);
    #endif
  }
}

int lidar_serial_read_callback() { return LidarSerial.read(); }
size_t lidar_serial_write_callback(const uint8_t * buffer, size_t length) { return LidarSerial.write(buffer, length); }

void setupLidar() {
  lidar = new LDS_LDS02RR();
  lidar->setScanPointCallback(lidar_scan_point_callback);
  lidar->setMotorPinCallback(lidar_motor_pin_callback);
  lidar->setSerialWriteCallback(lidar_serial_write_callback);
  lidar->setSerialReadCallback(lidar_serial_read_callback);
  lidar->setPacketCallback([](uint8_t*,uint16_t,bool){});
  lidar->setInfoCallback([](LDS::info_t, String){});
  lidar->setErrorCallback([](LDS::result_t, String){});
  LidarSerial.begin(lidar->getSerialBaudRate(), SERIAL_8N1, LIDAR_GPIO_TX, LIDAR_GPIO_RX);
  lidar->init();
}

void moveForward() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); analogWrite(ENA, motorSpeed);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); analogWrite(ENB, motorSpeed);
}
void moveBackward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  analogWrite(ENA, motorSpeed);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  analogWrite(ENB, motorSpeed);
}
void turnLeft() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); analogWrite(ENA, motorSpeed);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); analogWrite(ENB, motorSpeed / 2);
}
void turnRight() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); analogWrite(ENA, motorSpeed / 2);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); analogWrite(ENB, motorSpeed);
}
void pivotLeft() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); analogWrite(ENA, motorSpeed);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  analogWrite(ENB, motorSpeed);
}
void pivotRight() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  analogWrite(ENA, motorSpeed);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); analogWrite(ENB, motorSpeed);
}
void stopRobot() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}