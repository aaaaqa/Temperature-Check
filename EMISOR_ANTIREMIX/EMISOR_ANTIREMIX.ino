#include <esp_now.h>
#include <WiFi.h>
#include <max6675.h>
#include <ESP32Ping.h>
#include <EEPROM.h>

#define EEPROM_SIZE 6

#define PIN_ANALOGICO 34
#define VOLTAJE_MAX 3.3
#define VOLTAJE_MIN 0.0
#define VOLTAJE_BATERIA_MAX 6.5
#define VOLTAJE_BATERIA_MIN 3.9
#define DIVISOR_RESISTENCIA 1.97

int SO = 14, CS = 15, CSK = 12;
MAX6675 termocupla(CSK, CS, SO);

RTC_DATA_ATTR uint8_t esp2Address[6];// = {0x08, 0xD1, 0xF9, 0xDC, 0xDC, 0xBC };

// RTC_DATA_ATTR uint8_t esp2Address[] = {0xD4, 0x8A, 0xFC, 0xCE, 0xF3, 0xAC };

constexpr int buttonPin = 2, ledR = 23, ledG = 22, ledB = 21, numMeasurements = 10;

bool macSent, buttonPressed, sequenceInProgress, bouncePress, stableTemperature, deepSleep;

int failCounter, timeSinceLastFailure, sequenceCountdown, initialNumber, bounceCountdown;
int lastButtonPressTime, buttonPressCounter, currentIndex, sendTime, deepSleepTime;

float temperatureReadings[numMeasurements];
float currentTemperature, averageTemperature;

void initializeVariables() {
  buttonPressed = false;
  deepSleep = true;
  macSent = false;
  stableTemperature = false;
  bouncePress = true;
  sequenceInProgress = false;

  failCounter = 0;
  deepSleepTime = 0;
  buttonPressCounter = 0;
  timeSinceLastFailure = 0;
  lastButtonPressTime = 0;
  sequenceCountdown = 0;
  currentIndex = 0;
  sendTime = 0;
  averageTemperature = 0;

  memset(temperatureReadings, 0, sizeof(temperatureReadings));

  EEPROM.begin(EEPROM_SIZE);
}

int tempWIFIOFF = 0;
int tempWIFION = 0;

void setup() {
  initializeVariables();
  
  esp_sleep_enable_ext0_wakeup((gpio_num_t)buttonPin, 1);
  Serial.begin(115200);
  analogReadResolution(12);

  WiFi.mode(WIFI_STA);

  pinMode(ledR, OUTPUT);
  pinMode(ledG, OUTPUT);
  pinMode(ledB, OUTPUT);
  pinMode(buttonPin, INPUT);

  setRGBColorSequence(255, 0, 255, 4);

  Serial.println("Conectado.");

  if (esp_now_init() != ESP_OK) {
      Serial.println("Error al inicializar ESP-NOW");
      deepSleepTime = 30000;
      deepSleep = true;
      return;
  }
  wl_status_t wifiStatus = WiFi.status();

  checkConnection();
  //WiFi.disconnect();
}

void setRGBColor(int red, int green, int blue) {
  analogWrite(ledR, red);
  analogWrite(ledG, green);
  analogWrite(ledB, blue);
}

void setRGBColorSequence(int red, int green, int blue, int repetitions) {
  for (int i = 0; i < repetitions; i++) {
    setRGBColor(red, green, blue);
    delay(500);
    setRGBColor(255, 255, 255);
    delay(500);
  }
}

float getThermocoupleMeasure() 
{
  return termocupla.readCelsius();
}

float calculateBattery(bool V)
{
  int lecturaADC = analogRead(PIN_ANALOGICO);
  float voltajePin = (lecturaADC / 4095.0) * VOLTAJE_MAX;
  float voltajeBateria = voltajePin * DIVISOR_RESISTENCIA;
  float porcentaje = ((voltajeBateria - VOLTAJE_BATERIA_MIN) / (VOLTAJE_BATERIA_MAX - VOLTAJE_BATERIA_MIN)) * 100.0;
  porcentaje = constrain(porcentaje, 0, 100);
  
  float result = V ? porcentaje : voltajeBateria;

  return result;
}

bool automaticBatteryCheck()
{
  return calculateBattery(false) >= 3.5;
}

void manualBatteryCheck()
{
  float porcentaje = calculateBattery(true);

  int red = (porcentaje <= 20.0) ? 0 : int(porcentaje * 255 / 100);
  int green = 255 - red;

  Serial.printf("Bateria actual: %.2f %\n", porcentaje);
  Serial.printf("Red: %.2f, Green: %.2f\n", red, green);

  setRGBColorSequence(red, green, 255, 4);
}


bool checkConnection()
{
  esp_now_register_send_cb(onSent);
  esp_now_register_recv_cb(onReceive);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, esp2Address, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("No se pudo agregar el peer");
    setRGBColor(0, 0, 255);
    delay(15000);
    return false;
  }

  return true;
}

void onSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    macSent = !macSent;
    Serial.println("Mensaje enviado con éxito");

    if (!macSent) {
      Serial.println("Secuencia completada, deteniendo reintentos.");
    }
  } else {
    failCounter += 1;
    timeSinceLastFailure = millis();

    Serial.println("Error al enviar mensaje");
    setRGBColor(0, 0, 255);

    if (!macSent) {
      Serial.println("Error en el mensaje inicial, secuencia terminada.");
      deepSleepTime = 10000;
    }
  }
}

void onReceive(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  if (data_len == sizeof(int)) {
    memcpy(&initialNumber, data, sizeof(initialNumber));
    Serial.print("Número inicial recibido: ");
    Serial.println(initialNumber);
  }
  else
  {
    for(int i = 0; i < 6; i++)
    {
        esp2Address[i] = (uint8_t)data[i];
        EEPROM.write(i, (uint8_t)data[i]);
    }

    EEPROM.commit();

    Serial.println("Mac actualizada");

    setRGBColorSequence(255, 255, 0, 5);
  }
}

void sendInitialMessage() 
{
    uint8_t initialMessage[1] = {0xFF};
    esp_now_send(esp2Address, initialMessage, sizeof(initialMessage));
    Serial.println("Mensaje inicial enviado.");
}

void sendTemperatureAndInitial() 
{
  sendTime = millis();
  //currentTemperature = a;  // Mide la temperatura usando el sensor MAX6675
  float dataToSend[2] = { averageTemperature, (float)initialNumber };

  while (macSent) {
    esp_now_send(esp2Address, (uint8_t *)dataToSend, sizeof(dataToSend));
    Serial.printf("Intentando enviar temperatura: %.2f y número inicial: %d\n", averageTemperature, initialNumber);
    delay(4000);
  }
}

bool checkTemperatureStability() {
  float maxTemp = temperatureReadings[0];
  float minTemp = temperatureReadings[0];

  averageTemperature = 0;

  for (int i = 0; i < numMeasurements; i++) {
    maxTemp = (temperatureReadings[i] > maxTemp) ? temperatureReadings[i] : maxTemp;
    minTemp = (temperatureReadings[i] < minTemp) ? temperatureReadings[i] : minTemp;
    averageTemperature += temperatureReadings[i];
  }

  averageTemperature /= numMeasurements;

  return (maxTemp - minTemp) <= 0.5;
}

void measureSequence() {
  while (!stableTemperature) {
    currentTemperature = getThermocoupleMeasure();
    temperatureReadings[currentIndex] = currentTemperature;
    Serial.println(currentTemperature);
    currentIndex = (currentIndex + 1) % numMeasurements;
    stableTemperature = checkTemperatureStability();
    delay(1000);
  }

  Serial.println("Temperatura estable, se enviarán los datos:");

  for (int i = 0; i < numMeasurements; i++) {
    Serial.print(temperatureReadings[i]);
    Serial.print(" ");
  }
  Serial.println();
}

void sendDeepSleep(int delayTime) {
  deepSleep = true;
  Serial.printf("Durmiendo en %d.\n", delayTime);
  WiFi.mode(WIFI_OFF);
  delay(delayTime);
  setRGBColor(255, 255, 255);
  delay(500);
  //esp_sleep_enable_timer_wakeup(300 * 1000000);
  esp_deep_sleep_start();
}

void loop() {

  if (digitalRead(buttonPin) == HIGH) {
    if (deepSleep) {
      esp_sleep_enable_ext0_wakeup((gpio_num_t)buttonPin, 1);
      deepSleep = false;
    } else {
      if (!buttonPressed) {
        sequenceCountdown = millis();
        buttonPressed = true;
      }
      sequenceInProgress = (buttonPressed && sequenceCountdown != 0 && millis() - sequenceCountdown >= 3000) ? true : false;    
    }

    bounceCountdown = millis();

    buttonPressCounter = bouncePress ? buttonPressCounter + 1 : buttonPressCounter;
    bouncePress = false;

  } else {
    bouncePress = (millis() - bounceCountdown >= 500) ? true : false;
    buttonPressed = false;
  }

  if (buttonPressCounter >= 3) {
    buttonPressCounter = 0;
    Serial.println("Checking battery...");
    manualBatteryCheck();
  }

  buttonPressCounter = (millis() - bounceCountdown >= 3000) ? 0 : buttonPressCounter;

  if (deepSleepTime != 0) {
    sendDeepSleep(deepSleepTime);
  }

  if (timeSinceLastFailure != 0 && millis() - timeSinceLastFailure >= 60000) {
    Serial.println(failCounter);
    failCounter = 0;
    deepSleepTime = 10000;
  }

  if (sequenceInProgress) {
    if (automaticBatteryCheck()) {
      WiFi.mode(WIFI_STA);
      esp_now_init();

      for(int i = 0; i < 6; i++)
      {
        esp2Address[i] = EEPROM.read(i); 
      }

      if (checkConnection()) {
        WiFi.reconnect();
        sendInitialMessage();
        delay(1000);
        if (macSent) {
          setRGBColor(255, 255, 0);
          WiFi.mode(WIFI_OFF);
          measureSequence();
          WiFi.mode(WIFI_STA);
          esp_now_init();
          if (checkConnection())
          {
            WiFi.reconnect();
            sendTemperatureAndInitial();
            WiFi.mode(WIFI_OFF);
            setRGBColor(255, 0, 255);
          }
        }
      } 
      deepSleepTime = 30000;
    } else {
      setRGBColorSequence(0, 255, 255, 10);
      deepSleepTime = 10000;
    }
  }

  //(deepSleepTime != 0) ? sendDeepSleep(deepSleepTime) : "";
}