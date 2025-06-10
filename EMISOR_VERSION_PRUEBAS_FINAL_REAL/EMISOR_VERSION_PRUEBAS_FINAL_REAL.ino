#include <esp_now.h>
#include <WiFi.h>
#include <max6675.h>  // Incluye la librería para el sensor MAX6675
#include <ESP32Ping.h>

#define PIN_ANALOGICO 34  // Cambia al pin que estás utilizando
#define VOLTAJE_MAX 3.3   // Voltaje máximo que recibe el pin del divisor
#define VOLTAJE_MIN 0.0   // Voltaje mínimo esperado en el pin
#define VOLTAJE_BATERIA_MAX 6.5 // Voltaje máximo de la batería real
#define VOLTAJE_BATERIA_MIN 3.5 // Voltaje mínimo de la batería real
#define DIVISOR_RESISTENCIA 1.97 // Factor del divisor (por ejemplo, 9V -> 3.3V implica un divisor de 3)

// Pines para el sensor MAX6675
int SO = 14;
int CS = 15;
int CSK = 12;
MAX6675 termocupla(CSK, CS, SO);

const uint8_t esp2Address[] = {0x08, 0xD1, 0xF9, 0xDC, 0xDC, 0xBC};//{0xD4, 0x8A, 0xFC, 0xCE, 0xF3, 0xAC};//{0x3C, 0x8A, 0x1F, 0x1E, 0x66, 0x84};//{0x3C, 0x8A, 0x1F, 0x1F, 0x0D, 0x68};//{0xD4, 0x8A, 0xFC, 0xCE, 0xF3, 0xAC};  // MAC de ESP2
const int buttonPin = 2;
const int ledR = 23;
const int ledG = 21;
const int ledB = 22;

bool buttonPress;
bool macSent;
bool sequenceInProgress;
bool stableTemperature;
bool finalMessageSent;
bool buttonUp;
bool waitingTime;
bool pressingHand;

bool deepSleep = false;
bool goToSleep = false;

long buttonPressStartTime = 0;
long lastMeasurementTime = 0;
long sequenceStartTime = 0;
long sendTime = 0;
long lastButtonPressTime = 0;

int deepSleepTime;
int failCounter;
int timeSinceLastFailure;
int timeSinceFailure;
int currentIndex;
int initialNumber;
int buttonPressCount;

/*  Tiempo de espera arbitrario de 3 segundos para iniciar la conexión. */
constexpr int holdingButtonStartTime = 3000;
/*  Duración arbitraria de 10 segundos para la secuencia de medición de temperatura. */
constexpr int temperatureMeasurementsDuration = 10000;
constexpr int timeBetweenTemperatureMeasurements = 1000;
constexpr int waitingTimeSinceFailure = 1000;
constexpr int timeToSleepSinceFailure = 10000;
constexpr int maxSendTime = 22500;
constexpr int timeBetweenSendRetries = 4000;
constexpr int timeBetweenBatteryChecks = 1000;
constexpr int pressTimeWindow = 3000;
constexpr int numMeasurements = 10;
constexpr int buttonPressThreshold = 3;

/*  Arreglo de temperatura. */
float temperatureReadings[numMeasurements];
float currentTemperature = 0

void initializeVariables()
{
    // BOOL
    buttonPress = false;
    macSent = false;
    sequenceInProgress = false;
    stableTemperature = false;
    finalMessageSent = false;
    buttonUp = true;
    waitingTime = true;
    pressingHand = false;
    goToSleep = false;
    deepSleep = true;

    // INT
    failCounter = 0;
    timeSinceLastFailure = 0;
    timeSinceFailure = 0;
    currentIndex = 0;
    initialNumber = 0;
    buttonPressCount = 0;

    // FLOAT*
    memset(temperatureReadings, 0, sizeof(temperatureReadings));
}

void setup() 
{
    initializeVariables();
    //sequenceInProgress = false;
    esp_sleep_enable_ext0_wakeup((gpio_num_t)buttonPin, 1);
    Serial.begin(115200);
    analogReadResolution(12); // Resolución de 12 bits (0-4095)

    WiFi.mode(WIFI_STA);

    pinMode(ledR, OUTPUT);
    pinMode(ledG, OUTPUT);
    pinMode(ledB, OUTPUT);

    /*  Prendemos el led intermitentemente de color verde por 3 segundos. */
    setRGBColor_Sequence(255, 0, 255, 500, 3);

    Serial.println("Conectado.");

    if (esp_now_init() != ESP_OK) 
    {
        Serial.println("Error al inicializar ESP-NOW");
        return;
    }

    checkConnection();

    pinMode(buttonPin, INPUT);
}

void setRGBColor(int red, int green, int blue) 
{
    analogWrite(ledR, red);
    analogWrite(ledG, green);
    analogWrite(ledB, blue);
}

void setRGBColor_Sequence(int red, int green, int blue, int delayTime, int repetitions) 
{
    for(int i = 0; i < repetitions; i++)
    {
        setRGBColor(red, green, blue);
        delay(delayTime);
        setRGBColor(255, 255, 255);
        delay(delayTime);
    }
}

void checkConnection()
{
    esp_now_register_send_cb(onSent);
    esp_now_register_recv_cb(onReceive);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, esp2Address, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK) 
    {
        Serial.println("No se pudo agregar el peer");
        return;
    }
}

void onSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if(status == ESP_NOW_SEND_SUCCESS) 
    {
        Serial.println("Mensaje enviado con éxito");
        /*  Verifica si se envió el mensaje inicial con la dirección MAC. */
        if(!macSent) 
        {
            macSent = true;
            sequenceInProgress = true;
        }
        /*  Verifica si se envió el mensaje inicial con la dirección MAC
            y la secuencia está en progreso. */
        else if(sequenceInProgress) 
        {
            finalMessageSent = true;
            Serial.println("Secuencia completada, deteniendo reintentos.");
            sequenceInProgress = false;
        }
    }
    else 
    {
        sequenceInProgress = false;
        failCounter += 1;
        timeSinceLastFailure = millis();
        
        Serial.println("Error al enviar mensaje");
        /*  Prendemos el led de color amarillo para indicar que hubo un error enviando el mensaje al receptor. */
        setRGBColor(0, 0, 255);
        /*  Verifica si se envió la dirección MAC. */
        if (!macSent) 
        {
            Serial.println("Error en el mensaje inicial, secuencia terminada.");
        }
    }
}

void onReceive(const uint8_t *mac_addr, const uint8_t *data, int data_len) 
{   
    if (data_len == sizeof(int)) 
    {
        memcpy(&initialNumber, data, sizeof(initialNumber));
        Serial.print("Número inicial recibido: ");
        Serial.println(initialNumber);
        /*  Apagamos el Wi-Fi para no consumir energía adicional. */
        //WiFi.disconnect();
        //WiFi.mode(WIFI_OFF);
    }
}

void sendInitialMessage() 
{
    uint8_t initialMessage[1] = {0xFF};
    esp_now_send(esp2Address, initialMessage, sizeof(initialMessage));
    Serial.println("Mensaje inicial enviado.");
}

float getThermocoupleMeasure() 
{
    return 10.0;//termocupla.readCelsius();
}

void sendTemperatureAndInitial() 
{
    WiFi.mode(WIFI_STA);

    float temperature = getThermocoupleMeasure();  // Mide la temperatura usando el sensor MAX6675
    float dataToSend[2] = {temperature, (float)initialNumber};

    delay(500);
    
    while (!finalMessageSent && millis() - sendTime < maxSendTime)
    {
        if (!macSent || finalMessageSent) break;
        
        esp_now_send(esp2Address, (uint8_t *)dataToSend, sizeof(dataToSend));
        Serial.printf("Intentando enviar temperatura: %.2f y número inicial: %d\n", temperature, initialNumber);
        delay(timeBetweenSendRetries);
    }

    if (failCounter != 0)
    {
        Serial.println(failCounter);
    }
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

bool batteryCheck()
{
    float voltajeBateria = calculateBattery(false);
    //return voltajeBateria >= 3.5;
    return true;
}

bool checkTemperatureStability() 
{
    float maxTemp = temperatureReadings[0];
    float minTemp = temperatureReadings[0];

    for (int i = 1; i < numMeasurements; i++) 
    {
        if (temperatureReadings[i] > maxTemp) 
        {
            maxTemp = temperatureReadings[i];
        }
        if (temperatureReadings[i] < minTemp) 
        {
            minTemp = temperatureReadings[i];
        }
    }

    // Verifica si la diferencia entre la máxima y mínima es de ±0.5 o menos
    return (maxTemp - minTemp) <= 0.5;
}

bool measureSequence()
{
    sequenceStartTime = millis();
    /*  Se realiza la medición de temperatura durante 10 segundos. */
    while(!stableTemperature)
    {
        /*  Espera un segundo para la siguiente medición. */
        delay(timeBetweenTemperatureMeasurements);
        /*  Función para medir la temperatura. */
        currentTemperature = getThermocoupleMeasure();
        temperatureReadings[currentIndex] = currentTemperature;
        currentIndex = (currentIndex + 1) % numMeasurements;

        stableTemperature = checkTemperatureStability();
    }

    Serial.println("Temperatura estable, se enviarán los datos:");
    for (int i = 0; i < numMeasurements; i++) 
    {
        Serial.print(temperatureReadings[i]);
        Serial.print(" ");
    }
    Serial.println();

    sendTime = millis();
    
    /*  Se reintenta el envío por 20 segundos. */
    sendTemperatureAndInitial();

    /*  Marcar como finalizado. */
    finalMessageSent = true;

    /*  Detener la secuencia. */
    return false;
}

void batteryCheck_()
{
    float porcentaje = calculateBattery(true);

    int red = (porcentaje <= 20.0) ? 0 : int(porcentaje * 255 / 100);
    int green = 255 - red;

    Serial.printf("Bateria actual: %.2f %\n", porcentaje);
    Serial.printf("Red: %.2f, Green: %.2f\n", red, green);

    setRGBColor_Sequence(red, green, 255, 1000, 4);
}

void sendDeepSleep(int delayTime)
{
    Serial.printf("Durmiendo en %d.\n", delayTime);
    WiFi.mode(WIFI_OFF);
    delay(delayTime);
    setRGBColor(255, 255, 255);
    delay(500);
    esp_deep_sleep_start();
}

void loop() 
{
    if(digitalRead(buttonPin) == HIGH)
    {
        /*  Verifica si está en deep sleep y lo despierta. */
        if(deepSleep)
        {
            esp_sleep_enable_ext0_wakeup((gpio_num_t)buttonPin, 1);
            deepSleep = false;
        }
        else
        {
            /*  Como el digitalRead va a seguir == HIGH en el siguiente loop, necesitamos 
                asegurar una forma de mantener el tiempo inicial de presionado. */
            if(!buttonPress)
            {
                buttonPressStartTime = millis();
                buttonPress = true;
            }
            else if(millis() - buttonPressStartTime >= holdingButtonStartTime && !sequenceInProgress)
            {
                /* Verifica si la batería tiene suficiente */
                if(batteryCheck())
                {
                    macSent = false;
                    /*  Enviamos 0xFF al receptor para recibir la posición actual y
                        apagamos el Wi-Fi. */
                    if(millis() - timeSinceLastFailure >= waitingTimeSinceFailure && !pressingHand)
                    {
                        sendInitialMessage();
                        pressingHand = true;
                    }
                }
                else
                {
                    /*  No tiene sufienciente batería, mandamos el
                        dispositivo a deep sleep. */
                    setRGBColor_Sequence(0, 255, 255, 500, 4);
                    deepSleepTime = 10000;
                    goToSleep = true;
                }
            }

            if(buttonUp)
            {
                buttonUp = false;
                buttonPressCount += 1;
                lastButtonPressTime = millis();
                Serial.printf("Button pressed %d times\n", buttonPressCount);
            }
        }
    }
    else
    {
        if(millis() - lastButtonPressTime > timeBetweenBatteryChecks && !sequenceInProgress)
        {  
            buttonUp = true;
        }
        buttonPress = false;
        pressingHand = false;
    }

    if (buttonPressCount == 3) 
    {
        buttonPressCount = 0;
        Serial.println("Checking battery...");
        batteryCheck_();
    }

    if(millis() - lastButtonPressTime > pressTimeWindow)
    {
      buttonPressCount = 0;
    }

    /*  Verifica si se inició correctamente la secuencia. */
    if(sequenceInProgress && macSent && !finalMessageSent)
    {
        /*  Prendemos el led azul durante la secuencia de medición. */
        setRGBColor(255, 255, 0);
        /*  Esperamos 10 segundos para empezar a medir la temperatura. */
        delay(10000);
        /*  Se establece al inicio de la secuencia de medición. */
        sequenceInProgress = measureSequence();
        /*  Esperamos 60 segundos. */
        setRGBColor(255, 0, 255);
        /* Lo mandamos a dormir. */
        deepSleepTime = 60000;
        goToSleep = true;
    }

    if(goToSleep && !deepSleep)
    {
        goToSleep = false;
        deepSleep = true;
        sendDeepSleep(deepSleepTime);
    }

    if(failCounter != 0)
    {
        if(timeSinceFailure == 0)
        {
            timeSinceFailure = millis();
        }
        else if (failCounter >= 3 || millis() - timeSinceFailure >= timeToSleepSinceFailure)
        {
            Serial.println(failCounter);
            failCounter = 0;
            timeSinceFailure = 0;
            deepSleepTime = 10000;
            goToSleep = false;
            deepSleep = true;
            sendDeepSleep(deepSleepTime);
        }
    }
}