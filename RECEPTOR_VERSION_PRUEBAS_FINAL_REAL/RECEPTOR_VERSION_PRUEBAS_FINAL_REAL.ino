#include <esp_now.h>
#include <WiFi.h>

int initialNumber = 0;
char type;
int expectedColumns;

// Lista de direcciones MAC permitidas
const uint8_t allowedMacs[][6] = {
    {0x3C, 0x8A, 0x1F, 0x1F, 0x0D, 0x68},  // MAC de HT7833 1
    {0x3C, 0x8A, 0x1F, 0x30, 0xC5, 0x94},  // MAC de SPX3819 1
    {0x3C, 0x8A, 0x1F, 0x30, 0xA8, 0x04},  // MAC de TPS63021 1
    {0x3C, 0x8A, 0x1F, 0x1E, 0x66, 0x84},  // MAC de ADP124 1
    {0x3C, 0x8A, 0x1F, 0x30, 0x95, 0x98},  // MAC de LP5912 1
    {0x3C, 0x8A, 0x1F, 0x1E, 0x58, 0x4C},  // MAC de HT7833 2
    {0x3C, 0x8A, 0x1F, 0x30, 0x89, 0xB0},
    {0x34, 0xCD, 0xB0, 0x8C, 0xF7, 0xF4},
    {0x34, 0xCD, 0xB0, 0x8C, 0xF7, 0xE4},
};

uint8_t macAddress[6];

int row;
int column;

const int numAllowedMacs = sizeof(allowedMacs) / sizeof(allowedMacs[0]);

typedef struct response_message {
    int initialNumber;
} response_message;

void onSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("Mensaje enviado a ESP1: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Éxito" : "Fallo");
}

bool isAllowedMac(const uint8_t *mac_addr) {
    // Verifica si la MAC recibida está en la lista de direcciones MAC permitidas
    for (int i = 0; i < numAllowedMacs; i++) {
        bool match = true;
        for (int j = 0; j < 6; j++) {
            if (mac_addr[j] != allowedMacs[i][j]) {
                match = false;
                break;
            }
        }
        if (match) return true;
    }
    return false;
}

void onReceive(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
    if (isAllowedMac(mac_addr)) {
        if (data_len == 1 && data[0] == 0xFF) {
            // Mensaje inicial recibido (marcado con 0xFF)
            //Serial.println("Mensaje inicial recibido de un ESP1 autorizado. Enviando número de respuesta...");
            response_message sendData = {initialNumber++};
            
            // Envía el número incrementado a la MAC que envió el mensaje
            esp_now_send(mac_addr, (uint8_t *)&sendData, sizeof(sendData));
        } else if (data_len == sizeof(float) * 2) {
            // Mensaje de los dos números recibido de ESP1
            float numbersReceived[2];
            memcpy(numbersReceived, data, sizeof(numbersReceived));

            
            float randomNumber = numbersReceived[0];  // El número aleatorio
            int receivedInitialNumber = (int)numbersReceived[1];  // El número inicial

            row = (receivedInitialNumber / expectedColumns) + 1;
            column = (receivedInitialNumber % expectedColumns) + 1;

            // Imprime los números recibidos
            //Serial.print("Número aleatorio recibido: ");
            String message = String(row) + "$" + String(column) + "$" + String(randomNumber);
            //Serial.printf("%,2f", randomNumber); // Imprime con 2 decimales
            Serial.println(message);
            //Serial.print("Número inicial recibido: ");
            //Serial.println(receivedInitialNumber);
        }
    } else {
        Serial.println("Mensaje recibido de un dispositivo no autorizado. Ignorado.");
    }
}

void getMac()
{
    String mac = WiFi.macAddress();
    
    sscanf(mac.c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", 
    &macAddress[0], &macAddress[1], &macAddress[2], &macAddress[3], &macAddress[4], &macAddress[5]);
}

void setup() {
    Serial.begin(115200);

    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error al inicializar ESP-NOW");
        return;
    }

    // Agrega cada MAC de la lista como peer
    for (int i = 0; i < numAllowedMacs; i++) {
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, allowedMacs[i], 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;
        if (esp_now_add_peer(&peerInfo) != ESP_OK) {
            Serial.println("No se pudo agregar el peer de ESP1");
        }
    }

    getMac();
}

void loop() {
    type = Serial.read();
    if(type == 'E' || type == 'C')
    {
      expectedColumns = (type == 'E') ? 2 : 3;
      esp_now_register_send_cb(onSent);
      esp_now_register_recv_cb(onReceive);

      Serial.println("OK");
    }

    if(type == 'R')
    {
      for(int i = 0; i < numAllowedMacs; i++)
      {
        esp_now_send(allowedMacs[i], (uint8_t *)macAddress, sizeof(macAddress));
        delay(100);
      }
    }
}
