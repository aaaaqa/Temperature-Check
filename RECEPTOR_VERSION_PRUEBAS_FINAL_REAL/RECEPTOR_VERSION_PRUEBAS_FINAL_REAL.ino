#include <esp_now.h>
#include <WiFi.h>

int initialNumber = 0;
char modeType;      // 'E' o 'C'
char directionType; // 'I' o 'D'
int expectedColumns;

// Lista de direcciones MAC permitidas
const uint8_t allowedMacs[][6] = {
  {0X34, 0XCD, 0XB0, 0X8D, 0XA3, 0XDC},
  {0X38, 0X18, 0X2B, 0X50, 0XC6, 0X60},
  {0X34, 0XCD, 0XB0, 0X8C, 0XF7, 0XDC},
  {0X34, 0XCD, 0XB0, 0X8C, 0XF7, 0XE4},
  {0X34, 0XCD, 0XB0, 0X8D, 0XA3, 0XCC},
};
const int numAllowedMacs = sizeof(allowedMacs) / sizeof(allowedMacs[0]);

uint8_t macAddress[6];

#define MAX_SKIP 50
int numbersToSkip[MAX_SKIP];
int skipCount = 0;

int row, column;

typedef struct response_message {
  int initialNumber;
} response_message;

bool isAllowedMac(const uint8_t *mac_addr) {
  for (int i = 0; i < numAllowedMacs; i++) {
    if (memcmp(mac_addr, allowedMacs[i], 6) == 0) return true;
  }
  return false;
}

void getMac() {
    String mac = WiFi.macAddress();
    
    sscanf(mac.c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", 
    &macAddress[0], &macAddress[1], &macAddress[2], &macAddress[3], &macAddress[4], &macAddress[5]);
}

bool shouldSkip(int number) {
  for (int i = 0; i < skipCount; i++) {
    if (numbersToSkip[i] == number) return true;
  }
  return false;
}

void parseSerialCommand(String input) {
  skipCount = 0;
  input.trim();

  // Debe tener al menos 4 chars: X$Y$

  if (input == 'R') {
    for(int i = 0; i < numAllowedMacs; i++)
      {
        esp_now_send(allowedMacs[i], (uint8_t *)macAddress, sizeof(macAddress));
        delay(500);
      }
  }
  else {
    if (input.length() < 4 || input.charAt(1) != '$' || input.charAt(3) != '$') {
      return;
    }
    char t0 = input.charAt(0);
    char t1 = input.charAt(2);

    if ((t0 == 'E' || t0 == 'C') && (t1 == 'I' || t1 == 'D')) {
    modeType = t0;
    directionType = t1;
    expectedColumns = (modeType == 'E') ? 2 : 3;

    // inicializa initialNumber según I o D
    if (directionType == 'I') {
      initialNumber = 50;
    } else {
      initialNumber = 0;
    }

    // Parsear valores omitidos tras el segundo '$'
    String rest = input.substring(4);
    while (rest.length() > 0) {
      int sep = rest.indexOf('$');
      String token;
      if (sep != -1) {
        token = rest.substring(0, sep);
        rest = rest.substring(sep + 1);
      } else {
        token = rest;
        rest = "";
      }
      int num = token.toInt();
      if (num > 0 && skipCount < MAX_SKIP) {
        numbersToSkip[skipCount++] = num;
      }
    }

    Serial.println("OK");
    }
  }
  
}

void onReceive(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  if (!isAllowedMac(mac_addr)) {
    Serial.println("MAC no autorizada");
    return;
  }

  if (data_len == 1 && data[0] == 0xFF) {
    // Ajusta initialNumber según direccion
    while (true) {
      int grupo = (expectedColumns == 2)
                  ? ((directionType=='I' ? (initialNumber - 1) : initialNumber) / expectedColumns) + 1
                  : ((directionType=='I' ? (initialNumber - 1) : initialNumber) / expectedColumns) + 1;
      if (!shouldSkip(grupo)) break;
      if (directionType == 'I') initialNumber--;
      else initialNumber++;
    }

    response_message sendData = { initialNumber };
    esp_now_send(mac_addr, (uint8_t *)&sendData, sizeof(sendData));

    if (directionType == 'I') initialNumber--;
    else initialNumber++;

  } else if (data_len == sizeof(float)*2) {
    float numbersReceived[2];
    memcpy(numbersReceived, data, sizeof(numbersReceived));
    float randomNumber = numbersReceived[0];
    int recvNum = (int)numbersReceived[1];

    row = (recvNum / expectedColumns) + 1;
    column = (recvNum % expectedColumns) + 1;

    String msg = String(row) + "$" + String(column) + "$" + String(randomNumber,2);
    Serial.println(msg);
  }
}

void onSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Mensaje enviado a ESP1: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Éxito" : "Fallo");
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error init ESP-NOW");
    return;
  }

  for (int i = 0; i < numAllowedMacs; i++) {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, allowedMacs[i], 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
  }

  esp_now_register_send_cb(onSent);
  esp_now_register_recv_cb(onReceive);

  getMac();
}

void loop() {
  if (Serial.available()) {
    String in = Serial.readStringUntil('\n');
    parseSerialCommand(in);
  }
}