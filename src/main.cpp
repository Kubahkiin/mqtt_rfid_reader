#include <Arduino.h>
#include <ETH.h>
#include <Network.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <cstring>
#include <rfid_secrets.h>

constexpr const char* ETHERNET_HOSTNAME = "esp32-p4-rfid";
constexpr uint32_t ETHERNET_DHCP_TIMEOUT_MS = 15000;
constexpr const char* MQTT_RFID_DATA_TOPIC =
    "esp32-p4-nano/rfid_reader/fast_switch/data";
constexpr const char* MQTT_RFID_TEMPERATURE_TOPIC =
    "esp32-p4-nano/rfid_reader/temperature";
constexpr uint16_t MQTT_BUFFER_SIZE = 24576;

constexpr int RFID_RS232_TX_PIN = 32;
constexpr int RFID_RS232_RX_PIN = 33;
constexpr uint32_t RFID_RS232_BAUD = 115200;
constexpr uint32_t RFID_RESPONSE_TIMEOUT_MS = 1000;
constexpr uint32_t RFID_INVENTORY_TIMEOUT_MS = 4000;
constexpr uint32_t RFID_TEMPERATURE_INTERVAL_MS = 10000;

// parametry sesji inwentaryzacji.
constexpr uint8_t RFID_INVENTORY_SESSION_COUNT = 1;
constexpr uint32_t RFID_INVENTORY_SESSION_DURATION_MS = 7500;
constexpr uint32_t RFID_INVENTORY_PAUSE_MS = 5000;
constexpr uint8_t RFID_FAST_SWITCH_REPEAT = 10;

constexpr uint8_t RFID_FRAME_HEADER = 0xA0;
constexpr uint8_t RFID_PUBLIC_ADDRESS = 0xFF;
constexpr uint8_t RFID_COMMAND_SUCCESS = 0x10;

constexpr uint8_t RFID_CMD_RESET = 0x70;
constexpr uint8_t RFID_CMD_GET_FIRMWARE_VERSION = 0x72;
constexpr uint8_t RFID_CMD_SET_WORK_ANTENNA = 0x74;
constexpr uint8_t RFID_CMD_GET_OUTPUT_POWER = 0x77;
constexpr uint8_t RFID_CMD_SET_OUTPUT_POWER = 0x76;
constexpr uint8_t RFID_CMD_SET_FREQUENCY_REGION = 0x78;
constexpr uint8_t RFID_CMD_SET_BEEPER_MODE = 0x7A;
constexpr uint8_t RFID_CMD_GET_READER_TEMPERATURE = 0x7B;
constexpr uint8_t RFID_CMD_GET_RF_PORT_RETURN_LOSS = 0x7E;
constexpr uint8_t RFID_CMD_GET_READER_IDENTIFIER = 0x68;
constexpr uint8_t RFID_CMD_SET_ANTENNA_GROUP = 0x6C;
constexpr uint8_t RFID_CMD_GET_ANTENNA_GROUP = 0x6D;
constexpr uint8_t RFID_CMD_FAST_SWITCH_INVENTORY = 0x8A;

constexpr uint8_t RFID_ANTENNA_GROUP_COUNT = 2;
constexpr uint8_t RFID_ANTENNAS_PER_GROUP = 8;
constexpr uint8_t RFID_ANTENNA_COUNT = 16;
constexpr uint8_t RFID_OUTPUT_POWER_DBM = 33;
constexpr uint8_t RFID_ANTENNA_MIN_RETURN_LOSS_DB = 6;
constexpr uint8_t RFID_RETURN_LOSS_FREQUENCY = 0x06;
constexpr int8_t RFID_MAX_TEMPERATURE_C = 65;

constexpr size_t RFID_MAX_FRAME_LENGTH = 96;
constexpr size_t RFID_MAX_UNIQUE_TAGS = 128;
constexpr size_t RFID_MAX_EPC_LENGTH = 62;

struct RfidAntenna {
  uint8_t group;
  uint8_t port;
  uint8_t number;
  uint8_t returnLossDb;
};

struct RfidTag {
  uint8_t epc[RFID_MAX_EPC_LENGTH];
  uint8_t epcLength;
  uint16_t antennaMask;
  uint32_t readCount;
};

HardwareSerial rfidSerial(1);
RfidAntenna rfidDetectedAntennas[RFID_ANTENNA_COUNT];
uint8_t rfidDetectedAntennaCount = 0;
RfidTag rfidUniqueTags[RFID_MAX_UNIQUE_TAGS];
size_t rfidUniqueTagCount = 0;
uint8_t rfidReaderIdentifier[12] = {};
bool rfidReaderReady = false;
bool rfidTemperatureSafe = true;
uint8_t rfidCompletedInventorySessions = 0;
uint32_t rfidLastInventorySessionEndMs = 0;
uint32_t rfidLastTemperatureCheckMs = 0;
bool rfidInventoryRequested = false;
bool rfidInventoryRunning = false;
bool rfidInventoryDataPending = false;
int8_t rfidLastTemperatureC = 0;
bool rfidTemperaturePublishPending = false;
volatile bool ethernetOnline = false;
WiFiClientSecure espClient; // Secure connection init
PubSubClient client(espClient);

constexpr const char* clientID = "ESP32-P4-NANO";

// Message buffer
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];


void onEthernetEvent(arduino_event_id_t event, arduino_event_info_t info);
bool startEthernet();
void reconnect();
void publishMessage(const char* topic, String payload, boolean retained);
// Call back Method for Receiving MQTT messages and starting inventory session
void callback(char* topic, byte* payload, unsigned int length);
uint8_t calculateRfidChecksum(const uint8_t* data, size_t length);
void printRfidHexByte(uint8_t value);
void printRfidHex(const uint8_t* data, size_t length);
size_t buildRfidCommand(uint8_t command, const uint8_t* payload, size_t payloadLength,
                        uint8_t* frame, size_t capacity);
bool readRfidFrame(uint8_t* frame, size_t capacity, size_t& receivedLength,
                   uint32_t timeoutMs);
void clearRfidInput();
bool exchangeRfidCommand(uint8_t command, const uint8_t* payload, size_t payloadLength,
                         uint8_t* response, size_t& responseLength,
                         const char* description, bool logFrames = true);
bool isRfidCommandSuccess(const uint8_t* response, size_t responseLength,
                          uint8_t command);
bool sendRfidSetting(uint8_t command, const uint8_t* payload, size_t payloadLength,
                     const char* description, bool logFrames = true);
bool resetRfidReader();
bool readRfidFirmwareVersion();
bool readRfidIdentifier();
bool readRfidTemperature();
bool configureRfidOutputPower();
bool setRfidAntennaGroup(uint8_t group, bool logFrames = true);
bool verifyRfidAntennaGroup(uint8_t expectedGroup, bool logFrames = true);
bool readRfidReturnLoss(uint8_t& returnLossDb);
bool scanRfidAntennas();
uint8_t collectRfidPortsForGroup(uint8_t group, uint8_t* ports);
void collectRfidTag(const uint8_t* frame, size_t length, uint8_t group);
bool runRfidFastSwitchInventory(uint8_t group, const uint8_t* ports,
                                uint8_t portCount);
void printRfidInventorySummary();
bool publishRfidInventoryData();
void runRfidInventorySession();
bool configureRfidReader();

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\nStart systemu RFID E710");

  startEthernet();

  espClient.setCACert(root_ca);

  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);
  if (!client.setBufferSize(MQTT_BUFFER_SIZE)) {
    Serial.println("[MQTT] Nie mozna przydzielic bufora na raport RFID");
  }

  rfidSerial.begin(RFID_RS232_BAUD, SERIAL_8N1,
                   RFID_RS232_RX_PIN, RFID_RS232_TX_PIN);
  delay(100);

  if (!configureRfidReader()) {
    Serial.println("[RFID] Konfiguracja nie powiodla sie - system zatrzymany");
    return;
  }
  if (!scanRfidAntennas()) {
    Serial.println("[RFID] Nie wykryto gotowego zestawu anten");
    return;
  }

  rfidReaderReady = true;
  rfidLastTemperatureCheckMs = millis();
  rfidLastInventorySessionEndMs = millis() - RFID_INVENTORY_PAUSE_MS;
  Serial.println("[RFID] Czytnik gotowy do inwentaryzacji");
}

void loop() {
  if (!client.connected())
    reconnect();
  
  client.loop();

  if (rfidTemperaturePublishPending && client.connected()) {
    const String payload(rfidLastTemperatureC);
    if (client.publish(MQTT_RFID_TEMPERATURE_TOPIC, payload.c_str(), false)) {
      rfidTemperaturePublishPending = false;
      Serial.print("[MQTT] Wyslano temperature na ");
      Serial.print(MQTT_RFID_TEMPERATURE_TOPIC);
      Serial.print(": ");
      Serial.print(payload);
      Serial.println(" C");
    } else {
      Serial.println("[MQTT] Nie udalo sie wyslac temperatury czytnika");
    }
  }

  if (rfidInventoryDataPending && client.connected() &&
      publishRfidInventoryData()) {
    rfidInventoryDataPending = false;
  }

  if (!rfidReaderReady) {
    delay(1000);
    Serial.println("[RFID] Czytnik nie został skonfigurowany poprawnie");
    return;
  }

  const uint32_t now = millis();
  if (now - rfidLastTemperatureCheckMs >= RFID_TEMPERATURE_INTERVAL_MS) {
    rfidLastTemperatureCheckMs = now;
    if (!readRfidTemperature()) {
      Serial.println("[RFID] Nie mozna sprawdzic temperatury - pomijam cykl");
      return;
    }
  }
  if (!rfidTemperatureSafe ||
      now - rfidLastInventorySessionEndMs < RFID_INVENTORY_PAUSE_MS) {
    delay(10);
    return;
  }
}

void onEthernetEvent(arduino_event_id_t event, arduino_event_info_t info) {
  (void)info;

  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("[ETH] Kontroler uruchomiony");
      ETH.setHostname(ETHERNET_HOSTNAME);
      break;

    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.print("[ETH] Kabel podlaczony, predkosc: ");
      Serial.print(ETH.linkSpeed());
      Serial.print(" Mb/s, ");
      Serial.println(ETH.fullDuplex() ? "full duplex" : "half duplex");
      break;

    case ARDUINO_EVENT_ETH_GOT_IP:
      ethernetOnline = true;
      Serial.println("[ETH] Polaczenie z siecia gotowe (DHCP)");
      Serial.print("[ETH] Hostname: ");
      Serial.println(ETHERNET_HOSTNAME);
      Serial.print("[ETH] MAC: ");
      Serial.println(ETH.macAddress());
      Serial.print("[ETH] IP: ");
      Serial.println(ETH.localIP());
      Serial.print("[ETH] Brama: ");
      Serial.println(ETH.gatewayIP());
      Serial.print("[ETH] Maska: ");
      Serial.println(ETH.subnetMask());
      Serial.print("[ETH] DNS: ");
      Serial.println(ETH.dnsIP());
      break;

    case ARDUINO_EVENT_ETH_LOST_IP:
      ethernetOnline = false;
      Serial.println("[ETH] Utracono adres IP");
      break;

    case ARDUINO_EVENT_ETH_DISCONNECTED:
      ethernetOnline = false;
      Serial.println("[ETH] Kabel odlaczony lub brak polaczenia");
      break;

    case ARDUINO_EVENT_ETH_STOP:
      ethernetOnline = false;
      Serial.println("[ETH] Kontroler zatrzymany");
      break;

    default:
      break;
  }
}

bool startEthernet() {
  ethernetOnline = false;
  Network.onEvent(onEthernetEvent);

  Serial.println("[ETH] Uruchamianie Ethernetu i oczekiwanie na DHCP");
  if (!ETH.begin()) {
    Serial.println("[ETH] Nie mozna uruchomic kontrolera Ethernet");
    return false;
  }

  const uint32_t startedAt = millis();
  while (!ethernetOnline && millis() - startedAt < ETHERNET_DHCP_TIMEOUT_MS) {
    delay(100);
  }

  if (!ethernetOnline) {
    Serial.println("[ETH] Brak adresu IP. RFID bedzie dzialac bez sieci; Ethernet polaczy sie automatycznie pozniej.");
    return false;
  }

  return true;
}

void reconnect() {
  // loop until connected
  while(!client.connected()) {
    Serial.print("\nConnecting to broker");
    if (client.connect(clientID, mqtt_username, mqtt_password)) {
      Serial.print("\nConnected");

      // subscribe the topics here
      client.subscribe("esp32-p4-nano/rfid_reader/fast_switch/start");
      client.subscribe("esp32-p4-nano/rfid_reader/reset");
         
    }
    else {
      Serial.print("\nfailed, rc=");
      Serial.print(client.state());
      Serial.print("\nretrying in 5 seconds");
      delay(5000);
    }
  }
}

void publishMessage(const char* topic, String payload, boolean retained) {
  if(client.publish(topic, payload.c_str(), true)) {
    Serial.print("\nMessage published [" + String(topic) + "]: " + payload);
  }
}

/***** Call back Method for Receiving MQTT messages and Switching LED ****/

void callback(char* topic, byte* payload, unsigned int length) {
  constexpr const char* startTopic =
      "esp32-p4-nano/rfid_reader/fast_switch/start";
  constexpr const char* resetTopic =
      "esp32-p4-nano/rfid_reader/reset";

  Serial.print("[MQTT] Wiadomosc [");
  Serial.print(topic);
  Serial.print("]: ");
  for (unsigned int i = 0; i < length; ++i) {
    Serial.write(payload[i]);
  }
  Serial.println();

  //obsluga tematu start
  if (strcmp(topic, startTopic) == 0 && length == 1 && payload[0] == '1') {
    Serial.println("[MQTT] Przyjeto zadanie inwentaryzacji");

    rfidCompletedInventorySessions = 0;
    runRfidInventorySession();
  }

  //obsluga tematu reset
  if (strcmp(topic, resetTopic) == 0 && length == 1 && payload[0] == '1') {
    Serial.println("[MQTT] Przyjeto zadanie restartu czytnika");
    resetRfidReader();
  }
}

uint8_t calculateRfidChecksum(const uint8_t* data, size_t length) {
  uint8_t sum = 0;
  for (size_t i = 0; i < length; ++i) {
    sum = static_cast<uint8_t>(sum + data[i]);
  }
  return static_cast<uint8_t>(~sum + 1);
}

void printRfidHexByte(uint8_t value) {
  if (value < 0x10) {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}

void printRfidHex(const uint8_t* data, size_t length) {
  for (size_t i = 0; i < length; ++i) {
    printRfidHexByte(data[i]);
    if (i + 1 < length) {
      Serial.print(' ');
    }
  }
  Serial.println();
}

size_t buildRfidCommand(uint8_t command, const uint8_t* payload, size_t payloadLength,
                        uint8_t* frame, size_t capacity) {
  const size_t frameLength = payloadLength + 5;
  if (frameLength > capacity || payloadLength > 252) {
    return 0;
  }

  frame[0] = RFID_FRAME_HEADER;
  frame[1] = static_cast<uint8_t>(payloadLength + 3);
  frame[2] = RFID_PUBLIC_ADDRESS;
  frame[3] = command;
  if (payloadLength > 0) {
    memcpy(&frame[4], payload, payloadLength);
  }
  frame[frameLength - 1] = calculateRfidChecksum(frame, frameLength - 1);
  return frameLength;
}

bool readRfidFrame(uint8_t* frame, size_t capacity, size_t& receivedLength,
                   uint32_t timeoutMs) {
  const uint32_t startedAt = millis();
  receivedLength = 0;

  while (millis() - startedAt < timeoutMs) {
    if (!rfidSerial.available()) {
      delay(1);
      continue;
    }

    const uint8_t value = static_cast<uint8_t>(rfidSerial.read());
    if (receivedLength == 0 && value != RFID_FRAME_HEADER) {
      continue;
    }
    if (receivedLength >= capacity) {
      return false;
    }
    frame[receivedLength++] = value;

    if (receivedLength >= 2) {
      const size_t expectedLength = static_cast<size_t>(frame[1]) + 2;
      if (expectedLength < 5 || expectedLength > capacity) {
        return false;
      }
      if (receivedLength == expectedLength) {
        return calculateRfidChecksum(frame, receivedLength - 1) == frame[receivedLength - 1];
      }
    }
  }
  return false;
}

void clearRfidInput() {
  while (rfidSerial.available()) {
    rfidSerial.read();
  }
}

bool exchangeRfidCommand(uint8_t command, const uint8_t* payload, size_t payloadLength,
                         uint8_t* response, size_t& responseLength,
                         const char* description, bool logFrames) {
  uint8_t request[RFID_MAX_FRAME_LENGTH];
  const size_t requestLength = buildRfidCommand(command, payload, payloadLength,
                                                request, sizeof(request));
  if (requestLength == 0) {
    Serial.print("[RFID] Nie mozna zbudowac komendy: ");
    Serial.println(description);
    return false;
  }

  clearRfidInput();
  if (logFrames) {
    Serial.print("[RFID] ");
    Serial.print(description);
    Serial.print(" TX: ");
    printRfidHex(request, requestLength);
  }
  rfidSerial.write(request, requestLength);
  rfidSerial.flush();

  if (!readRfidFrame(response, RFID_MAX_FRAME_LENGTH, responseLength,
                     RFID_RESPONSE_TIMEOUT_MS)) {
    if (logFrames) {
      Serial.print("[RFID] ");
      Serial.print(description);
      Serial.println(" - brak poprawnej odpowiedzi");
    }
    return false;
  }

  if (logFrames) {
    Serial.print("[RFID] ");
    Serial.print(description);
    Serial.print(" RX: ");
    printRfidHex(response, responseLength);
  }
  return responseLength >= 5 && response[3] == command;
}

bool isRfidCommandSuccess(const uint8_t* response, size_t responseLength,
                          uint8_t command) {
  return responseLength == 6 && response[3] == command &&
         response[4] == RFID_COMMAND_SUCCESS;
}

bool sendRfidSetting(uint8_t command, const uint8_t* payload, size_t payloadLength,
                     const char* description, bool logFrames) {
  uint8_t response[RFID_MAX_FRAME_LENGTH];
  size_t responseLength = 0;
  if (!exchangeRfidCommand(command, payload, payloadLength, response,
                           responseLength, description, logFrames)) {
    return false;
  }
  if (!isRfidCommandSuccess(response, responseLength, command)) {
    Serial.print("[RFID] Blad komendy ");
    Serial.print(description);
    Serial.print(", kod: 0x");
    if (responseLength > 4) {
      printRfidHexByte(response[4]);
    } else {
      Serial.print("??");
    }
    Serial.println();
    return false;
  }
  return true;
}

bool resetRfidReader() {
  uint8_t response[RFID_MAX_FRAME_LENGTH];
  size_t responseLength = 0;
  if(!exchangeRfidCommand(RFID_CMD_RESET, nullptr, 0, response, responseLength, "reset czytnika") || 
      responseLength > 0) { // udany reset nie powinien zwrocic zadnej odpowiedzi
        return false;
      }

  return true;
}

bool readRfidFirmwareVersion() {
  uint8_t response[RFID_MAX_FRAME_LENGTH];
  size_t responseLength = 0;
  if (!exchangeRfidCommand(RFID_CMD_GET_FIRMWARE_VERSION, nullptr, 0,
                           response, responseLength, "wersja firmware") ||
      responseLength != 7) {
    return false;
  }

  Serial.print("[RFID] Wersja firmware: ");
  printRfidHexByte(response[4]);
  Serial.print('.');
  printRfidHexByte(response[5]);
  Serial.println();
  return true;
}

bool readRfidIdentifier() {
  uint8_t response[RFID_MAX_FRAME_LENGTH];
  size_t responseLength = 0;
  if (!exchangeRfidCommand(RFID_CMD_GET_READER_IDENTIFIER, nullptr, 0,
                           response, responseLength, "identyfikator czytnika") ||
      responseLength != 17) {
    return false;
  }

  memcpy(rfidReaderIdentifier, &response[4], sizeof(rfidReaderIdentifier));
  Serial.print("[RFID] Identyfikator: ");
  printRfidHex(rfidReaderIdentifier, sizeof(rfidReaderIdentifier));
  return true;
}

bool readRfidTemperature() {
  uint8_t response[RFID_MAX_FRAME_LENGTH];
  size_t responseLength = 0;
  if (!exchangeRfidCommand(RFID_CMD_GET_READER_TEMPERATURE, nullptr, 0,
                           response, responseLength, "temperatura", false) ||
      responseLength != 7) {
    return false;
  }

  const int8_t temperature = response[4] == 0x00
                                 ? -static_cast<int8_t>(response[5])
                                 : static_cast<int8_t>(response[5]);
  Serial.print("[RFID] Temperatura: ");
  Serial.print(temperature);
  Serial.println(" C");
  rfidLastTemperatureC = temperature;
  rfidTemperaturePublishPending = true;
  if (client.connected()) {
    const String payload(temperature);
    if (client.publish(MQTT_RFID_TEMPERATURE_TOPIC, payload.c_str(), false)) {
      rfidTemperaturePublishPending = false;
      Serial.print("[MQTT] Wyslano temperature na ");
      Serial.print(MQTT_RFID_TEMPERATURE_TOPIC);
      Serial.print(": ");
      Serial.print(payload);
      Serial.println(" C");
    } else {
      Serial.println("[MQTT] Nie udalo sie wyslac temperatury czytnika");
    }
  }
  rfidTemperatureSafe = temperature < RFID_MAX_TEMPERATURE_C;
  if (!rfidTemperatureSafe) {
    Serial.println("[RFID] UWAGA: temperatura >= 65 C, inwentaryzacja wstrzymana");
  }
  return true;
}

bool configureRfidOutputPower() {
  uint8_t response[RFID_MAX_FRAME_LENGTH];
  size_t responseLength = 0;
  if (!exchangeRfidCommand(RFID_CMD_GET_OUTPUT_POWER, nullptr, 0,
                           response, responseLength, "odczyt mocy") ||
      responseLength < 6) {
    return false;
  }

  bool powerAlreadyConfigured = true;
  for (size_t i = 4; i + 1 < responseLength; ++i) {
    if (response[i] != RFID_OUTPUT_POWER_DBM) {
      powerAlreadyConfigured = false;
      break;
    }
  }
  if (powerAlreadyConfigured) {
    Serial.println("[RFID] Moc jest juz ustawiona na 30 dBm");
    return true;
  }

  const uint8_t power = RFID_OUTPUT_POWER_DBM;
  Serial.println("[RFID] Zmiana mocy na 30 dBm (zapis w flash czytnika)");
  return sendRfidSetting(RFID_CMD_SET_OUTPUT_POWER, &power, 1, "ustawienie mocy");
}

bool setRfidAntennaGroup(uint8_t group, bool logFrames) {
  if (group >= RFID_ANTENNA_GROUP_COUNT) {
    return false;
  }
  return sendRfidSetting(RFID_CMD_SET_ANTENNA_GROUP, &group, 1,
                         "ustawienie grupy anten", logFrames);
}

bool verifyRfidAntennaGroup(uint8_t expectedGroup, bool logFrames) {
  uint8_t response[RFID_MAX_FRAME_LENGTH];
  size_t responseLength = 0;
  if (!exchangeRfidCommand(RFID_CMD_GET_ANTENNA_GROUP, nullptr, 0,
                           response, responseLength, "odczyt grupy anten", logFrames) ||
      responseLength != 6) {
    return false;
  }
  if (response[4] != expectedGroup) {
    Serial.print("[RFID] Nieoczekiwana grupa: ");
    Serial.println(response[4]);
    return false;
  }
  return true;
}

bool readRfidReturnLoss(uint8_t& returnLossDb) {
  const uint8_t frequency = RFID_RETURN_LOSS_FREQUENCY;
  uint8_t response[RFID_MAX_FRAME_LENGTH];
  size_t responseLength = 0;
  if (!exchangeRfidCommand(RFID_CMD_GET_RF_PORT_RETURN_LOSS, &frequency, 1,
                           response, responseLength, "return loss") ||
      responseLength != 6) {
    return false;
  }
  returnLossDb = response[4];
  return returnLossDb != 0xEE;
}

bool scanRfidAntennas() {
  rfidDetectedAntennaCount = 0;
  bool scanCompleted = true;
  Serial.println("\n[RFID] Identyfikacja podlaczonych anten");

  for (uint8_t group = 0; group < RFID_ANTENNA_GROUP_COUNT; ++group) {
    Serial.print("[RFID] Grupa ");
    Serial.println(group);
    if (!setRfidAntennaGroup(group) || !verifyRfidAntennaGroup(group)) {
      scanCompleted = false;
      continue;
    }

    for (uint8_t port = 0; port < RFID_ANTENNAS_PER_GROUP; ++port) {
      if (!sendRfidSetting(RFID_CMD_SET_WORK_ANTENNA, &port, 1,
                           "wybor portu anteny")) {
        scanCompleted = false;
        continue;
      }

      uint8_t returnLossDb = 0;
      const uint8_t antennaNumber = group * RFID_ANTENNAS_PER_GROUP + port + 1;
      Serial.print("[RFID] Antena ");
      Serial.print(antennaNumber);
      Serial.print(": ");
      if (!readRfidReturnLoss(returnLossDb)) {
        Serial.println("pomiar nieudany");
        scanCompleted = false;
        continue;
      }

      Serial.print(returnLossDb);
      Serial.print(" dB - ");
      if (returnLossDb >= RFID_ANTENNA_MIN_RETURN_LOSS_DB) {
        Serial.println("podlaczona");
        rfidDetectedAntennas[rfidDetectedAntennaCount++] = {
            group, port, antennaNumber, returnLossDb};
      } else {
        Serial.println("brak / slabe dopasowanie");
      }
    }
  }

  Serial.print("[RFID] Wykryte anteny:");
  for (uint8_t i = 0; i < rfidDetectedAntennaCount; ++i) {
    Serial.print(' ');
    Serial.print(rfidDetectedAntennas[i].number);
  }
  if (rfidDetectedAntennaCount == 0) {
    Serial.print(" brak");
  }
  Serial.println();
  return scanCompleted && rfidDetectedAntennaCount > 0;
}

uint8_t collectRfidPortsForGroup(uint8_t group, uint8_t* ports) {
  uint8_t count = 0;
  for (uint8_t i = 0; i < rfidDetectedAntennaCount; ++i) {
    if (rfidDetectedAntennas[i].group == group) {
      ports[count++] = rfidDetectedAntennas[i].port;
    }
  }
  return count;
}

void collectRfidTag(const uint8_t* frame, size_t length, uint8_t group) {
  if (length < 9) {
    return;
  }
  const uint8_t frequencyAndAntenna = frame[4];
  const uint8_t rawRssi = frame[length - 2];
  const uint8_t port = static_cast<uint8_t>((frequencyAndAntenna & 0x03) +
                                             ((rawRssi & 0x80) ? 4 : 0));
  const uint8_t antennaNumber = group * RFID_ANTENNAS_PER_GROUP + port + 1;
  const size_t epcLength = length - 9;
  if (epcLength == 0 || epcLength > RFID_MAX_EPC_LENGTH) {
    return;
  }

  for (size_t i = 0; i < rfidUniqueTagCount; ++i) {
    if (rfidUniqueTags[i].epcLength == epcLength &&
        memcmp(rfidUniqueTags[i].epc, &frame[7], epcLength) == 0) {
      rfidUniqueTags[i].antennaMask |= static_cast<uint16_t>(1U << (antennaNumber - 1));
      ++rfidUniqueTags[i].readCount;
      return;
    }
  }

  if (rfidUniqueTagCount >= RFID_MAX_UNIQUE_TAGS) {
    return;
  }
  RfidTag& tag = rfidUniqueTags[rfidUniqueTagCount++];
  memcpy(tag.epc, &frame[7], epcLength);
  tag.epcLength = static_cast<uint8_t>(epcLength);
  tag.antennaMask = static_cast<uint16_t>(1U << (antennaNumber - 1));
  tag.readCount = 1;
}

bool runRfidFastSwitchInventory(uint8_t group, const uint8_t* ports,
                                uint8_t portCount) {
  if (portCount == 0) {
    return true;
  }
  if (!setRfidAntennaGroup(group, false) || !verifyRfidAntennaGroup(group, false)) {
    return false;
  }

  uint8_t payload[29] = {};
  for (uint8_t slot = 0; slot < RFID_ANTENNAS_PER_GROUP; ++slot) {
    payload[slot * 2] = slot < portCount ? ports[slot] : 0xFF;
    payload[slot * 2 + 1] = slot < portCount ? 1 : 0;
  }
  payload[16] = 0;  // Interval
  // payload[17..21] = Reserve0
  payload[22] = 1;  // Session S1
  payload[23] = 0;  // Target A
  // payload[24..26] = Reserve1..3
  payload[27] = 0;  // Bez pomiaru fazy
  payload[28] = RFID_FAST_SWITCH_REPEAT;

  uint8_t request[RFID_MAX_FRAME_LENGTH];
  const size_t requestLength = buildRfidCommand(RFID_CMD_FAST_SWITCH_INVENTORY,
                                                payload, sizeof(payload),
                                                request, sizeof(request));
  clearRfidInput();
  rfidSerial.write(request, requestLength);
  rfidSerial.flush();

  const uint32_t startedAt = millis();
  while (millis() - startedAt < RFID_INVENTORY_TIMEOUT_MS) {
    uint8_t response[RFID_MAX_FRAME_LENGTH];
    size_t responseLength = 0;
    const uint32_t remaining = RFID_INVENTORY_TIMEOUT_MS - (millis() - startedAt);
    if (!readRfidFrame(response, sizeof(response), responseLength, remaining)) {
      Serial.println("[RFID] Fast switch - brak ramki koncowej");
      return false;
    }
    if (response[3] != RFID_CMD_FAST_SWITCH_INVENTORY) {
      continue;
    }
    if (responseLength == 12) {
      const uint32_t totalRead = (static_cast<uint32_t>(response[4]) << 16) |
                                 (static_cast<uint32_t>(response[5]) << 8) |
                                 response[6];
      const uint32_t duration = (static_cast<uint32_t>(response[7]) << 24) |
                                (static_cast<uint32_t>(response[8]) << 16) |
                                (static_cast<uint32_t>(response[9]) << 8) |
                                response[10];
      (void)totalRead;
      (void)duration;
      return true;
    }
    if (responseLength == 6) {
      Serial.print("[RFID] Fast switch - blad 0x");
      printRfidHexByte(response[4]);
      Serial.println();
      return false;
    }
    if (responseLength == 7 && response[5] == 0x22) {
      Serial.println("[RFID] Czytnik zglosil brak anteny");
      continue;
    }
    collectRfidTag(response, responseLength, group);
  }
  return false;
}

void printRfidInventorySummary() {
  Serial.print("\n[RFID] Anteny wykorzystane w inwentaryzacji:");
  for (uint8_t i = 0; i < rfidDetectedAntennaCount; ++i) {
    Serial.print(' ');
    Serial.print(rfidDetectedAntennas[i].number);
  }
  Serial.println();

  Serial.print("[RFID] Liczba unikalnych tagow: ");
  Serial.println(rfidUniqueTagCount);
  for (size_t i = 0; i < rfidUniqueTagCount; ++i) {
    Serial.print("[TAG ");
    Serial.print(i + 1);
    Serial.print("] EPC=");
    for (uint8_t j = 0; j < rfidUniqueTags[i].epcLength; ++j) {
      printRfidHexByte(rfidUniqueTags[i].epc[j]);
    }
    Serial.print(" anteny=");
    bool firstAntenna = true;
    for (uint8_t antenna = 0; antenna < RFID_ANTENNA_COUNT; ++antenna) {
      if (rfidUniqueTags[i].antennaMask & static_cast<uint16_t>(1U << antenna)) {
        if (!firstAntenna) {
          Serial.print(',');
        }
        Serial.print(antenna + 1);
        firstAntenna = false;
      }
    }
    Serial.print(" odczyty=");
    Serial.println(rfidUniqueTags[i].readCount);
  }
}

bool publishRfidInventoryData() {
  if (!client.connected()) {
    return false;
  }

  JsonDocument document;
  document["tag_count"] = rfidUniqueTagCount;

  JsonArray usedAntennas = document["used_antennas"].to<JsonArray>();
  for (uint8_t i = 0; i < rfidDetectedAntennaCount; ++i) {
    usedAntennas.add(rfidDetectedAntennas[i].number);
  }

  JsonArray tags = document["tags"].to<JsonArray>();
  constexpr char hexDigits[] = "0123456789ABCDEF";
  for (size_t i = 0; i < rfidUniqueTagCount; ++i) {
    JsonObject tag = tags.add<JsonObject>();

    char epc[RFID_MAX_EPC_LENGTH * 2 + 1];
    for (uint8_t j = 0; j < rfidUniqueTags[i].epcLength; ++j) {
      epc[j * 2] = hexDigits[rfidUniqueTags[i].epc[j] >> 4];
      epc[j * 2 + 1] = hexDigits[rfidUniqueTags[i].epc[j] & 0x0F];
    }
    epc[rfidUniqueTags[i].epcLength * 2] = '\0';

    tag["epc"] = epc;
    tag["read_count"] = rfidUniqueTags[i].readCount;
    JsonArray tagAntennas = tag["antennas"].to<JsonArray>();
    for (uint8_t antenna = 0; antenna < RFID_ANTENNA_COUNT; ++antenna) {
      if (rfidUniqueTags[i].antennaMask & static_cast<uint16_t>(1U << antenna)) {
        tagAntennas.add(antenna + 1);
      }
    }
  }

  String jsonPayload;
  serializeJson(document, jsonPayload);
  if (jsonPayload.length() + 1 > MQTT_BUFFER_SIZE) {
    Serial.print("[MQTT] Raport RFID jest za duzy: ");
    Serial.print(jsonPayload.length());
    Serial.println(" bajtow");
    return false;
  }

  if (!client.publish(MQTT_RFID_DATA_TOPIC, jsonPayload.c_str(), false)) {
    Serial.println("[MQTT] Nie udalo sie wyslac raportu RFID");
    return false;
  }

  Serial.print("[MQTT] Wyslano raport RFID na ");
  Serial.print(MQTT_RFID_DATA_TOPIC);
  Serial.print(" (");
  Serial.print(jsonPayload.length());
  Serial.println(" bajtow)");
  return true;
}

void runRfidInventorySession() {
  const uint8_t sessionNumber = rfidCompletedInventorySessions + 1;
  const uint32_t sessionStartedAt = millis();

  Serial.print("\n[RFID] Start sesji inwentaryzacji ");
  Serial.print(sessionNumber);
  Serial.print('/');
  Serial.print(RFID_INVENTORY_SESSION_COUNT);
  Serial.print(" (czas ");
  Serial.print(RFID_INVENTORY_SESSION_DURATION_MS / 1000);
  Serial.println(" s)");

  rfidUniqueTagCount = 0;

  while (millis() - sessionStartedAt < RFID_INVENTORY_SESSION_DURATION_MS) {
    for (uint8_t group = 0; group < RFID_ANTENNA_GROUP_COUNT; ++group) {
      uint8_t ports[RFID_ANTENNAS_PER_GROUP];
      const uint8_t portCount = collectRfidPortsForGroup(group, ports);
      if (!runRfidFastSwitchInventory(group, ports, portCount)) {
        Serial.print("[RFID] Inwentaryzacja grupy ");
        Serial.print(group);
        Serial.println(" nie powiodla sie");
      }

      if (millis() - sessionStartedAt >= RFID_INVENTORY_SESSION_DURATION_MS) {
        break;
      }
    }
  }

  ++rfidCompletedInventorySessions;
  rfidLastInventorySessionEndMs = millis();
  Serial.print("[RFID] Koniec sesji ");
  Serial.println(sessionNumber);
  printRfidInventorySummary();
  rfidInventoryDataPending = true;
  if (client.connected() && publishRfidInventoryData()) {
    rfidInventoryDataPending = false;
  }

  if (rfidCompletedInventorySessions >= RFID_INVENTORY_SESSION_COUNT) {
    Serial.println("[RFID] Wszystkie zaplanowane sesje zakonczone. Skanowanie zatrzymane.");
  } else {
    Serial.print("[RFID] Przerwa przed kolejna sesja: ");
    Serial.print(RFID_INVENTORY_PAUSE_MS / 1000);
    Serial.println(" s");
  }
}

bool configureRfidReader() {
  Serial.println("\n[RFID] Konfiguracja czytnika");
  if (!readRfidFirmwareVersion()) {
    return false;
  }

  bool configured = true;
  configured = readRfidIdentifier() && configured;
  configured = readRfidTemperature() && configured;

  const uint8_t quietBeeper = 0x00;
  configured = sendRfidSetting(RFID_CMD_SET_BEEPER_MODE, &quietBeeper, 1,
                               "wyciszenie buzzera") && configured;
  configured = configureRfidOutputPower() && configured;

  const uint8_t etsi865To868[] = {0x02, 0x00, 0x06};
  configured = sendRfidSetting(RFID_CMD_SET_FREQUENCY_REGION,
                               etsi865To868, sizeof(etsi865To868),
                               "ETSI 865-868 MHz") && configured;
  return configured;
}
