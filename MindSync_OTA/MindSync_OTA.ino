#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Update.h>

#ifdef _BLE_DEVICE_H_
  #error "Conflicting BLE library detected (possibly ArduinoBLE). Please remove it to proceed."
#endif

#define SAMPLE_RATE 256
#define INPUT_PIN1 4
#define INPUT_PIN2 16
// UUIDs for BLE services and characteristics
// #define UART_SERVICE_UUID      "00000001-0000-FEED-0000-000000000000"
// #define RX_CHARACTERISTIC_UUID "00000002-0000-FEED-0000-000000000000"
// #define TX_CHARACTERISTIC_UUID "00000003-0000-FEED-0000-000000000000"

#define UART_SERVICE_UUID      "12345678-1234-1234-1234-123456789ABC"
#define RX_CHARACTERISTIC_UUID "00000002-0000-FEED-0000-000000000000"
#define TX_CHARACTERISTIC_UUID "ABCDEF12-3456-7890-1234-567890ABCDEF"

#define UPDATE_SERVICE_UUID       "00000001-0000-C0DE-0000-000000000000"
#define UPDATE_CHARACTERISTIC_UUID "00000002-0000-C0DE-0000-000000000000"

// BLE server/characteristic pointers and connection state
BLEServer *pServer = NULL;
BLECharacteristic *pRxCharacteristic = NULL;
BLECharacteristic *pTxCharacteristic = NULL;
BLECharacteristic *pUpdateCharacteristic = NULL;
bool deviceConnected = false;

// OTA update variables
bool otaInProgress = false;
uint32_t otaFileSize = 0;
uint32_t otaReceived = 0;
int lastProgressPercent = -1;

// Counter and timing variables
int counter = 0;
unsigned long previousMillis = 0;
const long interval = 1000;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("[BLE] Device connected.");
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    // Cancel OTA update on disconnect
    if (otaInProgress) {
      Serial.println("[OTA] Update cancelled.");
      Update.end(false); // Abort the update process
      otaInProgress = false;
    }
    // Restart advertising after disconnecting
    pServer->getAdvertising()->start();
    Serial.println("[BLE] Device disconnected. Re-advertising...");
  }
};

class UartCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    uint8_t* data = pCharacteristic->getData();
    size_t length = pCharacteristic->getLength();

    if (length == 0) return;

    // Handle regular UART input
    Serial.print("[Console] ");
    Serial.write(data, length);
    Serial.println();
  }
};

class UpdateCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    uint8_t* data = pCharacteristic->getData();
    size_t length = pCharacteristic->getLength();

    if (length == 0) return;

    // Check for OTA start command "OPEN"
    if (!otaInProgress && length == 4 &&
        memcmp(data, "OPEN", 4) == 0) {
      Serial.println("[OTA] Update started.");
      otaInProgress = true;
      otaFileSize = 0;
      otaReceived = 0;
      lastProgressPercent = -1;
      return;
    }

    // Check for OTA cancel command "HALT" during an OTA update
    if (otaInProgress && length == 4 &&
        memcmp(data, "HALT", 4) == 0) {
      Serial.println("[OTA] Update cancelled.");
      Update.end(false); // Abort the update process
      otaInProgress = false;
      return;
    }

    if (otaInProgress) {
      // Handle file size reception
      if (otaFileSize == 0 && length == 4) {
        memcpy(&otaFileSize, data, 4);
        Serial.printf("[OTA] Update size: %u bytes.\n", otaFileSize);
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
          Serial.println("[OTA] ERROR: Unable to start update.");
          otaInProgress = false;
        }
        return;
      }

      // Check for OTA end command "DONE"
      if (length == 4 && memcmp(data, "DONE", 4) == 0) {
        Serial.println("[OTA] Finalizing update.");

        if (otaReceived != otaFileSize) {
          Serial.printf("[OTA] ERROR: Size mismatch (%u/%u bytes).\n", otaReceived, otaFileSize);
          Update.end(false); // Abort safely
        } else if (Update.end(true) && Update.isFinished()) {
          Serial.println("[OTA] Update successful.");
          Serial.println("[OTA] Rebooting...");
          ESP.restart();
        } else {
          Serial.println("[OTA] ERROR: Failed to finalize update.");
          Update.printError(Serial);
        }

        otaInProgress = false;
        return;
      }

      // Process binary data chunks
      if (otaReceived < otaFileSize) {
        size_t written = Update.write(data, length);
        if (written > 0) {
          otaReceived += written;
          int progress = (otaReceived * 100) / otaFileSize;
          // Print only if progress has advanced to a new integer value
          if (progress != lastProgressPercent) {
            lastProgressPercent = progress;
            Serial.printf("[OTA] Progress: %d%%\n", progress);
          }
        }
      }
    }
  }
};

void setup() {
  Serial.begin(115200);
  BLEDevice::init("MindSync");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create UART Service
  BLEService *pUartService = pServer->createService(UART_SERVICE_UUID);

  // Create RX Characteristic (write_nr)
  pRxCharacteristic = pUartService->createCharacteristic(
    RX_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE_NR
  );
  pRxCharacteristic->setCallbacks(new UartCallbacks());

  // Create TX Characteristic (read, notify)
  pTxCharacteristic = pUartService->createCharacteristic(
    TX_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pTxCharacteristic->addDescriptor(new BLE2902());

  // Create Update Service
  BLEService *pUpdateService = pServer->createService(UPDATE_SERVICE_UUID);

  // Create Update Characteristic (write_nr)
  pUpdateCharacteristic = pUpdateService->createCharacteristic(
    UPDATE_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE_NR
  );
  pUpdateCharacteristic->setCallbacks(new UpdateCallbacks());

  // Start both services
  pUartService->start();
  pUpdateService->start();

  // Start advertising with both service UUIDs
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(UART_SERVICE_UUID);
  pAdvertising->addServiceUUID(UPDATE_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("[BLE] Device initialized. Advertising...");
  analogReadResolution(12);
  analogSetPinAttenuation(INPUT_PIN1, ADC_11db);
  analogSetPinAttenuation(INPUT_PIN2, ADC_11db);

}

void loop() {
  if (deviceConnected && !otaInProgress) {
  static unsigned long past = 0;
  unsigned long present = micros();
  unsigned long interval = present - past;
  past = present;
  static long timer = 0;
  timer -= interval;

  if(timer < 0){
    timer += 1000000 / SAMPLE_RATE;
  
    float signal1 = ECGFilter1(analogRead(INPUT_PIN1));
    float signal2 = ECGFilter1(analogRead(INPUT_PIN2));
    float signal3 = random(500, 1000) / 10.0;
    float signal4 = random(500, 1000) / 10.0;
      char buffer[60];
      sprintf(buffer, "%.1f,%.1f,%.1f,%.1f", signal1, signal2, signal3, signal4);
      pTxCharacteristic->setValue(buffer);
      pTxCharacteristic->notify(); //software
  }
  }
}

float ECGFilter1(float input) {
  float output = input;
  {
    static float z1, z2;
    float x = output - 0.70682283*z1 - 0.15621030*z2;
    output = 0.28064917*x + 0.56129834*z1 + 0.28064917*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2;
    float x = output - 0.95028224*z1 - 0.54073140*z2;
    output = 1.00000000*x + 2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2;
    float x = output - -1.95360385*z1 - 0.95423412*z2;
    output = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2;
    float x = output - -1.98048558*z1 - 0.98111344*z2;
    output = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}