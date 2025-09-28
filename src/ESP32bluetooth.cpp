
// NOTE(s):
// - ESP32   have older classic BT
// - ESP32S3 have only BLE
// - Enable only one of following example codes
//
#define EXAMPLE_SSP         0  // ESP32:   Tested to work with Bluetooth Serial Terminal
#define EXAMPLE_BLE_SERVER  1  // ESP32S3: Fixed to  work with LightBlue
#define EAXMPLE_BLE_UART    0  // ESP32S3: Fixed to  work with LightBlue and Bluetooth Serial Terminal
#define EXAMPLE_BLE         0  // ESP32S3: Not work properly ???

//========================================================================================
// https://microcontrollerslab.com/esp32-bluetooth-classic-arduino-ide/

#if EXAMPLE_SSP

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
}

void loop() {

  #if 1
  // Modified - Echo: loop back
  if (SerialBT.available()) {
    SerialBT.write(SerialBT.read());
  }

  #else

  // Original demo - Gateway: Bluetooth <-> SerialPort
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  if (SerialBT.available()) {
    Serial.write(SerialBT.read());
  }
  #endif
  delay(20);
}

#endif // EXAMPLE_SSP

//========================================================================================
// https://github.com/espressif/arduino-esp32/blob/master/libraries/BLE/examples/Write/Write.ino
// https://github.com/espressif/arduino-esp32/blob/master/libraries/BLE/examples/Server_multiconnect/Server_multiconnect.ino
//
//  \Users\mattihirvonen\.platformio\packages\framework-espressif32\libraries\BLE\src
//
// Possible use/case strategy:
// - get  "command" with onWrite() call back (here data example "read:xyz");
// - wait onRead() callback to return "onWrite() selected" item (example "data:xyz=1234")
//
// See also methods: notify and indicate

#if 0
/**
 * @brief Set the current value.
 */
void BLEValue::setValue(std::string value) {
	m_value = value;
} // setValue


/**
 * @brief Set the current value.
 * @param [in] pData The data for the current value.
 * @param [in] The length of the new current value.
 */
void BLEValue::setValue(uint8_t* pData, size_t length) {
	m_value = std::string((char*) pData, length);
} // setValue
#endif

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

#if EXAMPLE_BLE_SERVER

#include "Arduino.h"     // This include missing from original example code
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID            "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID     "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_TX  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"


BLEServer         *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
int                connectedClients;
bool               deviceConnected = false;
bool               oldDeviceConnected = false;


class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    connectedClients += 1;
    deviceConnected   = true;
    Serial.println("****************");
    Serial.println("Device connected");
    Serial.println("****************");
  };

  void onDisconnect(BLEServer *pServer) {
    connectedClients -= 1;
    deviceConnected   = false;
    Serial.println("*******************");
    Serial.println("Device disconnected");
    Serial.println("*******************");
  }
};


class MyCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic) {
    String value = pCharacteristic->getValue().c_str();  // Note: String conversion problem in orginal demo code!

    if (value.length() > 0) {
      Serial.println("*********");
      Serial.print("New value: ");
      for (int i = 0; i < value.length(); i++) {
        Serial.print(value[i]);
      }
      Serial.println();
      Serial.println("*********");
    }
  }

  void onRead(BLECharacteristic *pCharacteristic) {
    static int counter;
    char replyvalue[64];
  
    snprintf(replyvalue, sizeof(replyvalue), "counter: %d", ++counter);
    pCharacteristic->setValue( (uint8_t*) replyvalue, (size_t) (strlen(replyvalue)+1) );
  }
};


void setup()
{
  delay (3000);            // Give VScode+PlatformIO time to open monitor terminal window's COM port
  Serial.begin (115200);
//Serial.println (string (MACHINETYPE " (") + string ((int) ESP.getCpuFreqMHz ()) + (char *) " MHz) " HOSTNAME " SDK: " + ESP.getSdkVersion () + (char *) " " VERSION_OF_SERVERS " compiled at: " __DATE__ " " __TIME__);

  Serial.println("1- Download and install an BLE scanner app in your phone");
  Serial.println("2- Scan for BLE devices in the app");
  Serial.println("3- Connect to MyESP32");
  Serial.println("4- Go to CUSTOM CHARACTERISTIC in CUSTOM SERVICE and write something");
  Serial.println("5- See the magic =)");

  // Create the BLE Device
  BLEDevice::init("MyESP32");
  
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);

  // Descriptor 2902 is not required when using NimBLE as it is automatically added based on the characteristic properties
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pCharacteristic =
    pService->createCharacteristic(CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

  pCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();

  pinMode (LED_BUILTIN, OUTPUT | INPUT);
  digitalWrite (LED_BUILTIN, LOW);
}


// Notify send first 20 databytes
// Notify is push and forget type operation 
void notify_TxCharacteristics( void )
{
  static int counter;
  char       buffer[64];

  if ( connectedClients ) {
    snprintf(buffer, sizeof(buffer), "notify: %d", ++counter);
    pTxCharacteristic->setValue( (uint8_t*) buffer, strlen(buffer)+1);
    pTxCharacteristic->notify();
    Serial.println(buffer);
  }
}


void check_connect( void )
{
  #if 0
  if (deviceConnected) {
    Serial.print("Notifying Value: ");
    Serial.println(txValue);
    pTxCharacteristic->setValue(&txValue, 1);
    pTxCharacteristic->notify();
    txValue++;
  //delay(1000);  // Notifying every 1 second
  }
  #endif

  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);                   // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising();  // restart advertising
    Serial.println("Started advertising again...");
    oldDeviceConnected = false;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = true;
  }
}


void loop()
{
  static TickType_t  xLastWakeTime, xNowTime;
  static int         ledstate;

  // put your main code here, to run repeatedly:

  xNowTime = xTaskGetTickCount();
  if ( (int32_t)(xNowTime - xLastWakeTime) < 1000 ) {
    return;
  }
  xLastWakeTime = xNowTime;

  check_connect();
  
  if ( ledstate ) {
    ledstate = LOW;   // turn the LED off by making the voltage 
  }
  else {
    ledstate = HIGH; // turn the LED on (HIGH is the voltage level)
    notify_TxCharacteristics();
  }
  digitalWrite(LED_BUILTIN, ledstate);
}

#endif // EXAMPLE_BLE_SERVER

//========================================================================================
//https://github.com/espressif/arduino-esp32/blob/master/libraries/BLE/examples/UART/UART.ino

#if EAXMPLE_BLE_UART

/*
    Video: https://www.youtube.com/watch?v=oCMOYS71NIU
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
    Ported to Arduino ESP32 by Evandro Copercini

   Create a BLE server that, once we receive a connection, will send periodic notifications.
   The service advertises itself as: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
   Has a characteristic of: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E - used for receiving data with "WRITE"
   Has a characteristic of: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E - used to send data with  "NOTIFY"

   The design of creating the BLE server is:
   1. Create a BLE Server
   2. Create a BLE Service
   3. Create a BLE Characteristic on the Service
   4. Create a BLE Descriptor on the characteristic
   5. Start the service.
   6. Start advertising.

   In this example rxValue is the data received (only accessible inside that function).
   And txValue is the data to be sent, in this example just a byte incremented every second.
*/
#include "Arduino.h"     // This include missing from original example code
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"  // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    Serial.println("Device connected");
  };

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    Serial.println("Device disconnected");
  }
};

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String rxValue = pCharacteristic->getValue().c_str();   // NOTE conversion problem in Orginal dem0

    if (rxValue.length() > 0) {
      Serial.println("*********");
      Serial.print("Received Value: ");
      for (int i = 0; i < rxValue.length(); i++) {
        Serial.print(rxValue[i]);
      }

      Serial.println();
      Serial.println("*********");
    }
  }
};

void setup() {
  Serial.begin(115200);

  // Create the BLE Device
  BLEDevice::init("UART Service");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);

  // Descriptor 2902 is not required when using NimBLE as it is automatically added based on the characteristic properties
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}

void loop() {

  if (deviceConnected) {
    Serial.print("Notifying Value: ");
    Serial.println(txValue);
    pTxCharacteristic->setValue(&txValue, 1);
    pTxCharacteristic->notify();
    txValue++;
    delay(1000);  // Notifying every 1 second
  }

  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);                   // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising();  // restart advertising
    Serial.println("Started advertising again...");
    oldDeviceConnected = false;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = true;
  }
}

#endif // EXAMPLE_BLE_UART

//========================================================================================
// https://jiayi0111.github.io/ESP32_S3-C3-extension-board-/Bluetooth/Bluetooth.html
//
// Building require add to pletform.ini:
// lib_deps = avinabmalla/ESP32_BleSerial@^2.0.1

#if EXAMPLE_BLE

#include "Arduino.h"
#include <string>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLECharacteristic* pCharacteristic;
bool deviceConnected = false;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("BLE Client Connected");
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("BLE Client Disconnected");
  }
};

void setup() {
  Serial.begin(115200);
  pinMode(2, OUTPUT);

  BLEDevice::init("ESP32_BLE_Device");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService("180F");  // Battery Service UUID
  pCharacteristic = pService->createCharacteristic(
                      "2A19",  // Battery Level characteristic
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_WRITE
                    );

  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setValue("Hello from ESP32!");
  pService->start();

  pServer->getAdvertising()->start();
  Serial.println("BLE advertising started!");
}

void loop() {
  if (deviceConnected) {
    pCharacteristic->setValue("Updated at " + String(millis()));
    pCharacteristic->notify();
    delay(2000);
  }
}

#endif // EXAMPLE BLE

//========================================================================================
