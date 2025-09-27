
//
// Enable only one of following example codes
//
#define EXAMPLE_SSP         0
#define EXAMPLE_BLE_SERVER  1
#define EXAMPLE_BLE         0

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

#if EXAMPLE_BLE_SERVER

#include "Arduino.h"  // This include missing from original example code

/*
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleWrite.cpp
    Ported to Arduino ESP32 by Evandro Copercini
*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

class MyCallbacks : public BLECharacteristicCallbacks {
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
};

void setup() {
  delay (3000);            // Give VScode+PlatformIO time to open monitor terminal window's COM port
  Serial.begin (115200);
//Serial.println (string (MACHINETYPE " (") + string ((int) ESP.getCpuFreqMHz ()) + (char *) " MHz) " HOSTNAME " SDK: " + ESP.getSdkVersion () + (char *) " " VERSION_OF_SERVERS " compiled at: " __DATE__ " " __TIME__);

  Serial.println("1- Download and install an BLE scanner app in your phone");
  Serial.println("2- Scan for BLE devices in the app");
  Serial.println("3- Connect to MyESP32");
  Serial.println("4- Go to CUSTOM CHARACTERISTIC in CUSTOM SERVICE and write something");
  Serial.println("5- See the magic =)");

  BLEDevice::init("MyESP32");
  BLEServer *pServer = BLEDevice::createServer();

  BLEService *pService = pServer->createService(SERVICE_UUID);

  BLECharacteristic *pCharacteristic =
    pService->createCharacteristic(CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

  pCharacteristic->setCallbacks(new MyCallbacks());

  pCharacteristic->setValue("Hello World");
  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(2000);
}

#endif // EXAMPLE_BLE_SERVER

//========================================================================================
// https://jiayi0111.github.io/ESP32_S3-C3-extension-board-/Bluetooth/Bluetooth.html

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
