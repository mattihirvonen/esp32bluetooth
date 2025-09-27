
//
// Enable only one of following example codes
//
#define EXAMPLE_SSP         0  // ESP32:   Tested to work
#define EXAMPLE_BLE_SERVER  1  // ESP32S3: Fixed to work
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

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLEServer         *pServer  = NULL;
BLEService        *pService = NULL;
BLECharacteristic *pCharacteristic;
int                connectedClients;


class MyCallbacks : public BLECharacteristicCallbacks
{
  void onConnect(BLECharacteristic *pCharacteristic) {
    connectedClients += 1;
    Serial.println("*******");
    Serial.println("Connect");
    Serial.println("*******");
  }

  void onDisconnect(BLECharacteristic *pCharacteristic) {
    connectedClients -= 1;
    Serial.println("*********");
    Serial.println("Disonnect");
    Serial.println("*********");
  }

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

    // All three methods work ok
    #if 0
    // Reply is uint16 value (note byteorder swap)
    uint16_t replyvalue = 1234;
    pCharacteristic->setValue( replyvalue );
    #endif
    #if 0
    const char replyvalue[] = "Reply Message";
    pCharacteristic->setValue(replyvalue);
    #endif
    #if 0
    // Use this method to reply:
    const char replyvalue[] = "Reply Message";
    pCharacteristic->setValue( (uint8_t*) replyvalue, (size_t) (strlen(replyvalue)+1) );
    #endif

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
  
  pServer  = BLEDevice::createServer();
  pService = pServer->createService(SERVICE_UUID);

  pCharacteristic =
    pService->createCharacteristic(CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ    | BLECharacteristic::PROPERTY_WRITE |
                                                        BLECharacteristic::PROPERTY_NOTIFY  | BLECharacteristic::PROPERTY_INDICATE);

  pCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();

  pinMode (LED_BUILTIN, OUTPUT | INPUT);
  digitalWrite (LED_BUILTIN, LOW);
}


// Notify send first 20 databytes
// Notify is push and forget type operation 
void notify_Characteristics( void )
{
  static int counter;
  char       buffer[64];

  if ( connectedClients ) {
    snprintf(buffer, sizeof(buffer), "notify: %d", ++counter);
    pCharacteristic->setValue( (uint8_t*) buffer, strlen(buffer)+1);
    pCharacteristic->notify();
    Serial.println(buffer);
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

  if ( ledstate ) {
    ledstate = LOW;   // turn the LED off by making the voltage 
  }
  else {
    ledstate = HIGH; // turn the LED on (HIGH is the voltage level)
    notify_Characteristics();
  }
  digitalWrite(LED_BUILTIN, ledstate);
}

#endif // EXAMPLE_BLE_SERVER

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
