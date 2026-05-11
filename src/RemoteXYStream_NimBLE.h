#ifndef RemoteXYStream_NimBLE_h
#define RemoteXYStream_NimBLE_h


#include "RemoteXYNet.h"

#define RemoteXYNet_BLEDEVICE__SEND_BUFFER_SIZE 20
#define RemoteXYNet_BLEDEVICE__RECEIVE_BUFFER_SIZE 1024
#define RemoteXYNet_BLEDEVICE__SERVICE_UUID             "0000FFE0-0000-1000-8000-00805F9B34FB" // UART service UUID
#define RemoteXYNet_BLEDEVICE__CHARACTERISTIC_UUID_RXTX "0000FFE1-0000-1000-8000-00805F9B34FB"

#define RemoteXYNet_BLEDEVICE__SEND_TIME_FOR_ONE_PACKAGE 7  // test min 5 ms
#define RemoteXYNet_BLEDEVICE__SEND_BYTES_BEFORE_OVERFLOW 340  // test max 350 bytes for ESP32S3


class CRemoteXYStream_NimBLE :
  public CRemoteXYStream,
  public NimBLEServerCallbacks,
  public NimBLECharacteristicCallbacks {
  
  protected:
  const char * bleDeviceName;    // need to delete

  NimBLEServer *pServer;
  NimBLECharacteristic * pRxTxCharacteristic;

  uint8_t sendBuffer[RemoteXYNet_BLEDEVICE__SEND_BUFFER_SIZE];
  uint16_t sendBufferCount;
  
  uint16_t sendCount;
  uint8_t sendBLEBufferOverflow;
  uint32_t lastNotifyMillis;
  
  uint8_t receiveBuffer[RemoteXYNet_BLEDEVICE__RECEIVE_BUFFER_SIZE];
  uint16_t receiveBufferStart;
  uint16_t receiveBufferPos;
  uint16_t receiveBufferCount;
  
  volatile uint8_t receiveBufferLook; 
    
    
  public:  
  CRemoteXYStream_NimBLE(const char * _bleDeviceName) : CRemoteXYStream () {
    bleDeviceName = _bleDeviceName;

    receiveBufferLook = 0;         
    receiveBufferCount = 0;        
    receiveBufferStart = 0;
    receiveBufferPos = 0;
    receiveBufferCount = 0;  
    
    sendBufferCount = 0;     
	lastNotifyMillis = 0;
        
#if defined(REMOTEXY__DEBUGLOG)
    RemoteXYDebugLog.write(F("Init ESP32 BLE on chip"));
#endif
	
	if (!NimBLEDevice::isInitialized()) {
		// Create the BLE Device
		NimBLEDevice::init(_bleDeviceName);
	}
	// Create the BLE Server
	pServer = NimBLEDevice::createServer();
	
    pServer->setCallbacks(this);

    // Create the BLE Service
    NimBLEService *pService = pServer->createService(RemoteXYNet_BLEDEVICE__SERVICE_UUID);

    // Create a BLE Characteristic
    pRxTxCharacteristic = pService->createCharacteristic(
                            RemoteXYNet_BLEDEVICE__CHARACTERISTIC_UUID_RXTX,
							NIMBLE_PROPERTY::READ |
							NIMBLE_PROPERTY::NOTIFY |
							NIMBLE_PROPERTY::WRITE_NR
                          );
       
    pRxTxCharacteristic->setCallbacks(this);

    // Start the service
    pService->start();

    // Start advertising

NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();

pAdvertising->stop();
pAdvertising->clearData();

pAdvertising->setName(bleDeviceName);
pAdvertising->addServiceUUID(RemoteXYNet_BLEDEVICE__SERVICE_UUID);
pAdvertising->start();

#if defined(REMOTEXY__DEBUGLOG)
    RemoteXYDebugLog.write(F("BLE started"));
#endif    

  }
                    
void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
#if defined(REMOTEXY__DEBUGLOG)
  RemoteXYDebugLog.write(F("BLE client connected"));
#endif
  receiveBufferStart = 0;
  receiveBufferPos = 0;
  receiveBufferCount = 0;
  sendCount = 0;
  sendBLEBufferOverflow = 0;
}

void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
#if defined(REMOTEXY__DEBUGLOG)
  RemoteXYDebugLog.write(F("BLE client disconnected"));
#endif

  receiveBufferCount = 0;

  NimBLEDevice::startAdvertising();
}

void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo& connInfo) override {
  std::string rxValue = pCharacteristic->getValue();

  if (rxValue.length() > 0) {
    while (receiveBufferLook != 0) { delay(1); }
    receiveBufferLook = 1;

    for (uint16_t i = 0; i < rxValue.length(); i++) {
      uint8_t b = (uint8_t)rxValue[i];
      receiveBuffer[receiveBufferPos++] = b;
      if (receiveBufferPos >= RemoteXYNet_BLEDEVICE__RECEIVE_BUFFER_SIZE) receiveBufferPos = 0;
      if (receiveBufferCount < RemoteXYNet_BLEDEVICE__RECEIVE_BUFFER_SIZE) receiveBufferCount++;
      else {
        receiveBufferStart++;
        if (receiveBufferStart >= RemoteXYNet_BLEDEVICE__RECEIVE_BUFFER_SIZE) receiveBufferStart = 0;
      }
    }

    receiveBufferLook = 0;
  }
}
      
  
	void write(uint8_t b) override {
	  while (sendBufferCount >= RemoteXYNet_BLEDEVICE__SEND_BUFFER_SIZE) {
		_flush();
		yield();
	  }

	  sendBuffer[sendBufferCount++] = b;

	  if (sendBufferCount == RemoteXYNet_BLEDEVICE__SEND_BUFFER_SIZE) {
		_flush();
	  }
	}
  
  void _flush () {
    // the ESP BLE library have some error
    // it has some sent buffer which can overflow after 600 bytes if sending bytes without delay
    // as a result of the tests, it was found that 5 ms per 20 bytes are needed
    // but if send without delay, it can send up to 600 bytes
  
	  if (sendBufferCount == 0) return;

	  if (millis() - lastNotifyMillis < RemoteXYNet_BLEDEVICE__SEND_TIME_FOR_ONE_PACKAGE) {
		return;
	  }

	  lastNotifyMillis = millis();

	  pRxTxCharacteristic->setValue((uint8_t *)sendBuffer, sendBufferCount);
	  pRxTxCharacteristic->notify();

	  sendCount += sendBufferCount;
	  sendBufferCount = 0;
	
	
  }  
  
	void flush () override {
	  while (sendBufferCount > 0) {
		_flush();
		yield(); // ou delay(0) sur ESP32
	  }

	  sendCount = 0;
	  sendBLEBufferOverflow = 0;
	} 
  
  void handler () override {   
    if (receiveBufferCount>0) {
      while (receiveBufferLook!=0) { delay(1); } 
      receiveBufferLook=1; 
      while (receiveBufferCount) {     
        notifyReadByteListener (receiveBuffer[receiveBufferStart++]);
        if (receiveBufferStart >= RemoteXYNet_BLEDEVICE__RECEIVE_BUFFER_SIZE) receiveBufferStart=0;
        receiveBufferCount--;
      }
      receiveBufferLook=0;
    }
  }
  

  
};

#endif // RemoteXYStream_NimBLE_h