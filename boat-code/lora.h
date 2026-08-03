#include <Arduino.h>
#include "Timer.h"

class lora {
  private:
    struct __attribute__((packed)) dataFormat {
      uint32_t time;      
      float temp; 
      float ph;
      double lat;
      double lon;        
      bool atWaypoint;    
      uint8_t checksum; 
    };

    HardwareSerial *loraSerial;
    Timer *mTimer;

    bool newTxData = false;
    bool newData = false;

    dataFormat data;
    const uint8_t startMarker = 0xAA; // 170
    const uint8_t dataLen = sizeof(dataFormat);

    unsigned long prevUpdateTime = 0;
    unsigned long updateInterval = 500;

    // checks the data is the correct length, and therefore that the start marker isn't accidentally occuring in the data, which could make the code v confusing
    uint8_t calculateChecksum(dataFormat &d) {
      uint8_t sum = 0;
      uint8_t* ptr = reinterpret_cast<uint8_t*>(&d);
      // Sum everything except the checksum byte itself
      for (uint8_t i = 0; i < offsetof(dataFormat, checksum); i++) {
        sum += ptr[i];
      }
      return sum;
    }

  public:
    // Pass Serial1, Serial2, etc.
    lora(HardwareSerial &serialPort, Timer& t_);

    void updateDataToSend(float temp, float ph, double lat, double lon, bool atWaypoint);
    void transmitData();
    void recvWithChecksum();
    void showNewData();
};

lora::lora(HardwareSerial &serialPort, Timer& t_) {
  loraSerial = &serialPort;
  loraSerial->begin(9600); 
  mTimer = &t_;
}

void lora::updateDataToSend(float temp, float ph, double lat, double lon, bool atWaypoint) {
    if (millis() - prevUpdateTime >= updateInterval) {
        prevUpdateTime = millis();
        data.time = mTimer->read();
        data.temp = temp;
        data.ph = ph;
        data.lat = lat;
        data.lon = lon;
        data.atWaypoint = atWaypoint;
        data.checksum = calculateChecksum(data); // Compute before sending
        newTxData = true;
    }
}

void lora::transmitData() {
  if(newTxData) {
    loraSerial->write(startMarker);
    loraSerial->write((uint8_t*) &data, dataLen);
    newTxData = false;
  }
}

void lora::recvWithChecksum() {
    // We need at least the marker + the struct
    if (loraSerial->available() > dataLen && newData == false) {
        if (loraSerial->read() == startMarker) {
            // Potential packet found, read it into a temp buffer
            dataFormat tempBuffer;
            uint8_t* ptr = reinterpret_cast<uint8_t*>(&tempBuffer);
            
            for (uint8_t i = 0; i < dataLen; i++) {
                ptr[i] = loraSerial->read();
            }

            // Verify checksum
            if (tempBuffer.checksum == calculateChecksum(tempBuffer)) { //checks data is correctly sent, and didn't get corrupted
                data = tempBuffer; // Valid data, copy to main struct
                newData = true;
            }
        }
    }
}

void lora::showNewData() {
      if (newData == true) {
            Serial.print(data.time);
            Serial.print(' ');
            Serial.print(data.temp);
            Serial.print(' ');
            Serial.print(data.ph);
            Serial.print(' ');
            Serial.print(data.lat);
            Serial.print(' ');
            Serial.print(data.lon);
            Serial.print(' ');
            Serial.println(data.atWaypoint);
        newData = false;
      }
}