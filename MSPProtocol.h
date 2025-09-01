#ifndef MSP_PROTOCOL_H
#define MSP_PROTOCOL_H

#include <Arduino.h>
#include <HardwareSerial.h>

class MSPProtocol {
public:
  MSPProtocol();
  void begin();
  void update();
  
  float getHeight() const { return lastHeight_m; }
  uint8_t getRangeQuality() const { return lastRangeQuality; }
  uint8_t getFlowQuality() const { return lastFlowQuality; }
  uint32_t getRangeAge() const { return millis() - lastRangeMs; }
  uint32_t getFlowAge() const { return millis() - lastFlowMs; }
  float getVelocityX() const { return vx_mps; }
  float getVelocityY() const { return vy_mps; }
  uint32_t getBytesReceived() const { return bytesSeen; }
  
private:
  HardwareSerial MSPSerial;
  
  // MSP constants
  static const uint16_t MSP2_SENSOR_RANGEFINDER = 0x1F01;
  static const uint16_t MSP2_SENSOR_OPTIC_FLOW = 0x1F02;
  static const uint8_t MSP_V1_HDR2 = 'M';
  static const uint8_t MSP_V2_HDR2 = 'X';
  static const uint8_t MSP_V1_FUNC_V2_FRAME = 255;
  
  // Sensor state
  volatile uint32_t lastFlowMs;
  volatile uint32_t lastRangeMs;
  float lastHeight_m;
  uint8_t lastRangeQuality;
  uint8_t lastFlowQuality;
  float vx_mps, vy_mps;
  uint32_t bytesSeen;
  
  // MSP parsing structures
  enum V2State { V2_WAIT_$, V2_WAIT_X, V2_DIR, V2_FLAGS, V2_FUNC_L, 
                 V2_FUNC_H, V2_LEN_L, V2_LEN_H, V2_PAYLOAD, V2_CRC };
  
  struct MSPv2 {
    V2State st = V2_WAIT_$;
    uint8_t dir=0, flags=0, crc=0;
    uint16_t func=0, len=0, idx=0;
    static const uint16_t MAXP = 64;
    uint8_t pay[MAXP];
  } v2;
  
  enum V1State { V1_WAIT_$, V1_WAIT_M, V1_DIR, V1_LEN, V1_CMD, V1_PAY, V1_CK };
  
  struct MSPv1 {
    V1State st = V1_WAIT_$;
    uint8_t dir=0, len=0, cmd=0, idx=0, ck=0;
    static const uint8_t MAXP = 64;
    uint8_t pay[MAXP];
  } v1;
  
  // Methods
  void mspSendV2(uint16_t func);
  void mspSendV1(uint8_t func);
  void feedByte(uint8_t b);
  void v2Feed(uint8_t b);
  void v2Reset();
  void v1Feed(uint8_t b);
  void v1Reset();
  void onOpticalFlow(const uint8_t* p, uint16_t n);
  void onRangefinder(const uint8_t* p, uint16_t n);
  
  static inline uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a);
};

#endif