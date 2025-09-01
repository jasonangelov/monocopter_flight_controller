#include "MSPProtocol.h"
#include "Config.h"

static const float FLOW_FOV_DEG = 42.0f;
static const float FLOW_PIXELS = 35.0f;
static const float FOV_RAD = FLOW_FOV_DEG * (PI / 180.0f);

MSPProtocol::MSPProtocol() : MSPSerial(2) {
  lastFlowMs = 0;
  lastRangeMs = 0;
  lastHeight_m = 0.0f;
  lastRangeQuality = 0;
  lastFlowQuality = 0;
  vx_mps = 0.0f;
  vy_mps = 0.0f;
  bytesSeen = 0;
}

void MSPProtocol::begin() {
  MSPSerial.begin(115200, SERIAL_8N1, UART_RX, UART_TX);
  Serial.println(F("MSP protocol started on UART2"));
}

void MSPProtocol::update() {
  // Read incoming bytes
  while (MSPSerial.available()) {
    uint8_t b = (uint8_t)MSPSerial.read();
    feedByte(b);
  }
  
  // Periodic polling
  static uint32_t tRF = 0, tOF = 0, tProbe = 0;
  const uint32_t now = millis();
  
  if (now - tRF >= 20) {
    tRF = now;
    mspSendV2(MSP2_SENSOR_RANGEFINDER);
  }
  
  if (now - tOF >= 20) {
    tOF = now;
    mspSendV2(MSP2_SENSOR_OPTIC_FLOW);
  }
  
  if (now - tProbe >= 500) {
    tProbe = now;
    mspSendV1(1); // MSP_API_VERSION
  }
}

uint8_t MSPProtocol::crc8_dvb_s2(uint8_t crc, uint8_t a) {
  crc ^= a;
  for (int i = 0; i < 8; i++) {
    crc = (crc & 0x80) ? ((crc << 1) ^ 0xD5) : (crc << 1);
  }
  return crc;
}

void MSPProtocol::mspSendV2(uint16_t func) {
  uint8_t msg[9];
  msg[0] = '$';
  msg[1] = MSP_V2_HDR2;
  msg[2] = '<';
  msg[3] = 0x00;
  msg[4] = (uint8_t)(func & 0xFF);
  msg[5] = (uint8_t)(func >> 8);
  msg[6] = 0x00;
  msg[7] = 0x00;
  
  uint8_t crc = 0;
  for (int i = 3; i <= 7; i++) {
    crc = crc8_dvb_s2(crc, msg[i]);
  }
  msg[8] = crc;
  
  MSPSerial.write(msg, sizeof(msg));
}

void MSPProtocol::mspSendV1(uint8_t func) {
  uint8_t msg[6];
  msg[0] = '$';
  msg[1] = MSP_V1_HDR2;
  msg[2] = '<';
  msg[3] = 0;
  msg[4] = func;
  msg[5] = msg[3] ^ msg[4];
  
  MSPSerial.write(msg, sizeof(msg));
}

void MSPProtocol::feedByte(uint8_t b) {
  bytesSeen++;
  MSPv2 snapshot = v2;
  v2Feed(b);
  if (v2.st == V2_WAIT_$ && snapshot.st == V2_WAIT_$ && b != '$') {
    v1Feed(b);
  }
}

void MSPProtocol::v2Reset() {
  v2 = MSPv2();
}

void MSPProtocol::v2Feed(uint8_t b) {
  switch (v2.st) {
    case V2_WAIT_$:
      if (b == '$') v2.st = V2_WAIT_X;
      break;
    case V2_WAIT_X:
      if (b == MSP_V2_HDR2) v2.st = V2_DIR;
      else v2Reset();
      break;
    case V2_DIR:
      v2.dir = b;
      v2.st = V2_FLAGS;
      break;
    case V2_FLAGS:
      v2.flags = b;
      v2.crc = 0;
      v2.crc = crc8_dvb_s2(v2.crc, b);
      v2.st = V2_FUNC_L;
      break;
    case V2_FUNC_L:
      v2.func = b;
      v2.crc = crc8_dvb_s2(v2.crc, b);
      v2.st = V2_FUNC_H;
      break;
    case V2_FUNC_H:
      v2.func |= (uint16_t)b << 8;
      v2.crc = crc8_dvb_s2(v2.crc, b);
      v2.st = V2_LEN_L;
      break;
    case V2_LEN_L:
      v2.len = b;
      v2.crc = crc8_dvb_s2(v2.crc, b);
      v2.st = V2_LEN_H;
      break;
    case V2_LEN_H:
      v2.len |= (uint16_t)b << 8;
      v2.crc = crc8_dvb_s2(v2.crc, b);
      v2.idx = 0;
      if (v2.len > MSPv2::MAXP) {
        v2Reset();
        break;
      }
      v2.st = (v2.len ? V2_PAYLOAD : V2_CRC);
      break;
    case V2_PAYLOAD:
      v2.pay[v2.idx++] = b;
      v2.crc = crc8_dvb_s2(v2.crc, b);
      if (v2.idx >= v2.len) v2.st = V2_CRC;
      break;
    case V2_CRC:
      if (b == v2.crc) {
        if (v2.func == MSP2_SENSOR_OPTIC_FLOW) {
          onOpticalFlow(v2.pay, v2.len);
        } else if (v2.func == MSP2_SENSOR_RANGEFINDER) {
          onRangefinder(v2.pay, v2.len);
        }
      }
      v2Reset();
      break;
  }
}

void MSPProtocol::v1Reset() {
  v1 = MSPv1();
}

void MSPProtocol::v1Feed(uint8_t b) {
  switch (v1.st) {
    case V1_WAIT_$:
      if (b == '$') v1.st = V1_WAIT_M;
      break;
    case V1_WAIT_M:
      if (b == MSP_V1_HDR2) v1.st = V1_DIR;
      else v1Reset();
      break;
    case V1_DIR:
      v1.dir = b;
      v1.st = V1_LEN;
      break;
    case V1_LEN:
      v1.len = b;
      v1.ck = v1.len;
      v1.st = V1_CMD;
      break;
    case V1_CMD:
      v1.cmd = b;
      v1.ck ^= v1.cmd;
      v1.idx = 0;
      if (v1.len > MSPv1::MAXP) {
        v1Reset();
        break;
      }
      v1.st = (v1.len ? V1_PAY : V1_CK);
      break;
    case V1_PAY:
      v1.pay[v1.idx++] = b;
      v1.ck ^= b;
      if (v1.idx >= v1.len) v1.st = V1_CK;
      break;
    case V1_CK:
      if (b == v1.ck) {
        if (v1.cmd == MSP_V1_FUNC_V2_FRAME && v1.len >= 6) {
          uint8_t flags = v1.pay[0];
          uint16_t func = (uint16_t)v1.pay[1] | ((uint16_t)v1.pay[2] << 8);
          uint16_t plen = (uint16_t)v1.pay[3] | ((uint16_t)v1.pay[4] << 8);
          
          if (plen + 6 == v1.len) {
            uint8_t crc = 0;
            crc = crc8_dvb_s2(crc, flags);
            crc = crc8_dvb_s2(crc, (uint8_t)(func & 0xFF));
            crc = crc8_dvb_s2(crc, (uint8_t)(func >> 8));
            crc = crc8_dvb_s2(crc, (uint8_t)(plen & 0xFF));
            crc = crc8_dvb_s2(crc, (uint8_t)(plen >> 8));
            
            for (uint16_t i = 0; i < plen; i++) {
              crc = crc8_dvb_s2(crc, v1.pay[5 + i]);
            }
            
            if (crc == v1.pay[5 + plen]) {
              if (func == MSP2_SENSOR_OPTIC_FLOW) {
                onOpticalFlow(&v1.pay[5], plen);
              } else if (func == MSP2_SENSOR_RANGEFINDER) {
                onRangefinder(&v1.pay[5], plen);
              }
            }
          }
        }
      }
      v1Reset();
      break;
  }
}

void MSPProtocol::onRangefinder(const uint8_t* p, uint16_t n) {
  if (n >= 5) {
    lastRangeQuality = p[0];
    int32_t dist_mm = (int32_t)((uint32_t)p[1] | 
                                 ((uint32_t)p[2] << 8) | 
                                 ((uint32_t)p[3] << 16) | 
                                 ((uint32_t)p[4] << 24));
    if (dist_mm > 0) {
      lastHeight_m = dist_mm / 1000.0f;
      lastRangeMs = millis();
    }
  } else if (n == 2) {
    uint16_t dist_mm = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
    lastRangeQuality = 255;
    lastHeight_m = dist_mm / 1000.0f;
    lastRangeMs = millis();
  }
}

void MSPProtocol::onOpticalFlow(const uint8_t* p, uint16_t n) {
  int32_t dx = 0, dy = 0;
  uint8_t qual = 0;
  
  if (n >= 9) {
    qual = p[0];
    dx = (int32_t)((uint32_t)p[1] | 
                   ((uint32_t)p[2] << 8) | 
                   ((uint32_t)p[3] << 16) | 
                   ((uint32_t)p[4] << 24));
    dy = (int32_t)((uint32_t)p[5] | 
                   ((uint32_t)p[6] << 8) | 
                   ((uint32_t)p[7] << 16) | 
                   ((uint32_t)p[8] << 24));
  } else if (n >= 4) {
    int16_t x = (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
    int16_t y = (int16_t)((uint16_t)p[2] | ((uint16_t)p[3] << 8));
    dx = x;
    dy = y;
    qual = (n >= 5) ? p[n - 1] : 0;
  }
  
  lastFlowQuality = qual;
  
  const uint32_t now = millis();
  const float dt = (lastFlowMs == 0) ? 0.02f : (now - lastFlowMs) / 1000.0f;
  lastFlowMs = now;
  
  // Velocity calculation for diagnostics
  const float rad_per_count = FOV_RAD / FLOW_PIXELS;
  const float dth_x = dx * rad_per_count;
  const float dth_y = dy * rad_per_count;
  
  vx_mps = (dt > 0.0005f) ? (lastHeight_m * (dth_x / dt)) : 0.0f;
  vy_mps = (dt > 0.0005f) ? (lastHeight_m * (dth_y / dt)) : 0.0f;
}
