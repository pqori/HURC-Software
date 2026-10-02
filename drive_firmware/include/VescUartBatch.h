#pragma once
#include <Arduino.h>
#include <vector>

class VescUartBatch {
public:
    VescUartBatch(HardwareSerial &serial) : serialPort(serial) {}

    void begin(uint32_t baud = 460800) {
        serialPort.begin(baud, SERIAL_8N1);
        batchBuffer.clear();
    }

    void beginBatch() {
        batchBuffer.clear();
    }

    void queueCANRPM(uint8_t canId, int32_t rpm) {
        // Compose one subframe: [CAN_ID][COMM_SET_RPM][4-byte RPM]
        uint8_t payload[6];
        payload[0] = canId;
        payload[1] = COMM_SET_RPM;      // 8 for COMM_SET_RPM
        int32ToBytes(rpm, payload + 2);
        batchBuffer.insert(batchBuffer.end(), payload, payload + sizeof(payload));
    }

    void sendBatch() {
        if (batchBuffer.empty()) return;

        // Construct full UART packet
        uint8_t packet[batchBuffer.size() + 1];
        packet[0] = COMM_FORWARD_CAN; // outer command
        memcpy(packet + 1, batchBuffer.data(), batchBuffer.size());

        sendPacket(packet, sizeof(packet));
        batchBuffer.clear();
    }

private:
    HardwareSerial &serialPort;
    std::vector<uint8_t> batchBuffer;

    static const uint8_t COMM_FORWARD_CAN = 9;
    static const uint8_t COMM_SET_RPM = 8;

    void int32ToBytes(int32_t val, uint8_t *buf) {
        // scale for VESC 4-byte RPM representation (x1000)
        int32_t scaled = val;
        buf[0] = (scaled >> 24) & 0xFF;
        buf[1] = (scaled >> 16) & 0xFF;
        buf[2] = (scaled >> 8) & 0xFF;
        buf[3] = scaled & 0xFF;
    }

    void sendPacket(uint8_t *buf, size_t len) {
        // VESC packet format: start, payload, CRC, end
        serialPort.write(2);      // start byte
        serialPort.write(len);    // payload length
        uint16_t crc = crc16(buf, len);
        serialPort.write(buf, len);
        serialPort.write(crc >> 8);
        serialPort.write(crc & 0xFF);
        serialPort.write(3);      // end byte
        serialPort.flush();
    }

    uint16_t crc16(uint8_t *buf, size_t len) {
        uint16_t crc = 0;
        for (size_t i = 0; i < len; i++) {
            crc ^= ((uint16_t)buf[i]) << 8;
            for (uint8_t j = 0; j < 8; j++) {
                if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
                else crc <<= 1;
            }
        }
        return crc;
    }
};
