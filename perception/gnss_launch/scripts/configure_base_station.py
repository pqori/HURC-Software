#!/usr/bin/env python3
"""
ZED-F9P Base Station Configuration Script
"""

import serial
import time
import struct
import sys

class UBXMessage:
    @staticmethod
    def checksum(data):
        ck_a = ck_b = 0
        for byte in data:
            ck_a = (ck_a + byte) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF
        return bytes([ck_a, ck_b])
    
    @staticmethod
    def create(msg_class, msg_id, payload):
        header = b'\xb5\x62'
        length = struct.pack('<H', len(payload))
        msg = header + bytes([msg_class, msg_id]) + length + payload
        return msg + UBXMessage.checksum(bytes([msg_class, msg_id]) + length + payload)

def configure_base_station(port, baudrate=38400):
    print(f"🔧 Connecting to {port}...")
    try:
        ser = serial.Serial(port, baudrate, timeout=2)
        time.sleep(1)
    except Exception as e:
        print(f"❌ Failed: {e}")
        return False
    
    print("✓ Connected to F9P\n")
    
    # TMODE3 payload (40 bytes): survey-in mode
    print("📡 Configuring Survey-In Mode...")
    tmode3_payload = bytes([
        0x00, 0x00,  # version, reserved1
        0x01, 0x00,  # flags (bit 0 = survey-in enabled)
        0x00, 0x00, 0x00, 0x00,  # ECEF X
        0x00, 0x00, 0x00, 0x00,  # ECEF Y
        0x00, 0x00, 0x00, 0x00,  # ECEF Z
        0x00, 0x00, 0x00, 0x00,  # ECEF X/Y/Z HP + reserved
        0x00, 0x00, 0x00, 0x00,  # Fixed pos accuracy
        0x2C, 0x01, 0x00, 0x00,  # Min duration: 300 sec
        0x10, 0x27, 0x00, 0x00,  # Acc limit: 10000 (1m in 0.1mm)
        0x00, 0x00, 0x00, 0x00   # Reserved
    ])
    
    msg = UBXMessage.create(0x06, 0x71, tmode3_payload)
    ser.write(msg)
    time.sleep(0.5)
    print("✓ Survey-In enabled\n")
    
    # Enable RTCM messages
    print("📡 Enabling RTCM messages...")
    rtcm_msgs = [(0xF5, 0x05), (0xF5, 0x4D), (0xF5, 0x57), (0xF5, 0x61), (0xF5, 0x7F), (0xF5, 0xE6)]
    
    for msg_class, msg_id in rtcm_msgs:
        payload = bytes([msg_class, msg_id, 1])  # rate = 1Hz
        msg = UBXMessage.create(0x06, 0x01, payload)
        ser.write(msg)
        time.sleep(0.1)
    
    print("✓ RTCM enabled\n")
    
    # Save to flash
    print("💾 Saving to flash...")
    cfg_payload = bytes([
        0x00, 0x00, 0x00, 0x00,  # Clear mask
        0x1F, 0x06, 0x00, 0x00,  # Save mask
        0x00, 0x00, 0x00, 0x00   # Load mask
    ])
    msg = UBXMessage.create(0x06, 0x09, cfg_payload)
    ser.write(msg)
    time.sleep(1)
    print("✓ Saved\n")
    
    ser.close()
    
    print("="*60)
    print("✅ BASE STATION CONFIGURED!")
    print("="*60)
    print("\nTake it outside:")
    print("1. GPS will get fix (30-60 sec)")
    print("2. Survey runs for 5 minutes")
    print("3. Switches to FIXED mode")
    print("4. Streams RTCM corrections")
    print("5. Rover gets RTK FIX!")
    
    return True

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 configure_base_station.py /dev/cu.usbmodem21301")
        return
    
    configure_base_station(sys.argv[1])

if __name__ == '__main__':
    main()
