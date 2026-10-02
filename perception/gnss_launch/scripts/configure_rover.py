#!/usr/bin/env python3
"""
ZED-F9P Rover Configuration Script
Sets TMODE to 0 (Disabled) for rover operation
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

def configure_rover(port, baudrate=38400):
    print(f"🔧 Connecting to {port}...")
    try:
        ser = serial.Serial(port, baudrate, timeout=2)
        time.sleep(1)
    except Exception as e:
        print(f"❌ Failed: {e}")
        return False
    
    print("✓ Connected to F9P\n")
    
    # TMODE3 payload (40 bytes): all zeros = disabled (rover mode)
    print("📡 Setting Rover Mode (TMODE 0 - Disabled)...")
    tmode3_payload = bytes(40)  # All zeros = disabled
    
    msg = UBXMessage.create(0x06, 0x71, tmode3_payload)
    ser.write(msg)
    time.sleep(0.5)
    print("✓ Rover mode set (TMODE 0)\n")
    
    # Enable RTCM input on UART (to receive corrections)
    print("📡 Configuring UART for RTCM input...")
    # CFG-PRT for UART1: enable UBX + RTCM3 input
    prt_payload = bytes([
        0x01,  # Port ID (UART1)
        0x00,  # Reserved
        0x00, 0x00,  # txReady
        0xD0, 0x08, 0x00, 0x00,  # Mode: 8N1
        0x80, 0x25, 0x00, 0x00,  # Baudrate: 9600 (will be overridden by ROS driver)
        0x07, 0x00,  # inProtoMask: UBX + NMEA + RTCM3
        0x03, 0x00,  # outProtoMask: UBX + NMEA
        0x00, 0x00,  # Flags
        0x00, 0x00   # Reserved
    ])
    msg = UBXMessage.create(0x06, 0x00, prt_payload)
    ser.write(msg)
    time.sleep(0.5)
    print("✓ UART configured for RTCM input\n")
    
    # Save to flash
    print("💾 Saving to flash...")
    cfg_payload = bytes([
        0x00, 0x00, 0x00, 0x00,  # Clear mask
        0x1F, 0x06, 0x00, 0x00,  # Save mask (all sections)
        0x00, 0x00, 0x00, 0x00   # Load mask
    ])
    msg = UBXMessage.create(0x06, 0x09, cfg_payload)
    ser.write(msg)
    time.sleep(1)
    print("✓ Configuration saved\n")
    
    ser.close()
    
    print("="*60)
    print("✅ ROVER GPS CONFIGURED!")
    print("="*60)
    print("\nConfiguration:")
    print("  - TMODE 0 (Disabled/Rover)")
    print("  - RTCM3 input enabled")
    print("  - Ready to receive corrections")
    print("\nNext steps:")
    print("  1. Connect to Jetson (/dev/ttyACM0)")
    print("  2. Launch GPS driver")
    print("  3. Start rtcm_injector.py")
    print("  4. Wait for RTK FIX!")
    
    return True

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 configure_rover.py <port>")
        print("\nExamples:")
        print("  Mac:    python3 configure_rover.py /dev/cu.usbmodem1301")
        print("  Jetson: python3 configure_rover.py /dev/ttyACM0")
        return
    
    port = sys.argv[1]
    baudrate = int(sys.argv[2]) if len(sys.argv) > 2 else 38400
    
    configure_rover(port, baudrate)

if __name__ == '__main__':
    main()

