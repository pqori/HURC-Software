#!/usr/bin/env python3
"""
RTCM Correction Injector for ZED-F9P
Receives RTCM corrections from network and injects to GPS serial port

The ZED-F9P can receive RTCM on the same serial port as UBX commands.
This script carefully writes RTCM data while ROS driver reads from the port.

Usage:
    python3 rtcm_injector.py <rtcm_server_ip> <rtcm_server_port> <gps_device> [baudrate]
    
Example:
    python3 rtcm_injector.py 10.242.111.130 50012 /dev/ttyACM0 38400
"""

import socket
import serial
import sys
import time
import threading
import struct

class RTCMInjector:
    def __init__(self, server_ip, server_port, gps_device, baudrate=38400):
        self.server_ip = server_ip
        self.server_port = int(server_port)
        self.gps_device = gps_device
        self.baudrate = baudrate
        self.running = False
        self.bytes_sent = 0
        self.rtcm_count = 0
        self.sock = None
        self.gps = None
        
    def connect_rtcm_server(self):
        """Connect to RTCM network server"""
        print(f"📡 Connecting to RTCM server {self.server_ip}:{self.server_port}...", flush=True)
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(10)
            self.sock.connect((self.server_ip, self.server_port))
            print(f"✓ Connected to RTCM server", flush=True)
            return True
        except Exception as e:
            print(f"❌ Failed to connect to RTCM server: {e}", flush=True)
            return False
    
    def open_gps_port(self):
        """Open GPS serial port for writing RTCM"""
        print(f"📍 Opening GPS port {self.gps_device} @ {self.baudrate} baud...", flush=True)
        try:
            # Open port without exclusive lock if possible
            # Use write_timeout to prevent blocking
            self.gps = serial.Serial(
                port=self.gps_device,
                baudrate=self.baudrate,
                timeout=None,  # Non-blocking read
                write_timeout=0.5,  # 500ms write timeout
                exclusive=False  # Don't require exclusive access
            )
            print(f"✓ GPS port opened", flush=True)
            return True
        except Exception as e:
            print(f"⚠️  Could not open GPS port exclusively: {e}", flush=True)
            print(f"   This is expected if ROS driver is running", flush=True)
            print(f"   Trying alternative method...", flush=True)
            
            # Try opening without configuring serial parameters
            # Just for writing, let ROS driver handle reads
            try:
                self.gps = open(self.gps_device, 'wb', buffering=0)
                print(f"✓ GPS port opened for RTCM injection", flush=True)
                return True
            except Exception as e2:
                print(f"❌ Failed to open GPS port: {e2}", flush=True)
                return False
    
    def parse_rtcm_header(self, data, offset=0):
        """Parse RTCM3 message header to get message type and length"""
        if len(data) < offset + 3:
            return None, None, None
            
        if data[offset] != 0xD3:  # RTCM3 preamble
            return None, None, None
        
        # Length is 10 bits after the preamble
        length = ((data[offset + 1] & 0x03) << 8) | data[offset + 2]
        
        if len(data) < offset + 3 + length:
            return None, None, None
        
        # Message type is first 12 bits of payload
        if length >= 2:
            msg_type = (data[offset + 3] << 4) | ((data[offset + 4] & 0xF0) >> 4)
        else:
            msg_type = None
        
        # Total message length: header(3) + payload(length) + crc(3)
        total_length = 3 + length + 3
        
        return msg_type, length, total_length
    
    def inject_rtcm(self):
        """Main injection loop - receive RTCM and write to GPS"""
        print("\n" + "="*60, flush=True)
        print("✅ INJECTING RTCM CORRECTIONS", flush=True)
        print("="*60, flush=True)
        print(f"RTCM Server: {self.server_ip}:{self.server_port}", flush=True)
        print(f"GPS Device: {self.gps_device}", flush=True)
        print(f"Baud Rate: {self.baudrate}", flush=True)
        print("\nPress Ctrl+C to stop...\n", flush=True)
        
        self.running = True
        last_status = time.time()
        buffer = b''
        
        try:
            while self.running:
                # Receive data from RTCM server
                try:
                    data = self.sock.recv(4096)
                    if not data:
                        print("❌ RTCM server disconnected", flush=True)
                        break
                    
                    buffer += data
                    
                    # Process complete RTCM messages from buffer
                    while len(buffer) >= 6:
                        # Find RTCM message start
                        start_idx = buffer.find(b'\xD3')
                        if start_idx == -1:
                            buffer = b''
                            break
                        
                        if start_idx > 0:
                            buffer = buffer[start_idx:]
                        
                        # Parse message
                        msg_type, payload_len, total_len = self.parse_rtcm_header(buffer)
                        
                        if total_len is None or len(buffer) < total_len:
                            # Incomplete message, wait for more data
                            break
                        
                        # Extract complete message
                        rtcm_msg = buffer[:total_len]
                        buffer = buffer[total_len:]
                        
                        # Write to GPS
                        try:
                            if isinstance(self.gps, serial.Serial):
                                self.gps.write(rtcm_msg)
                            else:  # file object
                                self.gps.write(rtcm_msg)
                                self.gps.flush()
                            
                            self.bytes_sent += len(rtcm_msg)
                            self.rtcm_count += 1
                            
                        except Exception as e:
                            print(f"⚠️  Write error: {e}", flush=True)
                            time.sleep(0.1)
                    
                except socket.timeout:
                    continue
                except Exception as e:
                    print(f"❌ Receive error: {e}", flush=True)
                    break
                
                # Print status every 5 seconds
                now = time.time()
                if now - last_status >= 5.0:
                    rate = self.bytes_sent / (now - last_status)
                    print(f"[{time.strftime('%H:%M:%S')}] 📊 {self.bytes_sent} bytes | "
                          f"{self.rtcm_count} RTCM msgs | {rate:.1f} B/s", flush=True)
                    self.bytes_sent = 0
                    last_status = now
                    
        except KeyboardInterrupt:
            print("\n\n⏹️  Stopping RTCM injection...", flush=True)
        except Exception as e:
            print(f"\n❌ Error: {e}", flush=True)
        finally:
            self.running = False
            if self.sock:
                self.sock.close()
            if self.gps:
                if isinstance(self.gps, serial.Serial):
                    self.gps.close()
                else:
                    self.gps.close()
            print(f"✓ Disconnected", flush=True)
    
    def run(self):
        """Connect and start injection"""
        if not self.connect_rtcm_server():
            return False
        
        if not self.open_gps_port():
            if self.sock:
                self.sock.close()
            return False
        
        # Small delay to let connections stabilize
        time.sleep(0.5)
        
        # Start injection
        self.inject_rtcm()
        return True


def main():
    if len(sys.argv) < 4:
        print("RTCM Correction Injector for ZED-F9P")
        print("\nUsage:")
        print("  python3 rtcm_injector.py <server_ip> <server_port> <gps_device> [baudrate]")
        print("\nExample:")
        print("  python3 rtcm_injector.py 10.242.111.130 50012 /dev/ttyACM0 38400")
        print("\nThis script receives RTCM corrections from a network server")
        print("and injects them into the GPS serial port for RTK positioning.")
        return 1
    
    server_ip = sys.argv[1]
    server_port = sys.argv[2]
    gps_device = sys.argv[3]
    baudrate = int(sys.argv[4]) if len(sys.argv) >= 5 else 38400
    
    print("🌐 RTCM Correction Injector")
    print("="*60)
    
    injector = RTCMInjector(server_ip, server_port, gps_device, baudrate)
    
    success = injector.run()
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())

