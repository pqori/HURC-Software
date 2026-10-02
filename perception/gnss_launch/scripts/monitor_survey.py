#!/usr/bin/env python3
"""
Base Station Survey-In Monitor
Monitors u-blox F9P survey-in progress in real-time
Run this on your Mac/laptop connected to base station GPS
"""

import serial
import struct
import time
import sys
from datetime import datetime, timedelta

class UBXMessage:
    @staticmethod
    def checksum(data):
        ck_a = ck_b = 0
        for byte in data:
            ck_a = (ck_a + byte) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF
        return bytes([ck_a, ck_b])
    
    @staticmethod
    def create(msg_class, msg_id, payload=b''):
        header = b'\xb5\x62'
        length = struct.pack('<H', len(payload))
        msg = header + bytes([msg_class, msg_id]) + length + payload
        return msg + UBXMessage.checksum(bytes([msg_class, msg_id]) + length + payload)
    
    @staticmethod
    def parse_nav_svin(payload):
        """Parse NAV-SVIN message"""
        if len(payload) < 40:
            return None
        
        version = payload[0]
        reserved1 = payload[1:4]
        iTOW = struct.unpack('<I', payload[4:8])[0]
        dur = struct.unpack('<I', payload[8:12])[0]  # Survey duration (seconds)
        meanX = struct.unpack('<i', payload[12:16])[0]  # ECEF X (cm)
        meanY = struct.unpack('<i', payload[16:20])[0]  # ECEF Y (cm)
        meanZ = struct.unpack('<i', payload[20:24])[0]  # ECEF Z (cm)
        meanXHP = struct.unpack('<b', payload[24:25])[0]  # High precision X (0.1mm)
        meanYHP = struct.unpack('<b', payload[25:26])[0]  # High precision Y (0.1mm)
        meanZHP = struct.unpack('<b', payload[26:27])[0]  # High precision Z (0.1mm)
        reserved2 = payload[27]
        meanAcc = struct.unpack('<I', payload[28:32])[0]  # Mean accuracy (0.1mm)
        obs = struct.unpack('<I', payload[32:36])[0]  # Number of observations
        valid = payload[36]
        active = payload[37]
        
        return {
            'duration': dur,
            'mean_x': meanX / 100.0,  # Convert to meters
            'mean_y': meanY / 100.0,
            'mean_z': meanZ / 100.0,
            'mean_acc': meanAcc / 10000.0,  # Convert to meters
            'observations': obs,
            'valid': bool(valid),
            'active': bool(active)
        }

def read_ubx_message(ser, timeout=2.0):
    """Read a complete UBX message from serial port"""
    start_time = time.time()
    
    # Find sync chars
    while (time.time() - start_time) < timeout:
        b1 = ser.read(1)
        if not b1 or b1[0] != 0xB5:
            continue
        b2 = ser.read(1)
        if not b2 or b2[0] != 0x62:
            continue
        
        # Read class, id, length
        header = ser.read(4)
        if len(header) < 4:
            continue
        
        msg_class, msg_id = header[0], header[1]
        length = struct.unpack('<H', header[2:4])[0]
        
        # Read payload and checksum
        payload = ser.read(length)
        checksum = ser.read(2)
        
        if len(payload) < length or len(checksum) < 2:
            continue
        
        # Verify checksum
        calc_ck = UBXMessage.checksum(header + payload)
        if calc_ck == checksum:
            return msg_class, msg_id, payload
    
    return None, None, None

def poll_nav_svin(ser):
    """Request NAV-SVIN status"""
    msg = UBXMessage.create(0x01, 0x3B)  # NAV-SVIN poll
    ser.write(msg)
    ser.flush()

def poll_cfg_tmode3(ser):
    """Request CFG-TMODE3 status"""
    msg = UBXMessage.create(0x06, 0x71)  # CFG-TMODE3 poll
    ser.write(msg)
    ser.flush()

def parse_cfg_tmode3(payload):
    """Parse CFG-TMODE3 message to get TMODE"""
    if len(payload) < 40:
        return None
    
    version = payload[0]
    reserved1 = payload[1]
    flags = struct.unpack('<H', payload[2:4])[0]
    
    # Extract mode from flags (bits 0-2)
    mode = flags & 0x03
    
    return {
        'mode': mode,
        'mode_name': ['Disabled (Rover)', 'Survey-In', 'Fixed'][mode] if mode <= 2 else 'Unknown'
    }

def format_duration(seconds):
    """Format duration as MM:SS"""
    minutes = seconds // 60
    secs = seconds % 60
    return f"{minutes:02d}:{secs:02d}"

def print_progress_bar(current, target, width=40):
    """Print a progress bar"""
    if target == 0:
        progress = 0
    else:
        progress = min(current / target, 1.0)
    
    filled = int(width * progress)
    bar = '█' * filled + '░' * (width - filled)
    percentage = progress * 100
    
    return f"[{bar}] {percentage:5.1f}%"

def monitor_survey(port, baudrate=38400, duration_minutes=15):
    """Monitor survey-in progress"""
    
    print("="*70)
    print("📡 BASE STATION SURVEY-IN MONITOR")
    print("="*70)
    print(f"Port: {port}")
    print(f"Baudrate: {baudrate}")
    print(f"Monitor duration: {duration_minutes} minutes")
    print("="*70)
    print()
    
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"✅ Connected to {port}\n")
        time.sleep(1)
    except Exception as e:
        print(f"❌ Failed to open port: {e}")
        return
    
    start_time = datetime.now()
    end_time = start_time + timedelta(minutes=duration_minutes)
    
    last_update = time.time()
    survey_complete = False
    last_mode = None
    
    # Typical survey targets (can be configured via CFG-TMODE3)
    target_duration = 300  # 5 minutes default
    target_accuracy = 1.0  # 1 meter default
    
    try:
        print("🔍 Polling GPS status...\n")
        
        while datetime.now() < end_time:
            current_time = time.time()
            
            # Poll every 2 seconds
            if current_time - last_update >= 2.0:
                # Request TMODE status
                poll_cfg_tmode3(ser)
                time.sleep(0.1)
                
                # Request survey status
                poll_nav_svin(ser)
                last_update = current_time
            
            # Read responses
            msg_class, msg_id, payload = read_ubx_message(ser, timeout=0.5)
            
            if msg_class is None:
                continue
            
            # Parse CFG-TMODE3 response
            if msg_class == 0x06 and msg_id == 0x71:
                tmode_data = parse_cfg_tmode3(payload)
                if tmode_data and tmode_data['mode'] != last_mode:
                    last_mode = tmode_data['mode']
                    timestamp = datetime.now().strftime("%H:%M:%S")
                    print(f"\n[{timestamp}] 🔧 TMODE CHANGED: {tmode_data['mode_name']} (mode={tmode_data['mode']})")
                    
                    if tmode_data['mode'] == 2:
                        print("\n" + "="*70)
                        print("🎉 SURVEY COMPLETE! BASE STATION IS NOW IN FIXED MODE!")
                        print("="*70)
                        print("\n✅ Base station is ready to provide RTK corrections")
                        print("✅ Rover should achieve RTK FIX within 1-3 minutes")
                        survey_complete = True
            
            # Parse NAV-SVIN response
            if msg_class == 0x01 and msg_id == 0x3B:
                svin_data = UBXMessage.parse_nav_svin(payload)
                
                if svin_data:
                    timestamp = datetime.now().strftime("%H:%M:%S")
                    elapsed = (datetime.now() - start_time).total_seconds()
                    remaining = (end_time - datetime.now()).total_seconds()
                    
                    # Clear previous lines (move cursor up and clear)
                    print("\033[F\033[K" * 7, end='')
                    
                    print(f"[{timestamp}] Survey Status (monitoring for {format_duration(int(remaining))} more)")
                    print("-" * 70)
                    
                    if svin_data['active']:
                        # Survey is running
                        dur_progress = print_progress_bar(svin_data['duration'], target_duration, 35)
                        acc_progress = print_progress_bar(target_accuracy, svin_data['mean_acc'], 35) if svin_data['mean_acc'] > 0 else "[░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░]   0.0%"
                        
                        print(f"📊 Active:       {'YES (Survey running)' if svin_data['active'] else 'NO'}")
                        print(f"⏱️  Duration:     {format_duration(svin_data['duration'])} / {format_duration(target_duration)} {dur_progress}")
                        print(f"🎯 Accuracy:     {svin_data['mean_acc']:.3f}m / {target_accuracy:.1f}m target {acc_progress}")
                        print(f"📡 Observations: {svin_data['observations']:,}")
                        print(f"✓  Valid:        {'YES ✅' if svin_data['valid'] else 'NO ⏳ (waiting...)'}")
                        
                        if svin_data['valid']:
                            print("\n🎉 Survey criteria MET! Waiting for TMODE to switch to Fixed (2)...")
                    else:
                        # Survey not active
                        if survey_complete:
                            print(f"✅ Survey completed - Base in FIXED mode")
                        else:
                            print(f"⏸️  Survey not active (TMODE may be 0 or 2)")
                        print(f"📊 Last duration:  {format_duration(svin_data['duration'])}")
                        print(f"🎯 Last accuracy:  {svin_data['mean_acc']:.3f}m")
                        print(f"📡 Observations:   {svin_data['observations']:,}")
                    
                    print()
            
            if survey_complete:
                print("\n✅ Monitoring complete - Survey finished successfully!")
                break
        
        if not survey_complete:
            print("\n⏰ Monitoring period ended")
            print("💡 If survey is still running, base needs more time")
            print("💡 Check PyGPSClient for TMODE status (should be 1 or 2)")
        
    except KeyboardInterrupt:
        print("\n\n⚠️  Monitoring stopped by user")
    except Exception as e:
        print(f"\n\n❌ Error: {e}")
    finally:
        ser.close()
        print("\n👋 Disconnected from GPS")

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 monitor_survey.py <port> [baudrate] [duration_minutes]")
        print("\nExamples:")
        print("  Mac:    python3 monitor_survey.py /dev/cu.usbmodem1201")
        print("  Mac:    python3 monitor_survey.py /dev/cu.usbmodem1201 38400 15")
        print("  Linux:  python3 monitor_survey.py /dev/ttyACM0 38400 10")
        return
    
    port = sys.argv[1]
    baudrate = int(sys.argv[2]) if len(sys.argv) > 2 else 38400
    duration_minutes = int(sys.argv[3]) if len(sys.argv) > 3 else 15
    
    monitor_survey(port, baudrate, duration_minutes)

if __name__ == '__main__':
    main()



