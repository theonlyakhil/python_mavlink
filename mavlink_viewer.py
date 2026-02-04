"""
MAVLink Data Viewer
Connects to a Pixhawk flight controller via USB and displays MAVLink data stream.

Author: Created for Pixhawk Cube flight controller communication......
"""

import time
import sys
from datetime import datetime

try:
    from pymavlink import mavutil
except ImportError:
    print("Error: pymavlink is not installed.")
    print("Please install it using: pip install pymavlink")
    sys.exit(1)


class MAVLinkViewer:
    """Class to handle MAVLink connection and data display."""
    
    def __init__(self, port='COM13', baudrate=115200):
        """
        Initialize MAVLink connection.
        
        Args:
            port: COM port where Pixhawk is connected (default: COM13)
            baudrate: Baud rate for serial communication (default: 115200)
        """
        self.port = port
        self.baudrate = baudrate
        self.connection = None
        self.running = True
        
    def connect(self):
        """Establish connection to the flight controller."""
        print(f"\n{'='*60}")
        print(f"  MAVLink Data Viewer")
        print(f"{'='*60}")
        print(f"\nConnecting to Pixhawk on {self.port} at {self.baudrate} baud...")
        
        try:
            # Create MAVLink connection
            self.connection = mavutil.mavlink_connection(
                self.port,
                baud=self.baudrate,
                source_system=255,  # Ground station ID
                source_component=0
            )
            print("Connection established successfully!")
            return True
        except Exception as e:
            print(f"Error connecting: {e}")
            return False
    
    def wait_for_heartbeat(self):
        """Wait for the first heartbeat from the flight controller."""
        print("\nWaiting for heartbeat from flight controller...")
        print("(Sending heartbeats to trigger response...)")
        
        try:
            # Actively send heartbeats while waiting to trigger response
            start_time = time.time()
            timeout = 60  # 60 seconds timeout
            
            while time.time() - start_time < timeout:
                # Send our heartbeat to trigger response
                self.send_heartbeat()
                
                # Check for incoming heartbeat (non-blocking, short timeout)
                msg = self.connection.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
                if msg:
                    print(f"Heartbeat received from system {self.connection.target_system}, "
                          f"component {self.connection.target_component}")
                    return True
                    
                print(f"  Still waiting... ({int(time.time() - start_time)}s elapsed)")
            
            print("Timeout waiting for heartbeat!")
            return False
        except Exception as e:
            print(f"Error waiting for heartbeat: {e}")
            return False
    
    def send_heartbeat(self):
        """Send heartbeat to the flight controller."""
        try:
            self.connection.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,           # Type: Ground Control Station
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,  # Autopilot type
                0,                                       # Base mode
                0,                                       # Custom mode
                mavutil.mavlink.MAV_STATE_ACTIVE        # System status
            )
        except Exception as e:
            print(f"Error sending heartbeat: {e}")
    
    def request_data_stream(self):
        """Request all data streams from the flight controller."""
        print("\nRequesting data streams from flight controller...")
        
        # Request all data streams at 4Hz
        self.connection.mav.request_data_stream_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            4,  # Rate in Hz
            1   # Start sending
        )
        print("Data stream request sent!")
    
    def format_message(self, msg):
        """Format a MAVLink message for display."""
        msg_type = msg.get_type()
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        
        # Create formatted output based on message type
        output = f"\n[{timestamp}] {msg_type}"
        output += f"\n{'-'*50}"
        
        # Format specific message types with human-readable output
        if msg_type == 'HEARTBEAT':
            autopilot_types = {
                0: "Generic", 3: "ArduPilot", 12: "PX4"
            }
            mav_types = {
                0: "Generic", 1: "Fixed Wing", 2: "Quadrotor",
                13: "Hexarotor", 14: "Octorotor"
            }
            output += f"\n  Type: {mav_types.get(msg.type, msg.type)}"
            output += f"\n  Autopilot: {autopilot_types.get(msg.autopilot, msg.autopilot)}"
            output += f"\n  Base Mode: {msg.base_mode}"
            output += f"\n  Custom Mode: {msg.custom_mode}"
            output += f"\n  System Status: {msg.system_status}"
            
        elif msg_type == 'ATTITUDE':
            import math
            output += f"\n  Roll:  {math.degrees(msg.roll):7.2f}°"
            output += f"\n  Pitch: {math.degrees(msg.pitch):7.2f}°"
            output += f"\n  Yaw:   {math.degrees(msg.yaw):7.2f}°"
            output += f"\n  Roll Rate:  {math.degrees(msg.rollspeed):7.2f}°/s"
            output += f"\n  Pitch Rate: {math.degrees(msg.pitchspeed):7.2f}°/s"
            output += f"\n  Yaw Rate:   {math.degrees(msg.yawspeed):7.2f}°/s"
            
        elif msg_type == 'GPS_RAW_INT':
            fix_types = {0: "No GPS", 1: "No Fix", 2: "2D Fix", 3: "3D Fix", 
                        4: "DGPS", 5: "RTK Float", 6: "RTK Fixed"}
            output += f"\n  Fix Type: {fix_types.get(msg.fix_type, msg.fix_type)}"
            output += f"\n  Latitude:  {msg.lat / 1e7:.7f}°"
            output += f"\n  Longitude: {msg.lon / 1e7:.7f}°"
            output += f"\n  Altitude:  {msg.alt / 1000:.2f} m (MSL)"
            output += f"\n  Satellites: {msg.satellites_visible}"
            output += f"\n  HDOP: {msg.eph / 100:.2f}"
            output += f"\n  VDOP: {msg.epv / 100:.2f}"
            
        elif msg_type == 'SYS_STATUS':
            output += f"\n  Voltage: {msg.voltage_battery / 1000:.2f} V"
            output += f"\n  Current: {msg.current_battery / 100:.2f} A"
            output += f"\n  Battery Remaining: {msg.battery_remaining}%"
            output += f"\n  Comm Drop Rate: {msg.drop_rate_comm / 100:.1f}%"
            output += f"\n  Errors: Comm={msg.errors_comm}"
            
        elif msg_type == 'VFR_HUD':
            output += f"\n  Airspeed:    {msg.airspeed:.1f} m/s"
            output += f"\n  Groundspeed: {msg.groundspeed:.1f} m/s"
            output += f"\n  Heading:     {msg.heading}°"
            output += f"\n  Throttle:    {msg.throttle}%"
            output += f"\n  Altitude:    {msg.alt:.1f} m"
            output += f"\n  Climb Rate:  {msg.climb:.1f} m/s"
            
        elif msg_type == 'GLOBAL_POSITION_INT':
            output += f"\n  Latitude:  {msg.lat / 1e7:.7f}°"
            output += f"\n  Longitude: {msg.lon / 1e7:.7f}°"
            output += f"\n  Altitude:  {msg.alt / 1000:.2f} m (MSL)"
            output += f"\n  Relative Alt: {msg.relative_alt / 1000:.2f} m"
            output += f"\n  Velocity X: {msg.vx / 100:.2f} m/s"
            output += f"\n  Velocity Y: {msg.vy / 100:.2f} m/s"
            output += f"\n  Velocity Z: {msg.vz / 100:.2f} m/s"
            output += f"\n  Heading: {msg.hdg / 100:.1f}°"
            
        elif msg_type == 'RAW_IMU':
            output += f"\n  Accel X: {msg.xacc} raw"
            output += f"\n  Accel Y: {msg.yacc} raw"
            output += f"\n  Accel Z: {msg.zacc} raw"
            output += f"\n  Gyro X:  {msg.xgyro} raw"
            output += f"\n  Gyro Y:  {msg.ygyro} raw"
            output += f"\n  Gyro Z:  {msg.zgyro} raw"
            output += f"\n  Mag X:   {msg.xmag} raw"
            output += f"\n  Mag Y:   {msg.ymag} raw"
            output += f"\n  Mag Z:   {msg.zmag} raw"
            
        elif msg_type == 'SCALED_PRESSURE':
            output += f"\n  Abs Pressure: {msg.press_abs:.2f} hPa"
            output += f"\n  Diff Pressure: {msg.press_diff:.2f} hPa"
            output += f"\n  Temperature: {msg.temperature / 100:.2f}°C"
            
        elif msg_type == 'RC_CHANNELS':
            output += f"\n  Channels: {msg.chancount}"
            output += f"\n  Ch1-4: {msg.chan1_raw}, {msg.chan2_raw}, {msg.chan3_raw}, {msg.chan4_raw}"
            output += f"\n  Ch5-8: {msg.chan5_raw}, {msg.chan6_raw}, {msg.chan7_raw}, {msg.chan8_raw}"
            output += f"\n  RSSI: {msg.rssi}"
            
        elif msg_type == 'SERVO_OUTPUT_RAW':
            output += f"\n  Port: {msg.port}"
            output += f"\n  Servo 1-4: {msg.servo1_raw}, {msg.servo2_raw}, {msg.servo3_raw}, {msg.servo4_raw}"
            output += f"\n  Servo 5-8: {msg.servo5_raw}, {msg.servo6_raw}, {msg.servo7_raw}, {msg.servo8_raw}"
            
        elif msg_type == 'BATTERY_STATUS':
            output += f"\n  Battery ID: {msg.id}"
            output += f"\n  Temperature: {msg.temperature / 100:.1f}°C" if msg.temperature != 32767 else ""
            output += f"\n  Current: {msg.current_battery / 100:.2f} A"
            output += f"\n  Current Consumed: {msg.current_consumed} mAh"
            output += f"\n  Energy Consumed: {msg.energy_consumed} hJ"
            output += f"\n  Battery Remaining: {msg.battery_remaining}%"
            
        elif msg_type == 'STATUSTEXT':
            severity_names = {
                0: "EMERGENCY", 1: "ALERT", 2: "CRITICAL", 3: "ERROR",
                4: "WARNING", 5: "NOTICE", 6: "INFO", 7: "DEBUG"
            }
            output += f"\n  [{severity_names.get(msg.severity, msg.severity)}] {msg.text}"
            
        elif msg_type == 'PARAM_VALUE':
            output += f"\n  {msg.param_id}: {msg.param_value}"
            output += f"\n  Index: {msg.param_index}/{msg.param_count}"
            
        else:
            # Generic handler for other message types
            msg_dict = msg.to_dict()
            for key, value in msg_dict.items():
                if key != 'mavpackettype':
                    output += f"\n  {key}: {value}"
        
        return output
    
    def run(self):
        """Main loop to receive and display MAVLink messages."""
        if not self.connect():
            return
        
        if not self.wait_for_heartbeat():
            return
        
        self.request_data_stream()
        
        print(f"\n{'='*60}")
        print("  Streaming MAVLink Data (Press Ctrl+C to stop)")
        print(f"{'='*60}")
        
        last_heartbeat_time = time.time()
        message_count = 0
        
        try:
            while self.running:
                # Send heartbeat every second to keep connection alive
                current_time = time.time()
                if current_time - last_heartbeat_time >= 1.0:
                    self.send_heartbeat()
                    last_heartbeat_time = current_time
                
                # Receive message with timeout
                msg = self.connection.recv_match(blocking=True, timeout=0.5)
                
                if msg:
                    message_count += 1
                    
                    # Skip some high-frequency messages to reduce console spam
                    # Comment out this filter to see ALL messages
                    skip_types = ['TIMESYNC', 'SYSTEM_TIME']
                    if msg.get_type() not in skip_types:
                        print(self.format_message(msg))
                        
        except KeyboardInterrupt:
            print(f"\n\n{'='*60}")
            print(f"  Stopped by user")
            print(f"  Total messages received: {message_count}")
            print(f"{'='*60}")
        finally:
            if self.connection:
                self.connection.close()
                print("Connection closed.")


def main():
    """Main entry point."""
    import argparse
    
    parser = argparse.ArgumentParser(
        description='MAVLink Data Viewer - Display MAVLink data from Pixhawk flight controller'
    )
    parser.add_argument(
        '--port', '-p',
        default='COM13',
        help='COM port where Pixhawk is connected (default: COM13)'
    )
    parser.add_argument(
        '--baud', '-b',
        type=int,
        default=115200,
        help='Baud rate for serial communication (default: 115200)'
    )
    
    args = parser.parse_args()
    
    viewer = MAVLinkViewer(port=args.port, baudrate=args.baud)
    viewer.run()


if __name__ == '__main__':
    main()
