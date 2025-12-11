#!/usr/bin/env python3

import serial
import json
import time
import pygame
import sys
import threading

ser = None

def read_serial():
    global ser
    while True:
        data = ser.readline().decode('utf-8')
        if data:
            print(f"Received: {data}", end='')


class DDSM400Controller:
    """Controller for DDSM400 hub motors via serial JSON protocol."""
    
    def __init__(self, port='/dev/ttyACM0', baud=115200):
        """Initialize serial connection to DDSM Driver HAT.
        
        Args:
            port: Serial port (e.g., '/dev/ttyUSB0' or '/dev/ttyAMA0')
            baud: Baud rate (default 115200)
        """
        self.port = port
        self.baud = baud
        self.serial = None
        self.lock = threading.Lock()
        
        # Motor parameters
        self.motor_ids = [1, 2]  # Right=1, Left=2
        self.max_rpm = 500  # DDSM400 max speed
        
        # Connect
        self.connect()
    
    def connect(self):
        """Connect to DDSM Driver HAT."""
        try:
            self.serial = serial.Serial(self.port, baudrate=115200, dsrdtr=None)
            self.serial.setRTS(False)
            self.serial.setDTR(False)
            time.sleep(2)  # Wait for connection
            print(f"✓ Connected to DDSM Driver HAT on {self.port}")
            global ser
            ser = self.serial

            serial_recv_thread = threading.Thread(target=read_serial)
            serial_recv_thread.daemon = True
            serial_recv_thread.start()

            return True
        except Exception as e:
            print(f"✗ Failed to connect: {e}")
            return False
    
    def send_command(self, cmd):
        """Send JSON command to motor driver.
        
        Args:
            cmd: Dictionary containing command
        """
        if self.serial is None:
            return False
        
        with self.lock:
            try:
                json_str = json.dumps(cmd) + '\n'
                time.sleep(0.05)  # IMPORTANT: Otherwise commands are sent too fast and it skips
                self.serial.write(json_str.encode())
                self.serial.flush()
                time.sleep(0.05)  # IMPORTANT: Otherwise commands are sent too fast and it skips
                return True
            except Exception as e:
                print(f"Command error: {e}")
                return False
    
    def enable_motors(self):
        """Enable both wheel motors."""
        print("Enabling motors...")
        for motor_id in self.motor_ids:
            # Enable motor
            self.send_command({"T": 11002, "id": motor_id})
            time.sleep(0.5)

            # Set motor to speed control mode (mode 2)
            self.send_command({"T": 10012, "id": motor_id, "mode": 2})
            time.sleep(0.5)
            
            ## Set heartbeat timeout (500ms) - motor stops if no commands received
            #self.send_command({"T": 10052, "id": motor_id, "cmd": 500})
            #time.sleep(0.5)
        
        print("✓ Motors enabled")
    
    def disable_motors(self):
        """Disable both wheel motors."""
        print("Disabling motors...")
        for motor_id in self.motor_ids:
            # Stop motor
            self.set_speed(motor_id, 0)
            time.sleep(0.5)
            
            # Disable
            self.send_command({"T": 11003, "id": motor_id})
            time.sleep(0.5)
        
        print("✓ Motors disabled")
    
    def set_speed(self, motor_id, rpm):
        """Set motor speed in RPM.
        
        Args:
            motor_id: Motor ID (1 or 2)
            rpm: Speed in RPM (positive=forward, negative=reverse)
        """
        # Clamp to max speed
        rpm = max(-self.max_rpm, min(self.max_rpm, rpm))
        
        # Send speed command
        # T:10010 = speed control
        # cmd = speed in 0.1 RPM units
        # act = acceleration (3 = moderate)
        cmd_value = int(rpm * 10)  # Convert to 0.1 RPM units
        self.send_command({"T": 10010, "id": motor_id, "cmd": cmd_value, "act": 3})
    
    def stop_all(self):
        """Stop both motors immediately."""
        for motor_id in self.motor_ids:
            self.set_speed(motor_id, 0)
    
    def close(self):
        """Close serial connection."""
        if self.serial:
            self.stop_all()
            time.sleep(0.1)
            self.disable_motors()
            time.sleep(1)
            self.serial.close()
            print("Connection closed")


class GamepadWheelControl:
    """Gamepad control for DDSM400 wheel motors."""
    
    def __init__(self):
        """Initialize gamepad and motor controller."""
        # Initialize pygame for gamepad input
        pygame.init()
        pygame.joystick.init()
        
        # Check for gamepad
        if pygame.joystick.get_count() == 0:
            print("✗ No gamepad detected!")
            sys.exit(1)
        
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        print(f"✓ Gamepad connected: {self.joystick.get_name()}")
        
        # Initialize motor controller
        self.motors = DDSM400Controller()
        self.motors.enable_motors()
        
        # State
        self.wheel_mode_active = False
        self.running = True
        
        # Button states for debouncing
        self.last_button_states = [0] * self.joystick.get_numbuttons()
        
        print("\n" + "="*60)
        print("DDSM400 Wheel Control Ready")
        print("="*60)
        print("Controls:")
        print("  X/Square (2)    : Activate wheel mode")
        print("  Circle/B (1)    : Deactivate wheel mode")
        print("  Left Stick Y    : Forward/Backward (base speed)")
        print("  Right Stick X   : Steering (left/right ratio)")
        print("                    - Left = turn left")
        print("                    - Right = turn right")
        print("  L1+R1 (4+5)     : Emergency stop and exit")
        print("="*60)
        print("\nWaiting for input...\n")
    
    def handle_buttons(self):
        """Handle button presses with debouncing."""
        # Button 2 (X/Square) - Activate wheel mode
        if self.joystick.get_button(2) == 1 and self.last_button_states[2] == 0:
            if not self.wheel_mode_active:
                self.wheel_mode_active = True
                print("\n" + "="*60)
                print("🚗 WHEEL MODE ACTIVATED")
                print("="*60)
                print("  Use left stick for forward/backward")
                print("  Use right stick for turning")
                print("  Press Circle/B to stop")
                print("="*60 + "\n")
        
        # Button 1 (Circle/B) - Deactivate wheel mode
        if self.joystick.get_button(1) == 1 and self.last_button_states[1] == 0:
            if self.wheel_mode_active:
                self.wheel_mode_active = False
                self.motors.stop_all()
                print("\n" + "="*60)
                print("🛑 WHEEL MODE DEACTIVATED")
                print("="*60)
                print("  Wheels stopped")
                print("  Press X/Square to reactivate")
                print("="*60 + "\n")
        
        # L1 + R1 (buttons 4 + 5) - Emergency stop
        if self.joystick.get_button(4) and self.joystick.get_button(5):
            print("\n" + "="*60)
            print("🛑 EMERGENCY STOP - Shutting down")
            print("="*60 + "\n")
            self.running = False
        
        # Update button states
        for i in range(self.joystick.get_numbuttons()):
            self.last_button_states[i] = self.joystick.get_button(i)
    
    def handle_axes(self):
        """Handle joystick axes for wheel control."""
        if not self.wheel_mode_active:
            return
        
        # Read joystick axes
        # Axis 1: Left stick Y (forward/backward) - NOT inverted since user said it's reversed
        # Axis 2: Right stick X (steering)
        base_speed = self.joystick.get_axis(1)  # Forward when pushed up (positive Y = backward on stick)
        steering = self.joystick.get_axis(0)    # Right stick X for steering
        
        # Apply deadzone
        deadzone = 0.1
        if abs(base_speed) < deadzone:
            base_speed = 0.0
        if abs(steering) < deadzone:
            steering = 0.0
        
        # Convert base_speed to target RPM
        max_rpm = self.motors.max_rpm
        
        forward = max_rpm * 0.7 * base_speed

        multi = 1
        if base_speed < 0:
            multi * -1

        right = max_rpm * 0.4 * steering * multi

        # Calculate final RPM for each wheel
        right_rpm = forward + right
        left_rpm = -1 * (forward - right)
        
        # Clamp to max RPM
        right_rpm = max(-max_rpm, min(max_rpm, right_rpm))
        left_rpm = max(-max_rpm, min(max_rpm, left_rpm))
        
        # Send commands to motors
        self.motors.set_speed(1, right_rpm)  # Right wheel (motor ID 1)
        self.motors.set_speed(2, left_rpm)   # Left wheel (motor ID 2)
    
    def run(self):
        """Main control loop."""
        clock = pygame.time.Clock()
        
        try:
            while self.running:
                # Process pygame events
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        self.running = False
                
                # Handle inputs
                self.handle_buttons()
                self.handle_axes()
                
                # Run at 20Hz (sends commands every 50ms, well within 500ms heartbeat)
                clock.tick(20)
        
        except KeyboardInterrupt:
            print("\n\nInterrupted by user")
        
        finally:
            # Cleanup
            print("\nCleaning up...")
            self.motors.close()
            pygame.quit()
            print("Done!")
    
    def __del__(self):
        """Destructor - ensure cleanup."""
        if hasattr(self, 'motors'):
            self.motors.close()


def main():
    """Main entry point."""
    print("\n" + "="*60)
    print("DDSM400 Standalone Wheel Control")
    print("="*60 + "\n")
    
    try:
        controller = GamepadWheelControl()
        controller.run()
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
