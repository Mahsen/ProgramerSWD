import sys
import serial
import os
import time

def send_command(ser, command):
    ser.write(command.encode())  # Append end-of-text character
    start_time = time.time()
    while time.time() - start_time < 2:  # Timeout after 2 seconds
        data = ser.read(1)  # Read one byte at a time
        if data == b'\x06':
            return True
    return False

def send_file(filename, port):
    try:
        ser = serial.Serial(port, baudrate=115200, timeout=1)
        
        # Send commands
        commands = [
            "W0(1)",
            "W1(0)",
            "W2(0)"
        ]
		
		# Send additional command after file transfer
        send_command(ser, "W0(2)")
			
        for cmd in commands:
            if not send_command(ser, cmd):
                print(f"Error: Acknowledgment not received for command: {cmd}")
                ser.close()
                return
        
        # Print "Connect on chip" after sending three commands
        print("Connect on chip")
        
        # File transfer
        file_size = os.path.getsize(filename)
        bytes_sent = 0
        
        with open(filename, 'rb') as f:
            while True:
                chunk = f.read(65536)
                if not chunk:
                    break
                ser.write(chunk)
                bytes_sent += len(chunk)
                progress = min(100, int(bytes_sent / file_size * 100))
                print(f"Progress: {bytes_sent}/{file_size} bytes sent ({progress}%)")
                
                ack_received = False
                start_time = time.time()
                while time.time() - start_time < 3:  # Timeout after 2 seconds
                    data = ser.read(int(int(len(chunk))/64))  # Read 64 bytes at a time
                    if data == b'\x06' * int((int(len(chunk))/64)):  # Check if 64 instances of 0x06 received
                        ack_received = True
                        break
                if not ack_received:
                    print(f"Error: Acknowledgment not received for file chunk.{data}")
                    ser.close()
                    return
        
        # Send additional command after file transfer
        if not send_command(ser, "W0(2)"):
            print("Error: Acknowledgment not received for additional command after file transfer.")
            ser.close()
            return
        
        ser.close()
        print("File sent successfully.")
    except serial.SerialException as e:
        print("Error opening serial port:", e)
    except Exception as e:
        print("Error:", e)
        ser.close()

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python script.py filename port")
        sys.exit(1)

    filename = sys.argv[1]
    port = sys.argv[2]
    
    send_file(filename, port)
	
# Usage example
# send_file('C:/Users/lotfi/Desktop/v1.14021103.bin', 'COM48')
