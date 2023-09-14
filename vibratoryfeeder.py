import serial
import time

def calculate_checksum(data):
    checksum = sum(data) + 13
    checksum1 = (checksum >> 4) & 0x0F
    checksum2 = checksum & 0x0F
    return checksum1, checksum2

def start_feeder(com_port):
    try:
        com_port.write(bytes([0x05, 0x4C, 0x5A, 0x53, 0x45]))
        checksum1, checksum2 = calculate_checksum([0x4C, 0x5A, 0x53, 0x45])
        com_port.write(bytes([checksum1, checksum2, 13]))

        print("Command sent. Waiting for response...")
        time.sleep(0.5)  # Add a small delay to allow the feeder to process the command
        response = com_port.read(1024)
        print("Response received.")
        print("Firmware version:", response[3:-3].decode("ascii"))
    except serial.SerialException as e:
        print("Serial port communication error:", str(e))

try:
    com_port = serial.Serial('/dev/ttyUSB0', baudrate=19200, bytesize=8, parity='N', stopbits=1, timeout=1)
    print("Serial port opened successfully.")
    start_feeder(com_port)
    com_port.close()
    print("Serial port closed.")
except serial.SerialException as e:
    print("Serial port connection error:", str(e))