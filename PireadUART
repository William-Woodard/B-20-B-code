import serial
from time import sleep

# Open serial port with the specified baud rate
ser = serial.Serial("/dev/ttyS0", 9600)

while True:
    received_data = ser.read()  # read one byte from serial port
    sleep(0.03)
    data_left = ser.inWaiting()  # check for remaining bytes
    received_data += ser.read(data_left)  # read remaining bytes

    try:
        # Try to decode the received data as UTF-8
        decoded_data = received_data.decode('utf-8')
        print(f"Decoded Data: {decoded_data}")
    except UnicodeDecodeError:
        # If decoding fails, print the raw hexadecimal data
        print(f"Raw Data: {received_data}")

    # Echo back the received data
    ser.write(received_data)
