import serial

COM_PORT = 'COM8'
BAUD_RATE = 115200
OUTPUT_FILE = "uart_output.txt"

# Open serial port
ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)

with open(OUTPUT_FILE, "w", encoding='utf-8') as f:
    print(f"Listening on {COM_PORT} at {BAUD_RATE} baud...")
    try:
        while True:
            line = ser.readline()  # read until \n
            if line:
                # Decode bytes to string
                text = line.decode(errors='ignore')  # ignore any non-UTF8 bytes
                f.write(text)  # write as-is
                print(text, end='')  # optional: also print to console
    except KeyboardInterrupt:
        print("Stopped by user")
