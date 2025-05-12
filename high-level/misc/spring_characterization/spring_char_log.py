import serial
import csv
from datetime import datetime
import time

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600
DURATION_SECONDS = 30

# Output CSV file 
timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
filename = f"sensor_log_{timestamp}.csv"

#  Open Serial and CSV 
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
csv_file = open(filename, mode='w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(["Timestamp (ms)", "Encoder1", "Encoder2", "Loadcell"])

print(f"Logging started. Saving to {filename}")

start_time = time.time()

try:
    while time.time() - start_time < DURATION_SECONDS:
        line = ser.readline().decode('utf-8').strip()
        parts = line.split(',')
        if len(parts) == 3:
            try:
                enc1 = float(parts[0])
                enc2 = float(parts[1])
                weight = float(parts[2])
                timestamp_ms = int(datetime.now().timestamp() * 1000)
                csv_writer.writerow([timestamp_ms, enc1, enc2, weight])
                csv_file.flush()
            except ValueError:
                continue 

except KeyboardInterrupt:
    print("\nLogging stopped by user.")

finally:
    ser.close()
    csv_file.close()
    print(f"Data saved to {filename}")
