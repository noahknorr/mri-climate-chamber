import serial
import csv
import time
import datetime

# Setup
ser = serial.Serial(port='COM3', baudrate=9600, timeout=1)
print(datetime.date.today().year)
output_file = "recording_" + str(datetime.datetime.now().year) + f'{datetime.datetime.now().month:02}'+ f'{datetime.datetime.now().day:02}' + "_" + f'{datetime.datetime.now().hour:02}' + f'{datetime.datetime.now().minute:02}' + ".csv"
print(output_file)
# Read and write
with open(output_file, mode='w', newline='\n') as file:
    file.write("sep=,\n")
    writer = None  # CSV writer will be defined after headers are known
    try:
        print(f"Reading from {ser.port} and writing to {output_file}, press Ctrl C to end recording...")
        while True:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8').strip()
                # Example format: "Temp=25.3 Humidity=60.1"
                parts = line.split(',')
                data = {}
                data['unixSeconds'] = time.time()
                for part in parts:
                    key, value = part.split(':')
                    if ':' in part:
                        data[key.strip()] =  float(value.strip())
                # Initialize writer and write header
                if writer is None:
                    writer = csv.DictWriter(file, fieldnames=list(data.keys()))
                    writer.writeheader()
                #print("line written")
                print(data)
                writer.writerow(data)
                file.flush()
    except KeyboardInterrupt:
        print("Stopped by user.")
    finally:
        ser.close()