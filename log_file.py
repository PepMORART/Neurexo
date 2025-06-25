import serial
import csv
from datetime import datetime


PORT = 'COM12'        
BAUDRATE = 115200
FILENAME = f"datos_movimiento_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

def main():
    try:
        with serial.Serial(PORT, BAUDRATE, timeout=1) as ser, open(FILENAME, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['timestamp', 'position_knee', 'position_hip', 'flex_value'])
            print(f"[INFO] Grabando en {FILENAME}. Presiona Ctrl+C para detener.")

            while True:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    parts = line.split(',')
                    if len(parts) == 4:
                        writer.writerow(parts)
                        print(parts)
    except KeyboardInterrupt:
        print("\n[INFO] Finalizado por el usuario.")
    except Exception as e:
        print(f"[ERROR] {e}")

if __name__ == '__main__':
    main()
