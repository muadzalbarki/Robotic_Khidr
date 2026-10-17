import serial
import time

PORT = "/dev/ttyUSB0"
BAUD = 115200

def debug_lidar():
    print(f"Membuka serial pada {PORT} dengan baudrate {BAUD}...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        print("Berhasil terbuka.")
        
        # Kirim perintah start (A5 60)
        ser.write(bytes([0xA5, 0x60]))
        print("Perintah start (A5 60) terkirim.")
        print("Menunggu data mentah (Hex)... Tekan Ctrl+C untuk berhenti.")
        
        while True:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                print(data.hex(' '), end=' ')
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nBerhenti.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    debug_lidar()
