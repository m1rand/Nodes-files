import socket
import sys
import termios
import tty
import time

# --- НАЛАШТУВАННЯ ---
ROBOT_IP = "192.168.0.101" # IP вашого робота
CONTROL_PORT = 8889       # Порт, який слухає ESP32 для команд

# Словник команд
key_bindings = {
    'w': 'w', 's': 's', 'a': 'a', 'd': 'd',
    'q': 'q', 'e': 'e', 'x': 'x', 'k': 'x',
    '1': '1', '2': '2', '3': '3', '4': '4', '5': '5',
    '6': '6', '7': '7', '8': '8', '9': '9',
}

# Функція отримання символу з клавіатури
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def print_instructions():
    print("--- Simple WiFi Keyboard Teleop ---")
    print("w/s: Forward/Backward | a/d: Turn Left/Right")
    print("q/e: Pivot Left/Right | x or k: STOP")
    print("1-9: Set Speed       | Ctrl+C: Exit")
    print("-----------------------------------")

if __name__ == '__main__':
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        print(f"Connecting to robot at {ROBOT_IP}:{CONTROL_PORT}...")
        sock.connect((ROBOT_IP, CONTROL_PORT))
        print("Connection successful! You can now control the robot.")
    except Exception as e:
        print(f"Connection failed: {e}")
        sys.exit(1)

    print_instructions()

    while True:
        key = getch()
        if key in key_bindings:
            command = key_bindings[key]
            try:
                sock.sendall(command.encode('utf-8'))
            except Exception as e:
                print(f"Failed to send command: {e}\nConnection lost. Exiting.")
                break
        elif key == '\x03': # Ctrl+C
            break
    
    sock.close()
    print("\nConnection closed.")