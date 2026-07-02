import socket
import time

HOST, PORT = "192.168.0.14", 3000
LOCAL_IP = "192.168.0.200"

while True:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(10.0)
            s.bind((LOCAL_IP, 0))
            s.connect((HOST, PORT))
            print("Connected", HOST, PORT)
            while True:
                data = s.recv(4096)
                if not data:
                    break
                text = data.decode("utf-8", errors="replace")
                # Каждое значение (x;y;v3;v4) — с новой строки
                for part in text.replace(")(", ")\n(").split("\n"):
                    if part.strip():
                        print(part.strip())
                time.sleep(0.01)
    except (ConnectionRefusedError, OSError) as e:
        print(f"Retry in 2s... ({e})")
        time.sleep(2)
