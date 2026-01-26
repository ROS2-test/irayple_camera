# echo-client.py

import socket
import time
import fractions

HOST = "192.168.0.8"  # The server's hostname or IP address
PORT = 3000  # The port used by the server
while(1):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        data = s.recv(1024)

    print(f"Received {data}")
    try:
        text = data.decode('utf-8').strip('()')
        values = tuple(map(int, text.split(';')))
        print(values)
    except Exception as e:
        print(f'error: {e}')
    time.sleep(0.1)