
import socket
host = "192.168.2.42"
port = 2001
# Создаем подключение
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((host, port))
try:
    while True:
        data = s.recv(1024)
        if not data:
            break
        print(list(data))
finally:
    # Закрываем соединение
    s.close()
    print("Соединение закрыто")
