import socket

# Use the Ethernet IP we verified earlier
SERVER_IP = '192.168.8.230' 
PORT = 65432

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    try:
        s.connect((SERVER_IP, PORT))
        print(f"Connected to Jetson at {SERVER_IP}")
        
        while True:
            # 1. Send message to Jetson
            message = input("Your message (Mac): ")
            if message.lower() == 'quit':
                break
            s.sendall(message.encode())
            
            # 2. Receive reply from Jetson
            data = s.recv(1024)
            print(f"Jetson says: {data.decode()}")
            
    except Exception as e:
        print(f"Connection error: {e}")

print("Connection closed.")