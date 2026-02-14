import socket

# Use 0.0.0.0 to listen on both Ethernet (.145) and Wi-Fi (.230)
HOST = '192.168.8.230'
PORT = 65432

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    print(f"Jetson Server is listening on port {PORT}...")
    
    conn, addr = s.accept()
    with conn:
        print(f"Connected by {addr}")
        while True:
            # 1. Receive data from Mac
            data = conn.recv(1024)
            if not data:
                break  # Connection closed
            print(f"Mac says: {data.decode()}")
            
            # 2. Send a reply back to Mac
            reply = input("Your reply (Jetson): ")
            conn.sendall(reply.encode())

print("Connection closed.")