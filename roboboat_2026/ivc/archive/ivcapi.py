import socket
import time

class ASVServer:
    def __init__(self, port=65432):
        self.port = port
        self.conn = None

    def start(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('192.168.8.229', self.port)) # Barco Polo IP 
        s.listen()
        print(f"Server ASV listening on {self.port}...")
        self.conn, addr = s.accept()
        print(f"Connected to Partner: {addr}")
        return self.conn

    def send_string(self, message):
        """Sends a raw string message to the connected client."""
        if self.conn:
            # Ensure the message ends with a newline for your parser
            if not message.endswith('\n'):
                message += '\n'
            self.conn.sendall(message.encode())
            print(f"Server Sent: {message.strip()}")
        else:
            print("Error: No client connected to server.")

    def receive_string(self, timeout=0.1):
        """Checks for an incoming message without freezing the script."""
        target = self.conn if hasattr(self, 'conn') else self.sock
        
        if not target:
            return None

        try:
            # Set a very short timeout so we don't 'get stuck'
            target.settimeout(timeout)
            data = target.recv(1024)
            if data:
                return data.decode().strip()
        except socket.timeout:
            # No message arrived, just keep going with the mission
            return None
        except Exception as e:
            print(f"Receive error: {e}")
            return None
        return None

class ASVClient:
    def __init__(self, server_ip, port=65432):
        self.server_ip = server_ip
        self.port = port
        self.sock = None

    def connect(self, retries=5):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        for i in range(retries):
            try:
                self.sock.connect((self.server_ip, self.port))
                print(f"Connected to Server ASV at {self.server_ip}")
                return True
            except Exception:
                print(f"Connection attempt {i+1} failed. Retrying...")
                time.sleep(2)
        return False

    def send_string(self, message):
        """Sends a raw string message to the server."""
        if self.sock:
            if not message.endswith('\n'):
                message += '\n'
            self.sock.sendall(message.encode())
            print(f"Client Sent: {message.strip()}")
        else:
            print("Error: Client not connected to server.")
    
    def receive_string(self, timeout=0.1):
        """Checks for an incoming message without freezing the script."""
        target = self.conn if hasattr(self, 'conn') else self.sock
        
        if not target:
            return None

        try:
            # Set a very short timeout so we don't 'get stuck'
            target.settimeout(timeout)
            data = target.recv(1024)
            if data:
                return data.decode().strip()
        except socket.timeout:
            # No message arrived, just keep going with the mission
            return None
        except Exception as e:
            print(f"Receive error: {e}")
            return None
        return None