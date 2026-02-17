import socket
import threading
import queue
import time

class ASVComms:
    """Base logic for thread-safe communication."""
    def __init__(self):
        self.conn = None
        self.receive_queue = queue.Queue() #
        self.running = False
        self.lock = threading.Lock() #

    def _receive_thread(self):
        """Background thread that constantly listens for data."""
        while self.running:
            try:
                with self.lock: # Protect socket access
                    if not self.conn:
                        break
                    self.conn.settimeout(1.0) 
                    data = self.conn.recv(1024)
                
                if data:
                    msg = data.decode().strip()
                    self.receive_queue.put(msg) #
                else:
                    self.running = False
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Receive Thread Error: {e}")
                self.running = False

    def start_threads(self):
        """Initializes the background listener."""
        self.running = True
        self.thread = threading.Thread(target=self._receive_thread, daemon=True) #
        self.thread.start()

    def get_next_message(self):
        """Getter for the mission script to pull from the queue."""
        try:
            return self.receive_queue.get_nowait() #
        except queue.Empty:
            return None

    def send_string(self, message):
        """Thread-safe send function."""
        if not message.endswith('\n'):
            message += '\n'
        
        with self.lock: # Prevent race conditions
            if self.conn:
                self.conn.sendall(message.encode())

    def stop(self):
        """Cleanly close the connection."""
        self.running = False
        if self.conn:
            self.conn.close()

# --- Segment 2: The Server Role ---
class ASVServer(ASVComms):
    def __init__(self, port=65432):
        super().__init__()
        self.port = port

    def start(self):
        """Binds to the Jetson port and waits for the partner boat."""
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) #
        s.bind(('0.0.0.0', self.port)) # Use 0.0.0.0 for Docker
        s.listen()
        print(f"Server ASV waiting for partner on port {self.port}...")
        self.conn, _ = s.accept()
        print("Partner connected!")
        self.start_threads() #

# --- Segment 3: The Client Role ---
class ASVClient(ASVComms):
    def __init__(self, server_ip, port=65432):
        super().__init__()
        self.server_ip = server_ip
        self.port = port

    def connect(self, retries=5):
        """Connects to the Leader ASV's IP."""
        for i in range(retries):
            try:
                self.conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.conn.connect((self.server_ip, self.port))
                self.start_threads() #
                print(f"Connected to Leader at {self.server_ip}")
                return True
            except Exception:
                print(f"Retry {i+1}/{retries}...")
                time.sleep(2)
        return False