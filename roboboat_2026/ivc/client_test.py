from ivc_api import ASVClient
import time

# Use the static IP of your Server Jetson (e.g., 192.168.8.145)
comms = ASVClient(server_ip='192.168.8.229')

if comms.connect():
    print("--- Client Listening Mode Active ---")
    # Use Queue data structure to store message to send (For sending)
    while True:
        # Constantly check for messages
        msg = comms.receive_string(timeout=0.01)
        
        if msg:
            print(f"Server says: {msg}")
            
            if msg == "MISSION_COMPLETED":
                print("Sending Acknowledgement...")
                comms.send_string("ACK")
        
        # Small sleep to prevent 100% CPU usage, but keep it responsive
        time.sleep(0.01)