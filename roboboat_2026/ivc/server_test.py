from ivc_api import ASVServer
import time

comms = ASVServer()
comms.start()

print("--- Starting Timed Mission Simulation ---")

while True:
    print("Mission in progress: Executing task for 5 seconds...")
    time.sleep(5)  # Simulate the boat doing something
    
    # Send the completion message
    comms.send_string("MISSION_COMPLETED")
    print("Message sent. Waiting for acknowledgement...")

    # Wait for acknowledgement from the client
    acknowledged = False
    timeout_start = time.time()
    
    while not acknowledged and (time.time() - timeout_start < 10):
        # Check the mailbox
        msg = comms.receive_string(timeout=0.1)
        
        if msg == "ACK":
            print("Received Acknowledgement from Client! Restarting loop...")
            acknowledged = True
        elif msg:
            print(f"Received other message: {msg}")
            
    if not acknowledged:
        print("Warning: No acknowledgement received within 10 seconds.")