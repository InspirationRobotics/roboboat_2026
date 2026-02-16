import time
from ivc_api import ASVServer, ASVClient

# --- CONFIGURATION ---
IS_LEADER = True  # Set to False on the second ASV
PARTNER_IP = '192.168.8.229'  # Use the IP of the Leader Jetson
PORT = 65432

def run_mission():
    # 1. Initialize Role
    if IS_LEADER:
        comms = ASVServer(port=PORT)
        comms.start()
    else:
        comms = ASVClient(server_ip=PARTNER_IP, port=PORT)
        if not comms.connect():
            print("Failed to connect to Leader. Exiting.")
            return

    print(f"Mission Manager started as {'LEADER' if IS_LEADER else 'FOLLOWER'}")

    try:
        while True:
            if IS_LEADER:
                # Leader Logic: Run task, then send completion
                print("Leader: Executing mission task (5s)...")
                time.sleep(5)
                
                comms.send_string("MISSION_COMPLETED")
                print("Leader: Message sent. Waiting for ACK...")

                # Non-blocking check for ACK
                start_wait = time.time()
                while time.time() - start_wait < 10:
                    msg = comms.get_next_message()
                    if msg == "ACK":
                        print("Leader: Received ACK! Restarting mission cycle.")
                        break
                    time.sleep(0.1)

            else:
                # Follower Logic: Constant listening
                msg = comms.get_next_message()
                if msg:
                    print(f"Follower: Received from Leader: {msg}")
                    
                    if msg == "MISSION_COMPLETED":
                        print("Follower: Sending ACK back...")
                        comms.send_string("ACK")
                
                # Prevent 100% CPU usage while listening
                time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nMission stopped by user.")
    finally:
        comms.stop()

if __name__ == "__main__":
    run_mission()