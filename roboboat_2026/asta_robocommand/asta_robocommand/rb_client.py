from datetime import datetime, timezone
from google.protobuf.timestamp_pb2 import Timestamp
from msgs.report_pb2 import *
from report_body import HeartbeatMsg,GatePassMsg
import socket, struct
import rclpy
import traceback
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from std_srvs.srv import Trigger

class RBClient(Node):
    def __init__(self):
        super().__init__('RBClient')
        # General Information
        self.team_id = "ASTA"
        self.vehicle_id = "Barco_Polo"
        
        # Track basic robot information
        self.pos = [] # lat,lon
        self.vel = float() # speed
        self.current_task = str() # current task
        self.host = "10.10.10.1"
        self.port = 50000
        
        # Create Subscribers and Service Servers
        # self.create_subscribers()
        # self.create_services()

        # send loop 
        self.msg_queue = []
        self.report_queue = [] 
        self.organizer = self.create_timer(1.0, self.organize_loop)  # 1Hz for heatbeat
        self.reporter  = self.create_timer(0.2, self.report_loop)  # 5Hz for all msgs
        self.loop_counter = 0
        self.seq = 0

    def _send(self, rpt: Report):
        """
        Sends a Report protobuf with a 4-byte big-endian length prefix.
        IMPORTANT: Do NOT append any extra bytes (no b"test").
        """

        # Optional: sanity check you're sending the expected envelope
        # oneof_name = rpt.WhichOneof("body")
        # if oneof_name != "heartbeat":
        #     self.get_logger().warn(f"Sending Report with body={oneof_name}")

        wire = rpt.SerializeToString()
        frame = struct.pack("!I", len(wire)) + wire

        # connect + send
        with socket.create_connection((self.host, self.port), timeout=1.0) as s:
            s.sendall(frame)


    def wrap_report(team_id: str, vehicle_id: str, seq: int, body):
        ts = Timestamp()
        ts.FromDatetime(datetime.now(timezone.utc))

        rpt = Report(team_id=team_id, vehicle_id=vehicle_id, seq=seq)
        rpt.sent_at.CopyFrom(ts)

    # Route by message type
        if isinstance(body, Heartbeat):
            rpt.heartbeat.CopyFrom(body)
        elif isinstance(body, GatePass):
            rpt.gate_pass.CopyFrom(body)
        else:
           raise TypeError(f"Unsupported report body type: {type(body)}")

        return rpt

    def organize_loop(self):
        """
        Creates a heartbeat body using report_body.HeartbeatMsg(...)
        Wraps it into a Report envelope (team_id, vehicle_id, seq, sent_at),
        then enqueues the Report for report_loop/_send to transmit.
        """

        # Build the Heartbeat body using your existing helper
        hb = HeartbeatMsg(
            state="UNKOWN",                 # "UNKNOWN"/"KILLED"/"MANUAL"/"AUTO"
            lat=0.0,
            lon=0.0,
            speed=0.0,
            heading=0.0,
            current_task="UNKOWN",   # "UNKNOWN"/"NONE"/"NAV_CHANNEL"/...
        )

        if hb is None:
            self.get_logger().warn("HeartbeatMsg returned None (invalid state/task); skipping heartbeat")
            return

    # Build Report envelope
        ts = Timestamp()
        ts.FromDatetime(datetime.now(timezone.utc))

        rpt = Report(
            team_id=self.team_id,
            vehicle_id=self.vehicle_id,
            seq=int(self.seq),
        )
        rpt.sent_at.CopyFrom(ts)
        rpt.heartbeat.CopyFrom(hb)

    # enqueue
        self.report_queue.append(rpt)

    # increment sequence for next report
        self.seq += 1

    def report_loop(self):
        if not self.report_queue:
            return

        msg = self.report_queue.pop(0)
        try:
            self.get_logger().info(f"[report] sending message, remaining queue {len(self.report_queue)}")
            self._send(msg)
            self.get_logger().info("[report] send returned")
        except Exception as e:
            # This prints the full traceback
            self.get_logger().error(f"report_loop exception (send failed): {type(e).__name__}: {e}\n{traceback.format_exc()}")
            # Put it back so you don't lose the message
            self.report_queue.insert(0, msg)
    
    """All subscribers and services"""
    def create_subscribers(self):
        self.sub_gps = self.create_subscription(
            Float32MultiArray,
            '/GPS',
            self.gps_callback,
            3
        )

        # TODO subscribe to other topics for velocity, robot state, current task

    def create_services(self):
        # TODO create services that contains str or float array ...etc 
        self.srv = self.create_service(
            Trigger,
            'toggle_water_pump',
            self.pump_callback
        )

    """Topic Callbacks"""
    def gps_callback(msg):
         pass
    
    """Service Callbacks"""
    


def main(args=None):
    rclpy.init(args=args)
    node = RBClient()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl-C received")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
