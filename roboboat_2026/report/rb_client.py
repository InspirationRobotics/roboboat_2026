from datetime import datetime, timezone
from google.protobuf.timestamp_pb2 import Timestamp
from roboboat_2026.report.msgs.report_pb2 import *
from roboboat_2026.report.report_body import HeartbeatMsg,GatePassMsg
import socket, struct
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, String
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_srvs.srv import Trigger

class RBClient(Node):
    def __init__(self):
        super().__init__('RBClient')
        # General Information
        self.team_id = "ASTA"
        self.vehicle_ids = {"Barco_Polo":'S1',"Crusader":'S2'}
        self.current_vehicle = "Barco_Polo"  # Track which vehicle is reporting
        
        # Track basic robot information
        self.cur_state:str = "UNKNOWN"
        self.pos = None # lat,lon
        self.heading:float = None
        self.vel:float = 0  # speed
        self.cur_task:str = "UNKNOWN" # current task
        
        # Create Subscribers and Service Servers
        self.create_subscribers()


        # send loop 
        self.msg_queue = []
        self.report_queue = [] 
        self.organizer = self.create_timer(1.0, self.organize_loop)  # 1Hz for heatbeat
        self.reporter  = self.create_timer(0.2, self.report_loop)  # 5Hz for all msgs
        self.seq_counter = 0  # Renamed for clarity

    def __send(self, msg_body):
        """
        Create and send a Report message with the given body
        
        :param msg_body: One of Heartbeat, GatePass, ObjectDetected, ObjectDelivery, Docking, or SoundSignal
        """
        # Create Report with proper fields
        report = Report(
            team_id=self.team_id,
            vehicle_id=self.vehicle_ids.get(self.current_vehicle, 'S1'),
            seq=self.seq_counter
        )
        
        # Set the appropriate body field based on message type
        if isinstance(msg_body, Heartbeat):
            report.heartbeat.CopyFrom(msg_body)
        elif isinstance(msg_body, GatePass):
            report.gate_pass.CopyFrom(msg_body)
        elif isinstance(msg_body, ObjectDetected):
            report.object_detected.CopyFrom(msg_body)
        elif isinstance(msg_body, ObjectDelivery):
            report.object_delivery.CopyFrom(msg_body)
        elif isinstance(msg_body, Docking):
            report.docking.CopyFrom(msg_body)
        elif isinstance(msg_body, SoundSignal):
            report.sound_signal.CopyFrom(msg_body)
        else:
            self.get_logger().error(f"Unknown message body type: {type(msg_body)}")
            return
        
        # Set timestamp
        ts = Timestamp()
        ts.FromDatetime(datetime.now(timezone.utc))
        report.sent_at.CopyFrom(ts)

        # Increment sequence counter after creating the report
        self.seq_counter += 1

        # Serialize (binary protobuf) and send with 4-byte big-endian length prefix
        wire = report.SerializeToString()
        frame = b'$R' + struct.pack("!B", len(wire)) + wire + b'!!'

        try:
            with socket.create_connection(("10.10.10.1", 50000)) as s:
                s.sendall(frame)
                self.get_logger().debug(f"Sent report seq={report.seq}")
        except Exception as e:
            self.get_logger().error(f"Failed to send report: {e}")

    def organize_loop(self): # 1 Hz
        try:
            if self.pos is None: 
                self.get_logger().warn("GPS data not received")
                return
            
            # Create heartbeat message body
            heartbeat_body = HeartbeatMsg(
                state=self.cur_state,
                lat=self.pos[0],
                lon=self.pos[1],
                speed=self.vel,
                heading=self.heading,
                current_task=self.cur_task
            )
            
            # Add heartbeat to front of queue
            self.report_queue = [heartbeat_body]
            
            # Add up to 5 additional messages from msg_queue
            while len(self.report_queue) <= 5 and len(self.msg_queue) > 0:
                self.report_queue.append(self.msg_queue.pop(0))
                
        except Exception as e:
            self.get_logger().error(f"Error in organize loop. report queue is {self.report_queue}, msg queue is {self.msg_queue}")
            self.get_logger().warn(f"{e}")
  
    def report_loop(self):
        try:
            if len(self.report_queue) > 0:
                msg_body = self.report_queue.pop(0)
                self.__send(msg_body)
        except Exception as e:
            self.get_logger().error(f"Error in report loop, report queue is {self.report_queue}")
            self.get_logger().warn(f"{e}")
    
    """All subscribers and services"""
    def create_subscribers(self):
        self.sub_gps = self.create_subscription(
            Float32MultiArray,
            '/GPS',
            self.gps_callback,
            3
        )

        self.sub_state = self.create_subscription(
            String,
            '/robot_state',
            self.state_callback,
            3
        )

        self.sub_pose = self.create_subscription(
            TwistStamped,
            '/fused/velocity',
            self.twist_callback,
            3
        )
        # TODO subscribe to other topics for velocity, robot state, current task

        self.sub_report = self.create_subscription(
            String,
            '/report',
            self.report_callback,
            3
        )

    """Topic Callbacks"""
    def gps_callback(self, msg):
        self.pos = [msg.data[0],msg.data[1]]
        self.heading = msg.data[2]
    
    def state_callback(self, msg):
        self.cur_state = msg.data

    def twist_callback(self,msg):
        self.vel = math.sqrt((msg.twist.linear.x)**2 + (msg.twist.linear.y)**2)

    def report_callback(self, msg):
        # Extract data
        line = msg.data 
        # Parse line
        self.parse(line)
    
    """Parsing function"""
    def parse(self, line):
        """
        Parse a string containing reporting info and add appropriate message to queue
        
        :param line str: A string containing reporting info
        
        Supported formats:
        - "GatePass,ENTRY,32.112345,-21.12345"
        - "GatePass,EXIT,32.112345,-21.12345"
        - "GatePass,SPEED_START,32.112345,-21.12345"
        - "GatePass,SPEED_END,32.112345,-21.12345"
        - "ObjectDetected,BOAT,RED,32.112345,-21.12345,1,NAV_CHANNEL"
        - "ObjectDelivery,GREEN,32.112345,-21.12345,WATER"
        - "Docking,N,1"
        - "SoundSignal,ONE_BLAST,600,DOCKING"
        """
        try:
            parts = line.strip().split(',')
            if len(parts) == 0:
                self.get_logger().warn(f"Empty line received")
                return
            
            msg_type = parts[0].strip()
            
            if msg_type == "GatePass":
                self._parse_gate_pass(parts)
            elif msg_type == "ObjectDetected":
                self._parse_object_detected(parts)
            elif msg_type == "ObjectDelivery":
                self._parse_object_delivery(parts)
            elif msg_type == "Docking":
                self._parse_docking(parts)
            elif msg_type == "SoundSignal":
                self._parse_sound_signal(parts)
            else:
                self.get_logger().warn(f"Unknown message type: {msg_type}")
                
        except Exception as e:
            self.get_logger().error(f"Error parsing line '{line}': {e}")

    def _parse_gate_pass(self, parts):
        """
        Parse GatePass message
        Format: "GatePass,TYPE,lat,lon"
        Example: "GatePass,ENTRY,32.112345,-21.12345"
        """
        try:
            if len(parts) != 4:
                self.get_logger().warn(f"Invalid GatePass format: expected 4 parts, got {len(parts)}")
                return
            
            gate_type = parts[1].strip()
            lat = float(parts[2].strip())
            lon = float(parts[3].strip())
            
            msg = GatePassMsg(gate_type, lat, lon)
            if msg is not None:
                self.msg_queue.append(msg)
                self.get_logger().info(f"GatePass added: {gate_type} at ({lat}, {lon})")
            else:
                self.get_logger().warn(f"Invalid GatePass type: {gate_type}")
                
        except ValueError as e:
            self.get_logger().error(f"Error parsing GatePass coordinates: {e}")
        except Exception as e:
            self.get_logger().error(f"Error in _parse_gate_pass: {e}")

    def _parse_object_detected(self, parts):
        """
        Parse ObjectDetected message
        Format: "ObjectDetected,TYPE,COLOR,lat,lon,id,task"
        Example: "ObjectDetected,BOAT,RED,32.112345,-21.12345,1,NAV_CHANNEL"
        """
        try:
            if len(parts) != 7:
                self.get_logger().warn(f"Invalid ObjectDetected format: expected 7 parts, got {len(parts)}")
                return
            
            obj_type = parts[1].strip()
            color = parts[2].strip()
            lat = float(parts[3].strip())
            lon = float(parts[4].strip())
            obj_id = int(parts[5].strip())
            task = parts[6].strip()
            
            from roboboat_2026.report.report_body import ObjectDetectedMsg
            msg = ObjectDetectedMsg(obj_type, color, (lat, lon), obj_id, task)
            
            if msg is not None:
                self.msg_queue.append(msg)
                self.get_logger().info(f"ObjectDetected added: {obj_type} {color} at ({lat}, {lon})")
            else:
                self.get_logger().warn(f"Invalid ObjectDetected parameters")
                
        except ValueError as e:
            self.get_logger().error(f"Error parsing ObjectDetected values: {e}")
        except Exception as e:
            self.get_logger().error(f"Error in _parse_object_detected: {e}")

    def _parse_object_delivery(self, parts):
        """
        Parse ObjectDelivery message
        Format: "ObjectDelivery,COLOR,lat,lon,DELIVERY_TYPE"
        Example: "ObjectDelivery,GREEN,32.112345,-21.12345,WATER"
        """
        try:
            if len(parts) != 5:
                self.get_logger().warn(f"Invalid ObjectDelivery format: expected 5 parts, got {len(parts)}")
                return
            
            color = parts[1].strip()
            lat = float(parts[2].strip())
            lon = float(parts[3].strip())
            delivery_type = parts[4].strip()
            
            from roboboat_2026.report.report_body import ObjectDeliveryMsg
            msg = ObjectDeliveryMsg(color, (lat, lon), delivery_type)
            
            if msg is not None:
                self.msg_queue.append(msg)
                self.get_logger().info(f"ObjectDelivery added: {color} vessel, {delivery_type} at ({lat}, {lon})")
            else:
                self.get_logger().warn(f"Invalid ObjectDelivery parameters")
                
        except ValueError as e:
            self.get_logger().error(f"Error parsing ObjectDelivery values: {e}")
        except Exception as e:
            self.get_logger().error(f"Error in _parse_object_delivery: {e}")

    def _parse_docking(self, parts):
        """
        Parse Docking message
        Format: "Docking,DOCK,SLIP"
        Example: "Docking,N,1"
        """
        try:
            if len(parts) != 3:
                self.get_logger().warn(f"Invalid Docking format: expected 3 parts, got {len(parts)}")
                return
            
            dock = parts[1].strip()
            slip = parts[2].strip()
            
            from roboboat_2026.report.report_body import DockingMsg
            msg = DockingMsg(dock, slip)
            
            if msg is not None:
                self.msg_queue.append(msg)
                self.get_logger().info(f"Docking added: Dock {dock}, Slip {slip}")
            else:
                self.get_logger().warn(f"Invalid Docking parameters: dock={dock}, slip={slip}")
                
        except Exception as e:
            self.get_logger().error(f"Error in _parse_docking: {e}")

    def _parse_sound_signal(self, parts):
        """
        Parse SoundSignal message
        Format: "SoundSignal,SIGNAL_TYPE,FREQUENCY,TASK"
        Example: "SoundSignal,ONE_BLAST,600,DOCKING"
        """
        try:
            if len(parts) != 4:
                self.get_logger().warn(f"Invalid SoundSignal format: expected 4 parts, got {len(parts)}")
                return
            
            signal_type = parts[1].strip()
            frequency = int(parts[2].strip())
            task = parts[3].strip()
            
            from roboboat_2026.report.report_body import SoundSignalMsg
            msg = SoundSignalMsg(signal_type, frequency, task)
            
            if msg is not None:
                self.msg_queue.append(msg)
                self.get_logger().info(f"SoundSignal added: {signal_type} at {frequency}Hz, task={task}")
            else:
                self.get_logger().warn(f"Invalid SoundSignal parameters")
                
        except ValueError as e:
            self.get_logger().error(f"Error parsing SoundSignal values: {e}")
        except Exception as e:
            self.get_logger().error(f"Error in _parse_sound_signal: {e}")


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