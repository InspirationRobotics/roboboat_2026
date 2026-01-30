# This script contains functions to create report body for robocommand
from roboboat_2026.api.robocommand.msgs.report_pb2 import *

def HeartbeatMsg(state:str,lat:float,lon:float,speed:float,heading:float,current_task:str):
    """
    Args:
        state (str): [MANUAL,AUTO,KILLED,UNKNOWN]
        lat (float): Latitude
        lon (float): Longitude
        speed (float): speed in m/s
        heading (float): heading
        current_task(str): [UNKNOWN,NONE,NAV_CHANNEL,SPEED_CHALLENGE,OBJECT_DELIVERY,DOCKING,SOUND_SIGNAL]

    Returns:
        Heartbeat message
    """
    states = ["UNKNOWN","KILLED","MANUAL","AUTO"]
    report_states = [RobotState.STATE_UNKOWN,
                     RobotState.STATE_KILLED,
                     RobotState.STATE_MANUAL,
                     RobotState.STATE_AUTO
                     ]

    tasks =  ['UNKNOWN','NONE','NAV_CHANNEL','SPEED_CHALLENGE','OBJECT_DELIVERY','DOCKING','SOUND_SIGNAL']
    report_tasks = [TaskType.TASK_UNKNOWN,
                    TaskType.TASK_NONE,
                    TaskType.TASK_ENTRY_EXIT,
                    TaskType.TASK_NAV_CHANNEL,
                    TaskType.TASK_SPEED_CHANLLENGE,
                    TaskType.TASK_OBJECT_DELIVERY,
                    TaskType.TASK_DOCKING,
                    TaskType.TASK_SOUND_SIGNAL
                    ]

    if state not in states or current_task not in tasks:
        return None
    
    return Heartbeat(
        state = report_states[states.index(state)],
        position = Latlng(latitude=lat, longitude=lon),
        spd_mps = float(speed),
        heading_deg = float(heading),
        current_task = report_tasks[tasks.index(current_task)],
    )

def GatePassMsg(type, lat, lon):
    """
    Args:
        Type (String): ENTRY or EXIT
        lat (float): Latitude
        lon (float): Longitude

    Returns:
        GatePass message
    """
    if type == "ENTRY":    
        return GatePass(
            type=GateType.GATE_ENTRY,
            position=LatLng(latitude=lat, longitude=lon),
        )

    if type == "EXIT":    
        return GatePass(
            type=GateType.GATE_EXIT,
            position=LatLng(latitude=lat, longitude=lon),
        )

    return None

