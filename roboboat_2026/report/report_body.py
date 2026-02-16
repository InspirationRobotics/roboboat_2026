# This script contains functions to create report body for robocommand
from roboboat_2026.report.msgs.report_pb2 import *

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
    report_states = [RobotState.STATE_UNKNOWN,
                     RobotState.STATE_KILLED,
                     RobotState.STATE_MANUAL,
                     RobotState.STATE_AUTO
                     ]

    tasks =  ['UNKNOWN','NONE','ENTRY_EXIT','NAV_CHANNEL','SPEED_CHALLENGE','OBJECT_DELIVERY','DOCKING','SOUND_SIGNAL']
    report_tasks = [TaskType.TASK_UNKNOWN,
                    TaskType.TASK_NONE,
                    TaskType.TASK_ENTRY_EXIT,
                    TaskType.TASK_NAV_CHANNEL,
                    TaskType.TASK_SPEED_CHALLENGE,
                    TaskType.TASK_OBJECT_DELIVERY,
                    TaskType.TASK_DOCKING,
                    TaskType.TASK_SOUND_SIGNAL
                    ]

    if state not in states or current_task not in tasks:
        return None
    
    return Heartbeat(
        state = report_states[states.index(state)],
        position = LatLng(latitude=lat, longitude=lon),
        spd_mps = float(speed),
        heading_deg = float(heading),
        current_task = report_tasks[tasks.index(current_task)],
    )

def GatePassMsg(type, lat, lon):
    """
    Args:
        type (str): [ENTRY, EXIT, SPEED_START, SPEED_END]
        lat (float): Latitude
        lon (float): Longitude

    Returns:
        GatePass message
    """
    gate_types = {
        "UNKNOWN": GateType.GATE_UNKNOWN,
        "ENTRY": GateType.GATE_ENTRY,
        "EXIT": GateType.GATE_EXIT,
        "SPEED_START": GateType.GATE_SPEED_START,
        "SPEED_END": GateType.GATE_SPEED_END
    }
    
    if type not in gate_types:
        return None
    
    return GatePass(
        type=gate_types[type],
        position=LatLng(latitude=lat, longitude=lon),
    )


def ObjectDetectedMsg(type, color, pos, id, task):
    """
    Args:
        type (str): [UNKNOWN, BOAT, LIGHT_BEACON, BUOY]
        color (str): [UNKNOWN, YELLOW, BLACK, RED, GREEN]
        pos (tuple): (latitude, longitude)
        id (int): team-scoped stable ID
        task (str): [UNKNOWN, NONE, ENTRY_EXIT, NAV_CHANNEL, SPEED_CHALLENGE, OBJECT_DELIVERY, DOCKING, SOUND_SIGNAL]

    Returns:
        ObjectDetected message
    """
    available_types = {
        "UNKNOWN": ObjectType.OBJECT_UNKNOWN, 
        "BOAT": ObjectType.OBJECT_BOAT,
        "LIGHT_BEACON": ObjectType.OBJECT_LIGHT_BEACON,
        "BUOY": ObjectType.OBJECT_BUOY
    }
    
    available_colors = {
        "UNKNOWN": Color.COLOR_UNKNOWN,
        "YELLOW": Color.COLOR_YELLOW,
        "BLACK": Color.COLOR_BLACK,
        "RED": Color.COLOR_RED,
        "GREEN": Color.COLOR_GREEN
    }
    
    available_tasks = {
        "UNKNOWN": TaskType.TASK_UNKNOWN,
        "NONE": TaskType.TASK_NONE,
        "ENTRY_EXIT": TaskType.TASK_ENTRY_EXIT,
        "NAV_CHANNEL": TaskType.TASK_NAV_CHANNEL,
        "SPEED_CHALLENGE": TaskType.TASK_SPEED_CHALLENGE,
        "OBJECT_DELIVERY": TaskType.TASK_OBJECT_DELIVERY,
        "DOCKING": TaskType.TASK_DOCKING,
        "SOUND_SIGNAL": TaskType.TASK_SOUND_SIGNAL
    }
    
    if type not in available_types or color not in available_colors or task not in available_tasks:
        return None
    
    return ObjectDetected(
        object_type=available_types[type],
        color=available_colors[color],
        position=LatLng(latitude=pos[0], longitude=pos[1]),
        object_id=int(id),
        task_context=available_tasks[task]
    )


def ObjectDeliveryMsg(color, pos, delivery_type):
    """
    Args:
        color (str): [UNKNOWN, YELLOW, BLACK, RED, GREEN]
        pos (tuple): (latitude, longitude)
        delivery_type (str): [UNKNOWN, WATER, BALL]

    Returns:
        ObjectDelivery message
    """
    available_colors = {
        "UNKNOWN": Color.COLOR_UNKNOWN,
        "YELLOW": Color.COLOR_YELLOW,
        "BLACK": Color.COLOR_BLACK,
        "RED": Color.COLOR_RED,
        "GREEN": Color.COLOR_GREEN
    }
    
    available_delivery_types = {
        "UNKNOWN": DeliveryType.DELIVERY_UNKNOWN,
        "WATER": DeliveryType.DELIVERY_WATER,
        "BALL": DeliveryType.DELIVERY_BALL
    }
    
    if color not in available_colors or delivery_type not in available_delivery_types:
        return None
    
    return ObjectDelivery(
        vessel_color=available_colors[color],
        position=LatLng(latitude=pos[0], longitude=pos[1]),
        delivery_type=available_delivery_types[delivery_type]
    )


def DockingMsg(dock, slip):
    """
    Args:
        dock (str): 'N' or 'S' (North or South)
        slip (str): '1', '2', or '3'

    Returns:
        Docking message
    """
    valid_docks = ['N', 'S']
    valid_slips = ['1', '2', '3']
    
    if dock not in valid_docks or slip not in valid_slips:
        return None
    
    return Docking(
        dock=dock,
        slip=slip
    )


def SoundSignalMsg(signal_type, frequency, task):
    """
    Args:
        signal_type (str): [UNKNOWN, ONE_BLAST, TWO_BLAST]
        frequency (int): nominal frequency (600, 800, or 1000 Hz)
        task (str): [UNKNOWN, NONE, ENTRY_EXIT, NAV_CHANNEL, SPEED_CHALLENGE, OBJECT_DELIVERY, DOCKING, SOUND_SIGNAL]

    Returns:
        SoundSignal message
    """
    available_signal_types = {
        "UNKNOWN": SignalType.SIGNAL_UNKNOWN,
        "ONE_BLAST": SignalType.SIGNAL_ONE_BLAST,
        "TWO_BLAST": SignalType.SIGNAL_TWO_BLAST
    }
    
    available_tasks = {
        "UNKNOWN": TaskType.TASK_UNKNOWN,
        "NONE": TaskType.TASK_NONE,
        "ENTRY_EXIT": TaskType.TASK_ENTRY_EXIT,
        "NAV_CHANNEL": TaskType.TASK_NAV_CHANNEL,
        "SPEED_CHALLENGE": TaskType.TASK_SPEED_CHALLENGE,
        "OBJECT_DELIVERY": TaskType.TASK_OBJECT_DELIVERY,
        "DOCKING": TaskType.TASK_DOCKING,
        "SOUND_SIGNAL": TaskType.TASK_SOUND_SIGNAL
    }
    
    valid_frequencies = [600, 800, 1000]
    
    if signal_type not in available_signal_types or task not in available_tasks or frequency not in valid_frequencies:
        return None
    
    return SoundSignal(
        signal_type=available_signal_types[signal_type],
        frequency_hz=int(frequency),
        assigned_task=available_tasks[task]
    )