import can
import logging
from typing import Callable
from dataclasses import dataclass

from state_management import (
    create_generic_context,
    device,
    device_action,
    device_parser,
)

@device
@dataclass(slots = True)
class CanBus:
    bus: can.Bus
    listeners: dict
    notifier : can.Notifier

    def __init__(self,channel = "can0",interface = "socketcan",config_context = None, ignore_config = False, **kwargs):
        self.bus = can.Bus(channel,interface,config_context,ignore_config,**kwargs)
        self.listeners = {}
        self.notifier = can.Notifier(self.bus, [lambda msg: read_message(self, msg)])

def read_message(can_bus: CanBus, msg) -> None:
    axisId = msg.arbitration_id >> 5
    if axisId in can_bus.listeners:
        can_bus.listeners[axisId](msg)
    else:
        logging.info("Unhandled message with id " + hex(msg.arbitration_id))

ctx = create_generic_context("can_bus", [CanBus])

@device_parser(ctx)
def parse_can_bus(data: dict) -> CanBus:
    return CanBus(**data)

@device_action(ctx)
def send_message(bus: CanBus, arbitration_id: int, data, is_extended_id: bool = False) -> bool:
    """Sends a message through the can bus. Returns whether the send was successful."""
    msg = can.Message(arbitration_id=arbitration_id, is_extended_id = is_extended_id, data = data)
    try:
        bus.send(msg)
        return True
    except can.CanError:
        return False
    
@device_action(ctx)
def send_message_recurring(bus: CanBus, arbitration_id: int, data, period: float, is_extended_id: bool = False, duration: float = None, modifier_callback: Callable[[object], None] = None) -> object:
    """Repeatedly sends a message through the can bus. Returns a task for it."""
    msg = can.Message(arbitration_id=arbitration_id, is_extended_id = is_extended_id, data = data)
    return bus.send_periodic(msg, period, duration, modifier_callback=modifier_callback)
    
@device_action(ctx)
def add_listener(bus: CanBus, axisID: int, callback: Callable[[object], None]) -> None:
    """Adds a listener to check for messages on a certain axis. There should be one listener per axis."""
    if axisID in bus.listeners:
        logging.warning("Attempting to register multiple listeners for the same CAN Axis when only one allowed. Overriding old listener.")
    bus.listeners[axisID] = callback

@device_action(ctx)
def close(bus: CanBus) -> None:
    bus.notifier.stop()
    bus.bus.shutdown()
