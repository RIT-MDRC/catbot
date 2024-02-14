from gpiozero import DigitalOutputDevice

from ..generic_devices.generic_devices import create_output_device_component

speed_pin_action = create_output_device_component("speedPin")
__all__ = [
    "register_speedPin",
    "set",
]


@speed_pin_action
def set(speedPin: DigitalOutputDevice, speed: int) -> None:
    """
    Turn a valve on.

    Args:
        valve (DigitalOutputDevice): the valve to turn on
    """
    speedPin.value = speed
