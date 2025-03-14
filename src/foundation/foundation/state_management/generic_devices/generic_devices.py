from dataclasses import dataclass
import logging

from gpiozero import DigitalInputDevice, DigitalOutputDevice, PWMOutputDevice

from .._device import device
from .. import create_generic_context, device_parser
from ..utils import (
    FakeDigitalInputDevice,
    FakeDigitalOutputDevice,
    FakePWMOutputDevice,
    is_dev,
)

__all__ = [
    "input_device_ctx",
    "output_device_ctx",
    "pwm_output_device_ctx",
    "analog_input_device_ctx",
    "AnalogInputDevice",
]

input_device_ctx = create_generic_context(
    "input_device",
    (DigitalInputDevice, FakeDigitalInputDevice),
)
"""
Create a new input device component.
allowed device classes: DigitalInputDevice, FakeInputDevice
returns: (device_action, register_device, get_device, get_registered_devices, get_registered_device_names, gloabal_store)
"""


@device_parser(input_device_ctx)
def parse_input_device(config):
    """
    Parse a new input device.

    Args:
        pin_num (int): the pin number of the device

    Returns:
        (DigitalInputDevice) the new input device
    """
    if isinstance(config, int):
        if is_dev():
            logging.info(
                "dev environment detected. Mocking digital input device for pin %s",
                config,
            )
            return FakeDigitalInputDevice(config)
        else:
            return DigitalInputDevice(config)
    config.pop("_identifier")
    if is_dev():
        logging.info(
            "dev environment detected. Mocking digital input device for pin %s", config
        )
        return FakeDigitalInputDevice(**config)
    else:
        return DigitalInputDevice(**config)


output_device_ctx = create_generic_context(
    "output_device", (DigitalOutputDevice, FakeDigitalOutputDevice)
)
"""
Create a new output device component.
allowed device classes: DigitalOutputDevice, FakeOutputDevice
returns: (device_action, register_device, get_device, get_registered_devices, get_registered_device_names, gloabal_store)
"""


@device_parser(output_device_ctx)
def parse_output_device(config):
    """
    Parse a new output device.

    Args:
        pin_num (int): the pin number of the device

    Returns:
        (DigitalOutputDevice) the new output device
    """
    if not isinstance(config, int):
        raise ValueError("Must be a pin number. Got " + str(config))
    if is_dev():
        logging.info(
            "dev environment detected. Mocking digital output device for pin %s", config
        )
        return FakeDigitalOutputDevice(config)
    else:
        return DigitalOutputDevice(config)


pwm_output_device_ctx = create_generic_context(
    "pwm_output_device", (PWMOutputDevice, FakePWMOutputDevice)
)


@device_parser(pwm_output_device_ctx)
def parse_pwm_output_device(config):
    """
    Parse a new pwm output device.

    Args:
        pin_num (int): the pin number of the device

    Returns:
        (PWMOutputDevice) the new pwm output device
    """
    input = {"pin": config} if isinstance(config, int) else config
    input = {k: v for k, v in input.items() if k != "_identifier"}
    if is_dev():
        logging.info(
            "dev environment detected. Mocking PWM output device for pin %s", config
        )
        return FakePWMOutputDevice(**input)
    else:
        return PWMOutputDevice(**input)


@device
@dataclass
class AnalogInputDevice:
    pin: int

    @property
    def value(self):
        raise NotImplementedError("AnalogInputDevice must implement value method")


analog_input_device_ctx = create_generic_context(
    "analog_input_device", (AnalogInputDevice)
)


@device_parser(analog_input_device_ctx)
def parse_analog_input_device(config):
    """
    Parse a new pwm output device.

    Args:
        pin_num (int): the pin number of the device

    Returns:
        (PWMOutputDevice) the new pwm output device
    """
    input = {"pin": config} if isinstance(config, int) else config
    input = {k: v for k, v in input.items() if k != "_identifier"}
    return AnalogInputDevice(**input)
