import logging

from state_management import create_generic_context, device_action, device_parser
from state_management.utils import FakeSMBus as SMBus, is_dev

if not is_dev():
    from smbus2 import SMBus

ctx = create_generic_context("smbus2", (SMBus))


@device_parser(ctx)
def parse_smbus2(config):
    """
    Parse a new smbus2 device.

    Args:
        bus_num (int): the bus number of the device

    Returns:
        (SMBus) the new smbus2 device
    """
    if not isinstance(config, int):
        raise ValueError("Must be a bus number. Got " + str(config))
    if is_dev():
        logging.info(
            "dev environment detected. Mocking smbus2 device for bus %s", config
        )
    return SMBus(config)


@device_action(ctx)
def write_byte(smbus2: SMBus, address, value, start_register=0x00) -> None:
    """
    Write a byte to the smbus2 device.

    Args:
        smbus2 (SMBus): the smbus2 device
        address (int): the address to write to
        value (list[int]): the value to write
        start_register (int): the register to start writing to
    """
    smbus2.write_i2c_block_data(address, start_register, value)


@device_action(ctx)
def read_byte(smbus2: SMBus, address, start_register, length=1) -> int:
    """Read bytes from the smbus2 device.

    Args:
        smbus2 (SMBus): SMBus device to use
        address (int): I2C address of the device
        start_register (int): index of the register to start reading from
        length (int, optional): legnth of the data to read. Defaults to 1.

    Returns:
        int: data read from the device
    """
    return smbus2.read_i2c_block_data(address, start_register, length)
