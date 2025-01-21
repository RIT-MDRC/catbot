# SMBUS

SMBUS2 is a library that gives us utility classes for interfacing hardware and this module is a wrapper for the library to make it easier to work with in out project. For more undocumented details go to [project description](https://pypi.org/project/smbus2/).

# Implementation

> [!NOTE]
> Currently, there is only support for i2c features and does not support intefacing other features of the SMBUS library.

The pinconfig will be configurable to create the SMBUS library class. Once that is stored on the context store, we can interface it through 4 functions: write_byte, read_byte, i2c_wrrd, i2c_rdwr. 

write_byte: Write a byte to the smbus2 device.
read_byte: Read bytes from the smbus2 device.
i2c_wrrd: Write and read data from the smbus2 device in that order.
i2c_rdwr: Read bytes or write bytes from the smbus2 device in any order in a unspecified amount given in actions argument.



DATE: 1/21/2025
MEMBER: Hiroto Takeuchi