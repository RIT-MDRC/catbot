# Latch

Latch is a pin expansion device where we can use 5 GPIO pins to make up 12 output GPIO pins. 

## Pins layout
  The latch takes 3 pins for pointing to a specific address on the latch. One for enable pin, and another one for the data pin.

## The address pins
  used to point to a specific GPIO on the latch that we want to work on. The three pins at mapped on a 3 bit number from 0 to 11 where 0 is 000 and 11 is 111. Every address pin that is 1 will be turned on and vice versa.

## Enable pin
  Enable pin acts as a on and off switch to make the device actively change the pins or not. This allows us to change address pins between two numbers without changing the pins in between. The pin is turned up to make the device turned off and turned on to allow the device to change states.

  Currently, there is no delay when the enable pin is high. This might not be an issue for us due to the speed of python, but could be problematic when we optimize speed. Further caution/exploration may need to be put on this part.

## Data pin
  This pin tells the latch which state to turned the gpio pins to. If the data pin is active when the enable pin is off then the gpio pin pointed by the address pins would be set active. The opposite is true for when the data pin is inactive.

# Implementation
The module contains two classes: Latch and VirtualDigitalOutputDevice

Latch is the main dataclass state that needs to be initialized to use the latch device. The latch class can be used to create additional output pins on the device.
VirtualDigitalOutputDevice is the output gpio pins on the latch. The reason why this class exist is to increase compatibility with other modules using the DigitalOutputDevice from gpiozero library. This class is replacable in the configuration file with no extra steps. The reason why it was structured this way is so that it allows us to treat the latch as if it is part of the standard raspberry pi GPIO and not have to worry about the extra pin manipulation in the main or sample scripts. The module also follows an extension architecture because the module will other wise not going to be activated even if the pin configuration is used. Every script that needs the latch will require `USE` variable in the module to be imported. while technically all that needs to be done is to import the module, some code formatter will auto cleanup unused imports and cause issues. That is why USE syntax is put in place. 

# How to use
  Below the digital output pin section declared on the pinconfig, add the latch identifier. Then create the configuration necessary to initialize the latch class. Then the configured amount of Virtual digital IO pins will be available to be referrenced into other components via their identifiers passed into pins dictionary of the latch class merged with the identifier of the latch in this syntax: `{latch identifier}.{output pin identifier}`. These pins will be stored into the gpiozero output pin context store and can be treated as gpiozero's digital output pin.

## Implementation Issues
  Currently, this module is not concurrency proof because it is configured with a binary lock. This is due to rushed development and also unknown compatibility with other library for concurrency like Threads or Asyncio.
  While it is not an issue, the extension architecture is not favorable currently as the script now contains configuration code, we may need a more robust extension architecture friendly support on the pinconfig syntax sugar to make this work better.



DATE: 1/21/2025
MEMBER: Hiroto Takeuchi