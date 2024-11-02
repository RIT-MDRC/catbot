import logging

from component.can import can_bus as canbus_action
from component.motor import odrive_motor_action as odrive_action
from component.motor.odrive_enums import *
from state_management import configure_device
from view.pygame import *

configure_device("src/raspi/pinconfig.json")

can_bus = "can_bus_1"
motor = "odrive_1"

odrive_action.request_set_state(motor, MotorState.FULL_CALIBRATION_SEQUENCE)

while odrive_action.get_state(motor) != MotorState.IDLE:
    print(f"Current state: {odrive_action.get_state(motor)}.")
    print(f"Current position: {odrive_action.get_current_position(motor)}.")
    print(f"Current velocity: {odrive_action.get_current_velocity(motor)}.")
    print("")
    pass

def hydrate_screen():
    """Post setup for the screen (after pygame.init() and global variable are set)"""
    logging.info("Hydrating Screen with initial values")
    logging.info("Hydrating Screen with initial values")
    render_left_status(False)
    render_right_status(False)
    update_screen()
    logging.info("Screen Hydrated")
    logging.info("Completed Screen Update Events")

LEFT_SPEED = 0.1  # unit: rev/s
RIGHT_SPEED = -0.1  # unit: rev/s

def main():
    """Main program loop"""
    exit = False
    while not exit:
        for event in get_keys():
            if is_event_type(event, "down"):
                if is_key_pressed(event, ["a", "left"]):
                    print("left call")
                    if odrive_action.set_target_velocity(motor, LEFT_SPEED):
                        render_left_status(True)
                elif is_key_pressed(event, ["d", "right"]):
                    print("right call")
                    if odrive_action.set_target_velocity(motor, RIGHT_SPEED):
                        render_right_status(True)
                elif is_key_pressed(event, ["q"]):
                    exit = True
            elif is_event_type(event, "up"):
                if is_key_pressed(event, ["a", "left"]):
                    if odrive_action.stop(motor):
                        render_left_status(False)
                elif is_key_pressed(event, ["d", "right"]):
                    if odrive_action.stop(motor):
                        render_right_status(False)
        update_screen()
        clock_tick(60)
    print("Exiting...")
    logging.info("Exiting...")
    quit_pygame()
    canbus_action.close(can_bus)

print("Initializing...")
setup_pygame()  # global variables
hydrate_screen()  # hydrate the screen
print("Initialization complete!")
main()
