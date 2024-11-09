import logging

from component.can import can_bus as canbus_action
from component.motor import odrive_motor_action as odrive_action
from component.motor.odrive_enums import *
from state_management import configure_device
from view.pygame import *

configure_device("src/raspi/pinconfig.json")

can_bus = "can_bus_1"
motor = "odrive_1"
speed = 15

odrive_action.set_controller_mode(motor, ControlMode.VELOCITY_CONTROL, InputMode.VEL_RAMP)
odrive_action.request_set_state(motor, MotorState.CLOSED_LOOP_CONTROL)


def hydrate_screen():
    """Post setup for the screen (after pygame.init() and global variable are set)"""
    logging.info("Hydrating Screen with initial values")
    logging.info("Hydrating Screen with initial values")
    render_row(0, f"Position: {odrive_action.get_current_position(motor)}")
    render_row(1, f"Velocity: {odrive_action.get_current_velocity(motor)}")
    render_left_status(False)
    render_right_status(False)

    update_screen()
    logging.info("Screen Hydrated")
    logging.info("Completed Screen Update Events")


def main():
    """Main program loop"""
    exit = False
    global speed
    while not exit:
        render_row(0, f"Position: {odrive_action.get_current_position(motor)}")
        render_row(1, f"Velocity: {odrive_action.get_current_velocity(motor)}")
        render_row(4, f"Target Speed: {speed}")
        for event in get_keys():
            if is_event_type(event, "down"):
                if is_key_pressed(event, ["a", "left"]):
                    print("left call")
                    if odrive_action.set_target_velocity(motor, -speed):
                        render_left_status(True)
                elif is_key_pressed(event, ["d", "right"]):
                    print("right call")
                    if odrive_action.set_target_velocity(motor, speed):
                        render_right_status(True)
                elif is_key_pressed(event, ["w", "up"]):
                    speed += 1
                elif is_key_pressed(event, ["down"]):
                    speed -= 1
                elif is_key_pressed(event, ["q"]):
                    exit = True
            elif is_event_type(event, "up"):
                if is_key_pressed(event, ["a", "left"]):
                    if odrive_action.set_target_velocity(motor, 0):
                        render_left_status(False)
                elif is_key_pressed(event, ["d", "right"]):
                    if odrive_action.set_target_velocity(motor, 0):
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
