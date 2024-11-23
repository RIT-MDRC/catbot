import logging

from component.can import can_bus as canbus_action
from component.motor import odrive_motor_action as odrive_action
from component.motor.odrive_enums import *
from state_management import configure_device
from view.pygame import *

configure_device("src/raspi/pinconfig.json")

can_bus = "can_bus_1"
motor = "odrive_1"


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
    left_input = False
    right_input = False
    speed = 5
    use_speed_limit = True
    resting_pos = 0
    
    while not exit:
        change = False
        for event in get_keys():
            if is_event_type(event, "down"):
                if event.key in [pygame.K_a, pygame.K_LEFT]:
                    left_input = True
                    change = True
                elif event.key in [pygame.K_d, pygame.K_RIGHT]:
                    right_input = True
                    change = True
                elif event.key in [pygame.K_w, pygame.K_UP]:
                    speed += 0.1
                    change = True
                elif event.key in [pygame.K_s, pygame.K_DOWN]:
                    speed -= 0.1
                    change = True
                elif event.key in [pygame.K_SPACE]:
                    use_speed_limit = not use_speed_limit
                    change = True
                elif event.key in [pygame.K_q]:
                    exit = True
                elif event.key in [pygame.K_r]:
                    odrive_action.reboot(motor)
                elif event.key in [pygame.K_l]:
                    print(odrive_action.get_position_limits(motor))
            elif is_event_type(event, "up"):
                if event.key in [pygame.K_a, pygame.K_LEFT]:
                    left_input = False
                    change = True
                elif event.key in [pygame.K_d, pygame.K_RIGHT]:
                    right_input = False
                    change = True
        
        position = odrive_action.get_current_position(motor)
        velocity = odrive_action.get_current_velocity(motor)
        
        render_row(0, f"Position: {position}")
        render_row(1, f"Velocity: {velocity}")
        render_left_status(left_input)
        render_right_status(right_input)
        render_row(4, f"Speed Limit: {speed if use_speed_limit else 'Uncapped'}")
        
        if odrive_action.get_state(motor) == MotorState.CLOSED_LOOP_CONTROL and (change or (position is not None and abs(position - resting_pos) > 0.1)):
            pos_limits = odrive_action.get_position_limits(motor)

#            if use_speed_limit:
#                if odrive_action.get_input_mode(motor) == InputMode.PASSTHROUGH:
#                    odrive_action.set_controller_mode(motor, ControlMode.POSITION_CONTROL, InputMode.TRAP_TRAJ)
#                odrive_action.set_trajectory_velocity(motor, velocity)
#            else:
#                if odrive_action.get_input_mode(motor) == InputMode.TRAP_TRAJ:
#                    odrive_action.set_controller_mode(motor, ControlMode.POSITION_CONTROL, InputMode.PASSTHROUGH)

            if left_input:
                odrive_action.set_target_position(motor, pos_limits[0])
            elif right_input:
                odrive_action.set_target_position(motor, pos_limits[1])
            else:
                odrive_action.stop(motor)

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
