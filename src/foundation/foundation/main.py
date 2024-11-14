#!/usr/bin/env /workspace/.venv/bin/python

from dataclasses import dataclass, field
import logging
import os

from .state_management._device import Context
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from std_msgs.msg import Empty, Int64
from .state_management import configure_device
from .component.latch import latch_actions
from .component.motor import raw_motor_actions
from .component.muscle import muscle_actions


latch_actions.USE = True


@dataclass
class DeviceActionSubscriber:
    topic: str
    callback: callable
    context: Context = field(default=None)
    paramType: object = field(default=Empty)

    def __post_init__(self):
        if self.context is None and hasattr(self.callback, "_ctx"):
            self.context = self.callback._ctx
        self.topic = self.callback._func_path if self.topic is None else self.topic

        assert callable(self.callback), "Callback must be a callable"
        assert self.context is not None, "Callback must have a context"
        assert self.topic is not None, "Topic must be provided"


# Subscribers
SUBSCRIBER_DEVICE_ACTION_CALLBACKS = [
    {
        "topic": "motor/step_n",
        "callback": lambda device, data: raw_motor_actions.step_n(device, data.data),
        "context": raw_motor_actions.ctx,
        "paramType": Int64,
    },
    {
        "topic": "muscle/contract",
        "callback": lambda device, _: muscle_actions.contract(device),
        "context": muscle_actions.ctx,
    },
    {
        "topic": "muscle/relax",
        "callback": lambda device, _: muscle_actions.relax(device),
        "context": muscle_actions.ctx,
    },
]
FOUNDATION_NODE_NAME = "foundation_node"


class DeviceActionSubscriberNode(Node):
    def __init__(self, nodeName, subscribers_info):
        configure_device(os.path.join(f"{os.path.dirname(__file__)}/pinconfig.json"))
        super().__init__(nodeName)

        # Create a Mutex CallbackGroup
        self.callback_group = MutuallyExclusiveCallbackGroup()
        subscribers_info = [
            DeviceActionSubscriber(**subscriber) for subscriber in subscribers_info
        ]
        # Iterate through the list of tuples and create subscribers
        for subscriber in subscribers_info:
            topic = subscriber.topic
            callback = subscriber.callback
            context = subscriber.context
            paramType = subscriber.paramType
            for device_name, device in context.store.items():

                def dev_callback(msg):
                    logging.info(f"Received message: {msg}")
                    print(f"Received message: {msg}")
                    try:
                        callback(
                            device, msg
                        )  # closure issue may arise if context store is changed
                    except Exception as e:
                        logging.error(f"Error: {e}")
                        self.get_logger().error(f"Error: {e}")

                self.create_subscription(
                    Empty if paramType is None else paramType,
                    f"{topic}/{device_name}",  # Topic path
                    dev_callback,  # Callback function
                    10,  # QoS profile depth
                    callback_group=self.callback_group,  # Use the Mutex callback group
                )


def main(args=None):
    rclpy.init(args=args)

    node = DeviceActionSubscriberNode(
        "foundation_node", SUBSCRIBER_DEVICE_ACTION_CALLBACKS
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
