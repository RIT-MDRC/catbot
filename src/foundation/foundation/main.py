#!/usr/bin/env /workspace/.venv/bin/python

from dataclasses import dataclass, field
import os
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node, Parameter
from .state_management import configure_device
from .component.latch import latch_actions
from .component.motor import motor_actions
from .component.muscle import muscle_actions


latch_actions.USE = True


@dataclass
class DeviceActionSubscriber:
    callback: callable
    topic: str = field(default=None)
    paramType: object = field(default=Parameter.Type.NOT_SET)

    def __post_init__(self):
        assert callable(self.callback), "Callback must be a callable"
        assert hasattr(self.callback, "_ctx"), "Callback must have a context"
        self.topic = self.callback._func_path if self.topic is None else self.topic


# Subscribers
SUBSCRIBER_DEVICE_ACTION_CALLBACKS = [
    {
        "topic": "/motor/step_n",
        "callback": motor_actions.step_n,
        "paramType": Parameter.Type.INTEGER,
    },
    {"topic": "/muscle/contract", "callback": muscle_actions.contract},
    {"topic": "/muscle/relax", "callback": muscle_actions.relax},
]


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
            context = getattr(callback, "_ctx")
            paramType = subscriber.param_type
            for device_name, device in context.store.items():

                def dev_callback(msg):
                    try:
                        callback(
                            device, msg
                        )  # closure issue may arise if context store is changed
                    except Exception as e:
                        self.get_logger().error(f"Error: {e}")

                self.create_subscription(
                    Parameter.Type.NOT_SET if paramType is None else paramType,
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
