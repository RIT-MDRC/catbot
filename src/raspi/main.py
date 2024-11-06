import concurrent.futures
import time

import rclpy
from rclpy.node import Node
from state_management import configure_device
from state_management.utils.interval import clear_all
from std_msgs.msg import String  # Example message type, customize for your use case
from view.textualUI.main import Main_UI


class MultiThreadedEventLoop:
    def __init__(self, max_workers=4):
        self.executor = concurrent.futures.ThreadPoolExecutor(max_workers=max_workers)
        self.tasks = []
        self.keep_running = True

    def add_task(self, func, *args, **kwargs):
        """
        Add a task to the event loop. The task should be a function, and it can accept
        any arguments and keyword arguments.
        """
        task = self.executor.submit(func, *args, **kwargs)
        self.tasks.append(task)

    def run(self):
        """
        Continuously run the event loop, checking for tasks completion and clearing them.
        """
        while self.keep_running:
            # Check for completed tasks
            for task in concurrent.futures.as_completed(self.tasks):
                try:
                    result = task.result()  # Retrieve result if needed
                    print(f"Task completed with result: {result}")
                except Exception as e:
                    print(f"Task generated an exception: {e}")
            # Clear completed tasks
            self.tasks = [task for task in self.tasks if not task.done()]
            time.sleep(0.1)  # Small sleep to prevent busy waiting

    def shutdown(self):
        """
        Shutdown the executor when done.
        """
        self.keep_running = False
        self.executor.shutdown(wait=True)


# Example task function
def sample_task(data):
    print(f"Task received data: {data}")
    time.sleep(2)  # Simulating a time-consuming task
    print(f"Task processing completed for data: {data}")
    return data


class TaskListenerNode(Node):
    def __init__(self, event_loop):
        super().__init__("task_listener_node")
        self.event_loop = event_loop
        # Set up ROS 2 subscriber to trigger tasks on message reception
        self.subscription = self.create_subscription(
            String, "task_topic", self.ros_callback, 10
        )
        self.get_logger().info(
            "TaskListenerNode has been started and is listening to the 'task_topic'."
        )

    def ros_callback(self, msg):
        self.get_logger().info(f"Received message: {msg.data}")
        # Add a new task based on incoming message
        self.event_loop.add_task(sample_task, msg.data)


def main(args=None):
    rclpy.init(args=args)

    # Create an instance of the event loop
    event_loop = MultiThreadedEventLoop(max_workers=4)

    # Create the ROS 2 node
    task_listener_node = TaskListenerNode(event_loop)

    try:
        # Run the event loop in a separate thread
        loop_thread = concurrent.futures.ThreadPoolExecutor(max_workers=1)
        loop_thread.submit(event_loop.run)

        # Spin the ROS 2 node
        rclpy.spin(task_listener_node)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        # Shutdown the event loop and the ROS node
        event_loop.shutdown()
        loop_thread.shutdown(wait=True)
        task_listener_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


# def main():
#     try:
#         Main_UI().run()
#     except Exception as e:
#         clear_all()
#         raise


# if __name__ == "__main__":
#     configure_device("src/raspi/pinconfig.json")
#     main()
