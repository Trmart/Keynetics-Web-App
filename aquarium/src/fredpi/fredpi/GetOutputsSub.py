# Imports.
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray



# A ROS 2 node that serves as an interface for receiving Fred's output pin
# values.
class GetOutputsSub(Node):
    # Constructor.
    def __init__(self):
        # Initialize parent class.
        super().__init__('get_outputs_sub')

        # Create subscriber for "/get_outputs" topic.
        self.create_subscription(
            Int8MultiArray,
            'get_outputs',
            self.__go_callback,
            10,
        )

        # Prepare data values.
        self.__output_pins = [-1, -1, -1, -1, -1, -1, -1]


    # Gets one message from the subscriber.
    def subscribe(self):
        # Wait for a message to be received.
        rclpy.spin_once(self)

        # Return data values.
        return self.__output_pins


    # Callback function for "/get_outputs" subscriber.
    def __go_callback(self, message):
        # Update data values.
        self.__output_pins = [o for o in message.data]
