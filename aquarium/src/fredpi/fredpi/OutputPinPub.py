# Imports.
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray



# A ROS 2 node that serves as an interface for toggling Fred's output pins.
class OutputPinPub(Node):
    # Constructor.
    def __init__(self):
        # Initialize parent class.
        super().__init__('output_pin_pub')

        # Create publisher for "/output_pin" topic.
        self.__output_pin_pub = self.create_publisher(
            Int8MultiArray,
            'output_pin',
            10,
        )


    # Publishes a message.
    def publish(self, pin_number, pin_power, pin_pulse=None):
        # Convert values to message.
        message = Int8MultiArray(
            data=[pin_number, (1 if pin_power else 0)]
        )

        # Add pulse duration if needed.
        if pin_pulse is not None:
            # Append to data.
            message.data.append(pin_pulse)

        # Publish.
        self.__output_pin_pub.publish(message)
