# Imports.
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState



# A ROS 2 node that serves as an interface for receiving Fred's joint position
# values.
class GetJointsSub(Node):
    # Constructor.
    def __init__(self):
        # Initialize parent class.
        super().__init__('get_joints_sub')

        # Create subscriber for "/get_joints" topic.
        self.create_subscription(
            JointState,
            'get_joints',
            self.__gj_callback,
            10,
        )

        # Prepare data value.
        self.__position_list = [None for i in range(6)]


    # Gets one message from the subscriber.
    def subscribe(self):
        # Wait for a message to be received.
        rclpy.spin_once(self)

        # Return data value.
        return self.__position_list


    # Callback function for "/get_joints" subscriber.
    def __gj_callback(self, message):
        # Update data value.
        self.__position_list = [p for p in message.position]
