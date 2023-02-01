# Imports.
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import Int8MultiArray
import math
import time



# Main function.
def main():
    # Start up.
    rclpy.init()

    # Create joint positions publisher node.
    joint_positions_node = JointPositionsPub()

    # Create pin output publisher node.
    output_pin_node = OutputPinPub()

    # Create messages for waving.
    left_message = get_joint_state(
        position=[
            0.0,
            0.0,
            0.0,
            math.pi / 4.0,
            0.0,
            0.0,
        ],
        velocity=80,
    )
    right_message = get_joint_state(
        position=[
            0.0,
            0.0,
            0.0,
            math.pi / -4.0,
            0.0,
            0.0,
        ],
        velocity=80,
    )
    reset_message = get_joint_state(
        position=[0.0 for i in range(6)],
        velocity=80,
    )

    # Create messages for toggling the last output pin.
    on_message = Int8MultiArray(
        data=[6, 1],
    )
    off_message = Int8MultiArray(
        data=[6, 0],
    )

    # Send a stream of messages to get ROS 2 to recognize the connection.
    # Also resets the robot's position.
    print('Initializing')
    for i in range(10):
        reset_message.header.stamp = joint_positions_node.get_clock().now().to_msg()
        joint_positions_node.publish(reset_message)
        time.sleep(0.1)

    # Turn on the last output pin.
    print('Turning on pin')
    output_pin_node.publish(on_message)
    time.sleep(0.5)

    # Loop 3 times.
    for i in range(3):
        # Send waving messages.
        print(f"Wave {i + 1}")
        left_message.header.stamp = joint_positions_node.get_clock().now().to_msg()
        joint_positions_node.publish(left_message)
        time.sleep(1.3)
        right_message.header.stamp = joint_positions_node.get_clock().now().to_msg()
        joint_positions_node.publish(right_message)
        time.sleep(1.3)

    # Go back to starting position.
    print('Resetting')
    reset_message.header.stamp = joint_positions_node.get_clock().now().to_msg()
    joint_positions_node.publish(reset_message)
    time.sleep(0.5)

    # Turn off the last output pin.
    print('Turning off pin')
    output_pin_node.publish(off_message)
    time.sleep(0.5)

    # Shut down.
    rclpy.shutdown()



# Runs a publisher for the /joint_positions topic.
class JointPositionsPub(Node):
    # Constructor.
    def __init__(self):
        # Initialize parent class.
        super().__init__('joint_positions_pub')

        # Create a publisher for the /joint_positions topic.
        self.joint_positions_pub = self.create_publisher(JointState, '/joint_positions', 10)

    # Publishes a message.
    def publish(self, message):
        # Publish.
        self.joint_positions_pub.publish(message)



# Runs a publisher for the /output_pin topic.
class OutputPinPub(Node):
    # Constructor.
    def __init__(self):
        # Initialize parent class.
        super().__init__('output_pin_pub')

        # Create a publisher for the /output_pin topic.
        self.output_pin_pub = self.create_publisher(Int8MultiArray, '/output_pin', 10)

    # Publishes a message.
    def publish(self, message):
        # Publish.
        self.output_pin_pub.publish(message)



# Creates a JointState message with header values already defined.
def get_joint_state(velocity=0.0, position=[0.0 for i in range(6)]):
    # Return the JointState.
    return JointState(
        header=Header(),
        name=[
            'joint2_to_joint1',
            'joint3_to_joint2',
            'joint4_to_joint3',
            'joint5_to_joint4',
            'joint6_to_joint5',
            'joint6output_to_joint6',
        ],
        velocity=[velocity],
        position=position,
    )



# Boilerplate:
if __name__ == '__main__':
    main()
