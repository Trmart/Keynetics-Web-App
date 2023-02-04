# Imports.
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header



# A ROS 2 node that serves as an interface for sending joint positions to Fred.
class JointPositionsPub(Node):
    # Constructor.
    def __init__(self):
        # Initialize parent class.
        super().__init__('joint_positions_pub')

        # Create publisher for "/joint_positions" topic.
        self.__joint_positions_pub = self.create_publisher(
            JointState,
            'joint_positions',
            10,
        )


    # Publishes a message.
    def publish(self, position_list, velocity):
        # Convert values to message.
        message = self.__get_joint_state(position_list, velocity)

        # Publish.
        self.__joint_positions_pub.publish(message)


    # Creates a JointState message with header values already defined.
    def __get_joint_state(self, position_list=[0.0 for i in range(6)], velocity=0.0):
        # Return the JointState.
        return JointState(
            header=Header(
                stamp=self.get_clock().now().to_msg()
            ),
            name=[
                'joint2_to_joint1',
                'joint3_to_joint2',
                'joint4_to_joint3',
                'joint5_to_joint4',
                'joint6_to_joint5',
                'joint6output_to_joint6',
            ],
            velocity=[velocity],
            position=position_list,
        )
