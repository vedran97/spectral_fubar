import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped
import tf2_ros

class HuskyTransformPublisher(Node):
    def __init__(self):
        super().__init__('husky_transform_publisher')

        # Create a TransformStamped message
        self.transform_msg = TransformStamped()
        self.transform_msg.header.frame_id = 'world'
        self.transform_msg.child_frame_id = 'base_link'

        # Subscribe to the /gazebo/model_states topic
        self.subscription = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10
        )

        # Create a TransformBroadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def model_states_callback(self, msg):
        try:
            # Find the index of the 'husky' model in the received message
            husky_index = msg.name.index('husky')

            # Extract pose information for the 'husky' model
            husky_pose = msg.pose[husky_index]
            # Update the transform message
            self.transform_msg.transform.translation.x = husky_pose.position.x
            self.transform_msg.transform.translation.y = husky_pose.position.y
            self.transform_msg.transform.translation.z = husky_pose.position.z
            self.transform_msg.transform.rotation = husky_pose.orientation

            # Set the timestamp
            self.transform_msg.header.stamp = self.get_clock().now().to_msg()

            # Publish the transform
            self.tf_broadcaster.sendTransform(self.transform_msg)

        except ValueError:
            # 'husky' model not found in the message
            self.get_logger().warn("'husky' model not found in /gazebo/model_states")

def main():
    rclpy.init()
    husky_transform_publisher = HuskyTransformPublisher()
    rclpy.spin(husky_transform_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
