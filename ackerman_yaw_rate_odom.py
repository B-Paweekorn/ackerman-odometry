import rclpy
from rclpy.node import Node
from geometry_msgs.msg import (Point, Pose, PoseWithCovariance, Quaternion, Twist, TwistWithCovariance, Vector3, TransformStamped)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Header, Float32
from tf2_ros import TransformBroadcaster
import tf_transformations
import math

class YawrateOdom(Node):
    def __init__(self):
        super().__init__('YawrateOdom')
        queue_size = 10
        self.dt_loop = 0.01
        # Publisher
        self.publisher = self.create_publisher(Odometry, 'odom', queue_size)
        self.timer = self.create_timer(self.dt_loop, self.timer_callback)

        # Subscriber
        self.subscription_imu = self.create_subscription(
            Imu,
            'feedback_imu',
            self.feedback_imu,
            queue_size)
        
        self.subscription_wheelspeed= self.create_subscription(
            Float32,
            'feedback_wheelspeed',
            self.feedback_wheelspeed,
            queue_size
        )

        # Initialize the transform broadcaster
        self.tf_br = TransformBroadcaster(self)
        self.isOdomUpdate = False

        # Initialize odometry variables
        self.odom_output = Odometry(
            header=Header(
                stamp=self.get_clock().now().to_msg(),
                frame_id='odom'
            ),
            child_frame_id='base_footprint',
            pose=PoseWithCovariance(
                pose=Pose(
                    position=Point(
                        x=0.0,
                        y=0.0,
                        z=0.0
                    ),
                    orientation=Quaternion(
                        x=0.0,
                        y=0.0,
                        z=0.0,
                        w=1.0
                    )
                )
            ),
            twist=TwistWithCovariance(
                twist=Twist(
                    linear=Vector3(
                        x=0.0,
                        y=0.0,
                        z=0.0
                    ),
                    angular=Vector3(
                        z=0.0
                    )
                )
            )
        )

        self.relative_yaw = 0.0
        self.wheelspeed = 0.0
        self.last_callback_time = self.get_clock().now()
        self.initial_orientation = None

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
    def feedback_wheelspeed(self, msg):
        self.wheelspeed = msg.data
        
    def feedback_imu(self, msg):
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = tf_transformations.euler_from_quaternion(orientation_list)
        while orientation_list == [0.0, 0.0, 0.0, 0.0] :
            return
        
        if self.initial_orientation is None:
            self.initial_orientation = yaw

        self.relative_yaw = yaw - self.initial_orientation

    def timer_callback(self):
        delta_x = self.wheelspeed * math.cos(self.relative_yaw) * self.dt_loop
        delta_y = self.wheelspeed * math.sin(self.relative_yaw) * self.dt_loop
        delta_th = self.relative_yaw * self.dt_loop

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, self.relative_yaw)
        # Create Odometry message and fill in the data
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()  # Update time stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose.pose = Pose(
            position=Point(x=self.x, y=self.y, z=0.0),
            orientation=Quaternion(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3]
        )
        )
        odom_msg.twist.twist.linear = Vector3(x=self.x, y=self.y, z=0.0)
        odom_msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=self.relative_yaw)

        self.publisher.publish(odom_msg)

        transform = TransformStamped()
        transform.header.stamp = odom_msg.header.stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'  # Make sure it matches the child frame ID in odom_output
        transform.transform.translation.x = odom_msg.pose.pose.position.x
        transform.transform.translation.y = odom_msg.pose.pose.position.y
        transform.transform.translation.z = odom_msg.pose.pose.position.z
        transform.transform.rotation = odom_msg.pose.pose.orientation

        self.tf_br.sendTransform(transform)

        # Publish the odometry message
        self.publisher.publish(self.odom_output)

def main(args=None):
    rclpy.init(args=args)
    pub_odom_node = YawrateOdom()
    rclpy.spin(pub_odom_node)
    pub_odom_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
