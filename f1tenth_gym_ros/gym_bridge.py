# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Quaternion
from ackermann_msgs.msg import AckermannDrive
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import tf_transformations

from rclpy.qos import qos_profile_sensor_data, QoSProfile

import gym
import yaml
import numpy as np
from transforms3d import euler

class GymBridge(Node):
    def __init__(self):
        super().__init__('gym_bridge')

        self.declare_parameter('ego_namespace', rclpy.Parameter.Type.STRING)
        self.declare_parameter('ego_odom_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('ego_opp_odom_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('ego_scan_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('ego_drive_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('opp_namespace', rclpy.Parameter.Type.STRING)
        self.declare_parameter('opp_odom_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('opp_ego_odom_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('opp_scan_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('opp_drive_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('scan_distance_to_base_link', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('scan_fov', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('scan_beams', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('map_path', rclpy.Parameter.Type.STRING)
        self.declare_parameter('opp', rclpy.Parameter.Type.BOOL)
        self.declare_parameter('sx', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('sy', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('stheta', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('sx1', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('sy1', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('stheta1', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('kb_teleop', rclpy.Parameter.Type.BOOL)

        # check num_agents
        opp = self.get_parameter('opp').value
        # if num_agents < 1 or num_agents > 2:
        #     raise ValueError('num_agents should be either 1 or 2.')
        # elif not isinstance(num_agents, int):
        #     raise ValueError('num_agents should be an int.')
        
        map_yaml = yaml.safe_load(open(self.get_parameter('map_path').value, 'r'))
        # Take out the .yaml
        map = self.get_parameter('map_path').value.split('.')[0]
        # Get the map_ext from the yaml file with key "image"
        map_ext = '.' + map_yaml['image'].split('.')[-1]

        # env backend
        self.env = gym.make('f110_gym:f110-v0',
                            map=map,
                            map_ext=map_ext,
                            num_agents=2 if opp else 1,)
        
        params_dict = {
            'mu': 1.0489,
            'C_Sf': 4.718,
            'C_Sr': 5.4562,
            'lf': 0.1795,
            'lr': 0.1405,
            'h': 0.074,
            'm': 3.5,
            'I': 0.0612,
            's_min': -0.4189,
            's_max': 0.4189,
            'sv_min': -3.2,
            'sv_max': 3.2,
            'v_switch':7.319,
            'a_max': 9.51,
            'v_min':-5.0,
            'v_max': 20.0,
            'width': 0.31,
            'length': 0.58
        }

        sx = self.get_parameter('sx').value
        sy = self.get_parameter('sy').value
        stheta = self.get_parameter('stheta').value
        self.ego_pose = [sx, sy, stheta]
        self.ego_speed = [0.0, 0.0, 0.0]
        self.ego_requested_speed = 0.0
        self.ego_steer = 0.0
        self.ego_collision = False
        self.ego_namespace = self.get_parameter('ego_namespace').value
        ego_scan_topic = self.ego_namespace + '/' + self.get_parameter('ego_scan_topic').value
        ego_drive_topic = self.ego_namespace + '/' + self.get_parameter('ego_drive_topic').value
        scan_fov = self.get_parameter('scan_fov').value
        scan_beams = self.get_parameter('scan_beams').value
        self.angle_min = -scan_fov / 2.
        self.angle_max = scan_fov / 2.
        self.angle_inc = scan_fov / scan_beams
        ego_odom_topic = self.ego_namespace + '/' + self.get_parameter('ego_odom_topic').value
        self.scan_distance_to_base_link = self.get_parameter('scan_distance_to_base_link').value
        self.last_ego_command_time = self.get_clock().now()
        self.last_opp_command_time = self.get_clock().now()
        
        if opp:
            self.has_opp = True
            self.opp_namespace = self.get_parameter('opp_namespace').value
            sx1 = self.get_parameter('sx1').value
            sy1 = self.get_parameter('sy1').value
            stheta1 = self.get_parameter('stheta1').value
            self.opp_pose = [sx1, sy1, stheta1]
            self.opp_speed = [0.0, 0.0, 0.0]
            self.opp_requested_speed = 0.0
            self.opp_steer = 0.0
            self.opp_collision = False
            self.obs, _ , self.done, _ = self.env.reset(np.array([[sx, sy, stheta], [sx1, sy1, stheta1]]))
            self.ego_scan = list(self.obs['scans'][0])
            self.opp_scan = list(self.obs['scans'][1])

            opp_scan_topic = self.opp_namespace + '/' + self.get_parameter('opp_scan_topic').value
            opp_odom_topic = self.opp_namespace + '/' + self.get_parameter('opp_odom_topic').value
            opp_drive_topic = self.opp_namespace + '/' + self.get_parameter('opp_drive_topic').value

            ego_opp_odom_topic = self.ego_namespace + '/' + self.get_parameter('ego_opp_odom_topic').value
            opp_ego_odom_topic = self.opp_namespace + '/' + self.get_parameter('opp_ego_odom_topic').value
        else:
            self.has_opp = False
            self.obs, _ , self.done, _ = self.env.reset(np.array([[sx, sy, stheta]]))
            self.ego_scan = list(self.obs['scans'][0])

        # sim physical step timer
        self.drive_timer = self.create_timer(0.01, self.drive_timer_callback)
        # topic publishing timer
        self.timer = self.create_timer(0.004, self.timer_callback)

        # transform broadcaster
        self.br = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # publishers
        self.ego_scan_pub = self.create_publisher(LaserScan, ego_scan_topic, 10)
        self.ego_odom_pub = self.create_publisher(Odometry, ego_odom_topic, 10)
        self.ego_drive_published = False
        if opp:
            self.opp_scan_pub = self.create_publisher(LaserScan, opp_scan_topic, 10)
            self.ego_opp_odom_pub = self.create_publisher(Odometry, ego_opp_odom_topic, 10)
            self.opp_odom_pub = self.create_publisher(Odometry, opp_odom_topic, 10)
            self.opp_ego_odom_pub = self.create_publisher(Odometry, opp_ego_odom_topic, 10)
            self.opp_drive_published = False

        # subscribers
        self.ego_drive_sub = self.create_subscription(
            AckermannDrive,
            ego_drive_topic,
            self.drive_callback,
            10)
        self.ego_reset_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.ego_reset_callback,
            10)
        if opp:
            self.opp_drive_sub = self.create_subscription(
                AckermannDrive,
                opp_drive_topic,
                self.opp_drive_callback,
                10)
            self.opp_reset_sub = self.create_subscription(
                PoseStamped,
                '/goal_pose',
                self.opp_reset_callback,
                10)

        # if self.get_parameter('kb_teleop').value:
        #     self.teleop_sub = self.create_subscription(
        #         Twist,
        #         '/cmd_vel',
        #         self.teleop_callback,
        #         10)


    def drive_callback(self, drive_msg):
        self.last_ego_command_time = self.get_clock().now()
        self.ego_requested_speed = drive_msg.speed
        self.ego_steer = drive_msg.steering_angle
        self.ego_drive_published = True

    def opp_drive_callback(self, drive_msg):
        self.last_opp_command_time = self.get_clock().now()
        self.opp_requested_speed = drive_msg.speed
        self.opp_steer = drive_msg.steering_angle
        self.opp_drive_published = True

    def ego_reset_callback(self, pose_msg):
        rx = pose_msg.pose.pose.position.x
        ry = pose_msg.pose.pose.position.y
        rqx = pose_msg.pose.pose.orientation.x
        rqy = pose_msg.pose.pose.orientation.y
        rqz = pose_msg.pose.pose.orientation.z
        rqw = pose_msg.pose.pose.orientation.w
        _, _, rtheta = euler.quat2euler([rqw, rqx, rqy, rqz], axes='sxyz')
        if self.has_opp:
            opp_pose = [self.obs['poses_x'][1], self.obs['poses_y'][1], self.obs['poses_theta'][1]]
            self.obs, _ , self.done, _ = self.env.reset(np.array([[rx, ry, rtheta], opp_pose]))
        else:
            self.obs, _ , self.done, _ = self.env.reset(np.array([[rx, ry, rtheta]]))

    def opp_reset_callback(self, pose_msg):
        if self.has_opp:
            rx = pose_msg.pose.position.x
            ry = pose_msg.pose.position.y
            rqx = pose_msg.pose.orientation.x
            rqy = pose_msg.pose.orientation.y
            rqz = pose_msg.pose.orientation.z
            rqw = pose_msg.pose.orientation.w
            _, _, rtheta = euler.quat2euler([rqw, rqx, rqy, rqz], axes='sxyz')
            self.obs, _ , self.done, _ = self.env.reset(np.array([list(self.ego_pose), [rx, ry, rtheta]]))
    def teleop_callback(self, twist_msg):
        if not self.ego_drive_published:
            self.ego_drive_published = True

        self.ego_requested_speed = twist_msg.linear.x

        if twist_msg.angular.z > 0.0:
            self.ego_steer = 0.3
        elif twist_msg.angular.z < 0.0:
            self.ego_steer = -0.3
        else:
            self.ego_steer = 0.0

    def drive_timer_callback(self):
        # if last_ego_command_time is more than 0.25s ago, stop the car
        if (self.get_clock().now() - self.last_ego_command_time).nanoseconds / 1e9 > 0.25:
            self.ego_requested_speed = 0.0
            self.ego_steer = 0.0
            self.ego_drive_published = False
        if self.has_opp and (self.get_clock().now() - self.last_opp_command_time).nanoseconds / 1e9 > 0.25:
            self.opp_requested_speed = 0.0
            self.opp_steer = 0.0
            self.opp_drive_published = False
        if not self.has_opp:
            if self.ego_drive_published:
                self.obs, _, self.done, _ = self.env.step(np.array([[self.ego_steer, self.ego_requested_speed]]))
        else:
            if self.ego_drive_published and self.opp_drive_published:
                self.obs, _, self.done, _ = self.env.step(np.array([[self.ego_steer, self.ego_requested_speed], [self.opp_steer, self.opp_requested_speed]]))
            elif self.ego_drive_published:
                self.obs, _, self.done, _ = self.env.step(np.array([[self.ego_steer, self.ego_requested_speed], [0.0, 0.0]]))
            elif self.opp_drive_published:
                self.obs, _, self.done, _ = self.env.step(np.array([[0.0, 0.0], [self.opp_steer, self.opp_requested_speed]]))
        self._update_sim_state()

    def timer_callback(self):
        ts = self.get_clock().now().to_msg()

        # pub scans
        scan = LaserScan()
        scan.header.stamp = ts
        scan.header.frame_id = self.ego_namespace + '/laser'
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_inc
        scan.range_min = 0.
        scan.range_max = 30.
        scan.ranges = self.ego_scan
        self.ego_scan_pub.publish(scan)

        if self.has_opp:
            opp_scan = LaserScan()
            opp_scan.header.stamp = ts
            opp_scan.header.frame_id = self.opp_namespace + '/laser'
            opp_scan.angle_min = self.angle_min
            opp_scan.angle_max = self.angle_max
            opp_scan.angle_increment = self.angle_inc
            opp_scan.range_min = 0.
            opp_scan.range_max = 30.
            opp_scan.ranges = self.opp_scan
            self.opp_scan_pub.publish(opp_scan)

        # pub tf
        self._publish_odom(ts)
        self._publish_transforms(ts)
        self._publish_laser_transforms(ts)
        # self._publish_wheel_transforms(ts)

    def _update_sim_state(self):
        self.ego_scan = list(self.obs['scans'][0])
        if self.has_opp:
            self.opp_scan = list(self.obs['scans'][1])
            self.opp_pose[0] = self.obs['poses_x'][1]
            self.opp_pose[1] = self.obs['poses_y'][1]
            self.opp_pose[2] = self.obs['poses_theta'][1]
            self.opp_speed[0] = self.obs['linear_vels_x'][1]
            self.opp_speed[1] = self.obs['linear_vels_y'][1]
            self.opp_speed[2] = self.obs['ang_vels_z'][1]

        self.ego_pose[0] = self.obs['poses_x'][0]
        self.ego_pose[1] = self.obs['poses_y'][0]
        self.ego_pose[2] = self.obs['poses_theta'][0]
        self.ego_speed[0] = self.obs['linear_vels_x'][0]
        self.ego_speed[1] = self.obs['linear_vels_y'][0]
        self.ego_speed[2] = self.obs['ang_vels_z'][0]

    def _publish_odom(self, ts):
        ego_odom = Odometry()
        ego_odom.header.stamp = ts
        ego_odom.header.frame_id = 'map'
        ego_odom.child_frame_id = self.ego_namespace + '/base_link'
        ego_odom.pose.pose.position.x = self.ego_pose[0]
        ego_odom.pose.pose.position.y = self.ego_pose[1]
        ego_quat = euler.euler2quat(0., 0., self.ego_pose[2], axes='sxyz')
        ego_odom.pose.pose.orientation.x = ego_quat[1]
        ego_odom.pose.pose.orientation.y = ego_quat[2]
        ego_odom.pose.pose.orientation.z = ego_quat[3]
        ego_odom.pose.pose.orientation.w = ego_quat[0]
        ego_odom.twist.twist.linear.x = self.ego_speed[0]
        ego_odom.twist.twist.linear.y = self.ego_speed[1]
        ego_odom.twist.twist.angular.z = self.ego_speed[2]
        self.ego_odom_pub.publish(ego_odom)

        if self.has_opp:
            opp_odom = Odometry()
            opp_odom.header.stamp = ts
            opp_odom.header.frame_id = 'map'
            opp_odom.child_frame_id = self.opp_namespace + '/base_link'
            opp_odom.pose.pose.position.x = self.opp_pose[0]
            opp_odom.pose.pose.position.y = self.opp_pose[1]
            opp_quat = euler.euler2quat(0., 0., self.opp_pose[2], axes='sxyz')
            opp_odom.pose.pose.orientation.x = opp_quat[1]
            opp_odom.pose.pose.orientation.y = opp_quat[2]
            opp_odom.pose.pose.orientation.z = opp_quat[3]
            opp_odom.pose.pose.orientation.w = opp_quat[0]
            opp_odom.twist.twist.linear.x = self.opp_speed[0]
            opp_odom.twist.twist.linear.y = self.opp_speed[1]
            opp_odom.twist.twist.angular.z = self.opp_speed[2]
            self.opp_odom_pub.publish(opp_odom)
            self.opp_ego_odom_pub.publish(ego_odom)
            self.ego_opp_odom_pub.publish(opp_odom)

    def _publish_transforms(self, ts):
        # ego_t = Transform()
        # ego_t.translation.x = self.ego_pose[0]
        # ego_t.translation.y = self.ego_pose[1]
        # ego_t.translation.z = 0.0
        # ego_quat = euler.euler2quat(0.0, 0.0, self.ego_pose[2], axes='sxyz')
        # ego_t.rotation.x = ego_quat[1]
        # ego_t.rotation.y = ego_quat[2]
        # ego_t.rotation.z = ego_quat[3]
        # ego_t.rotation.w = ego_quat[0]

        # ego_ts = TransformStamped()
        # ego_ts.transform = ego_t
        # ego_ts.header.stamp = ts
        # ego_ts.header.frame_id = 'map'
        # ego_ts.child_frame_id = self.ego_namespace + '/base_link'
        # self.br.sendTransform(ego_ts)

        if self.has_opp:
            # Get transformation from map to self.opp_namespace/odom
            try:
                if not self.tf_buffer.can_transform(self.opp_namespace + '/odom', self.opp_namespace + '/base_link', rclpy.time.Time()):
                    self.get_logger().info('Waiting for transformation from ' + self.opp_namespace + '/odom to ' + self.opp_namespace + '/base_link ...', throttle_duration_sec=1.0)
                    return
                opp_odom_base_link = self.tf_buffer.lookup_transform(self.opp_namespace + '/odom', self.opp_namespace + '/base_link', rclpy.time.Time())
                opp_base_link_odom_transform = tf_transformations.inverse_matrix(
                    tf_transformations.concatenate_matrices(
                        tf_transformations.translation_matrix([opp_odom_base_link.transform.translation.x, opp_odom_base_link.transform.translation.y, opp_odom_base_link.transform.translation.z]),
                        tf_transformations.quaternion_matrix([opp_odom_base_link.transform.rotation.x, opp_odom_base_link.transform.rotation.y, opp_odom_base_link.transform.rotation.z, opp_odom_base_link.transform.rotation.w])
                    )
                )
                opp_quat = euler.euler2quat(0., 0., self.opp_pose[2], axes='sxyz')
                opp_map_base_link_transform = tf_transformations.concatenate_matrices(
                    tf_transformations.translation_matrix([self.opp_pose[0], self.opp_pose[1], 0.0]),
                    tf_transformations.quaternion_matrix([opp_quat[1], opp_quat[2], opp_quat[3], opp_quat[0]])
                )
                opp_map_odom_transform = tf_transformations.concatenate_matrices(
                    opp_map_base_link_transform,
                    opp_base_link_odom_transform
                )

                opp_map_odom_translation = tf_transformations.translation_from_matrix(opp_map_odom_transform)
                opp_map_odom_quaternion = tf_transformations.quaternion_from_matrix(opp_map_odom_transform)
                opp_map_odom = TransformStamped()
                opp_map_odom.transform.translation.x = opp_map_odom_translation[0]
                opp_map_odom.transform.translation.y = opp_map_odom_translation[1]
                opp_map_odom.transform.translation.z = opp_map_odom_translation[2]
                opp_map_odom.transform.rotation.x = opp_map_odom_quaternion[0]
                opp_map_odom.transform.rotation.y = opp_map_odom_quaternion[1]
                opp_map_odom.transform.rotation.z = opp_map_odom_quaternion[2]
                opp_map_odom.transform.rotation.w = opp_map_odom_quaternion[3]
                opp_map_odom.header.stamp = opp_odom_base_link.header.stamp
                opp_map_odom.header.frame_id = 'map'
                opp_map_odom.child_frame_id = self.opp_namespace + '/odom'
                self.br.sendTransform(opp_map_odom)
            except Exception as e:
                self.get_logger().error('Failed to get transformation from map to ' + self.opp_namespace + '/odom: ' + str(e))
            

    def _publish_wheel_transforms(self, ts):
        # ego_wheel_ts = TransformStamped()
        # ego_wheel_quat = euler.euler2quat(0., 0., self.ego_steer, axes='sxyz')
        # ego_wheel_ts.transform.rotation.x = ego_wheel_quat[1]
        # ego_wheel_ts.transform.rotation.y = ego_wheel_quat[2]
        # ego_wheel_ts.transform.rotation.z = ego_wheel_quat[3]
        # ego_wheel_ts.transform.rotation.w = ego_wheel_quat[0]
        # ego_wheel_ts.header.stamp = ts
        # ego_wheel_ts.header.frame_id = self.ego_namespace + '/chassis_link'
        # ego_wheel_ts.child_frame_id = self.ego_namespace + '/virtual_front_wheel'
        # self.br.sendTransform(ego_wheel_ts)
        # ego_wheel_ts.header.frame_id = self.ego_namespace + '/front_right_hinge'
        # ego_wheel_ts.child_frame_id = self.ego_namespace + '/front_right_wheel'
        # self.br.sendTransform(ego_wheel_ts)

        if self.has_opp:
            opp_wheel_ts = TransformStamped()
            opp_wheel_quat = euler.euler2quat(0., 0., self.opp_steer, axes='sxyz')
            opp_wheel_ts.transform.rotation.x = opp_wheel_quat[1]
            opp_wheel_ts.transform.rotation.y = opp_wheel_quat[2]
            opp_wheel_ts.transform.rotation.z = opp_wheel_quat[3]
            opp_wheel_ts.transform.rotation.w = opp_wheel_quat[0]
            opp_wheel_ts.header.stamp = ts
            opp_wheel_ts.header.frame_id = self.opp_namespace + '/chassis_link'
            opp_wheel_ts.child_frame_id = self.opp_namespace + '/front_left_wheel'
            self.br.sendTransform(opp_wheel_ts)
            opp_wheel_ts.header.frame_id = self.opp_namespace + '/front_right_hinge'
            opp_wheel_ts.child_frame_id = self.opp_namespace + '/front_right_wheel'
            self.br.sendTransform(opp_wheel_ts)

    def _publish_laser_transforms(self, ts):
        ego_scan_ts = TransformStamped()
        ego_scan_ts.transform.translation.x = self.scan_distance_to_base_link
        # ego_scan_ts.transform.translation.z = 0.04+0.1+0.025
        ego_scan_ts.transform.rotation.w = 1.
        ego_scan_ts.header.stamp = ts
        ego_scan_ts.header.frame_id = self.ego_namespace + '/base_link'
        ego_scan_ts.child_frame_id = self.ego_namespace + '/laser'
        self.br.sendTransform(ego_scan_ts)

        if self.has_opp:
            opp_scan_ts = TransformStamped()
            opp_scan_ts.transform.translation.x = self.scan_distance_to_base_link
            opp_scan_ts.transform.rotation.w = 1.
            opp_scan_ts.header.stamp = ts
            opp_scan_ts.header.frame_id = self.opp_namespace + '/base_link'
            opp_scan_ts.child_frame_id = self.opp_namespace + '/laser'
            self.br.sendTransform(opp_scan_ts)

def main(args=None):
    rclpy.init(args=args)
    gym_bridge = GymBridge()
    rclpy.spin(gym_bridge)

if __name__ == '__main__':
    main()
