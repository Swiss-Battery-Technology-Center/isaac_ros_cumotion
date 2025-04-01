#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from tf2_ros import Buffer, TransformListener, TransformException
from isaac_ros_moveit_goal_setter.move_group_client import MoveGroupClient
import rclpy.duration

class HomeNode(Node):
    def __init__(self):
        super().__init__('home_node')

        # 1. Declare parameters
        self._world_frame = self.declare_parameter(
            'world_frame', 'base_link'
        ).get_parameter_value().string_value

        self._home_frame = self.declare_parameter(
            'home_frame', 'home_frame'
        ).get_parameter_value().string_value

        self._grasp_frame = self.declare_parameter(
            'grasp_frame', 'grasp_frame'
        ).get_parameter_value().string_value

        self._plan_timer_period = self.declare_parameter(
            'plan_timer_period', 0.01
        ).get_parameter_value().double_value

        planner_group_name = self.declare_parameter(
            'planner_group_name', 'manipulator'
        ).get_parameter_value().string_value

        pipeline_id = self.declare_parameter(
            'pipeline_id', 'isaac_ros_cumotion'
        ).get_parameter_value().string_value

        planner_id = self.declare_parameter(
            'planner_id', 'cuMotion'
        ).get_parameter_value().string_value

        ee_link = self.declare_parameter(
            'end_effector_link', 'tool_frame'
        ).get_parameter_value().string_value

        # 2. Create MoveIt client
        self.move_group_client = MoveGroupClient(
            self, planner_group_name, pipeline_id, planner_id, ee_link
        )

        # 3. Set up TF buffer/listener
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # 4. Create a simple state machine:
        #    - self._did_home = False until we succeed in going home
        #    - self._did_grasp = False until we succeed in going to the grasp
        self._did_home = False
        self._did_grasp = False

        # 5. Create a timer that attempts to progress state each cycle
        self._timer = self.create_timer(self._plan_timer_period, self._tick)

    def _tick(self):
        """Called periodically by the timer to progress our state machine."""
        if not self._did_home:
            self._try_go_frame(frame=self._home_frame, label='home')
        elif not self._did_grasp:
            self._try_go_frame(frame=self._grasp_frame, label='grasp')
        else:
            # We did everything, so stop the timer
            self.get_logger().info("Already reached home and then grasp pose. Shutting down timer.")
            self.destroy_timer(self._timer)

    def _try_go_frame(self, frame: str, label: str):
        """
        Attempt to plan & move to a specified frame (e.g. home or grasp).
        If successful, we advance our internal state.
        """
        try:
            # Look up transform from world_frame -> target frame
            tf_msg = self._tf_buffer.lookup_transform(
                self._world_frame,
                frame,
                self.get_clock().now().to_msg(),
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
        except TransformException as ex:
            self.get_logger().warning(
                f'No transform from "{self._world_frame}" to "{frame}". '
                f'Will keep trying. TF error: {ex}'
            )
            return

        # Convert the transform to a PoseStamped
        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = self._world_frame
        goal_pose.pose = self._transform_msg_to_pose_msg(tf_msg.transform)

        # Send the goal to MoveIt
        response = self.move_group_client.send_goal_pose(goal_pose)
        if response.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info(f'Reached {label} pose successfully!')
            if label == 'home':
                self._did_home = True
            elif label == 'grasp':
                self._did_grasp = True
        else:
            self.get_logger().warning(
                f'{label} pose was not reachable (MoveIt error code: '
                f'{response.error_code.val}), retrying...'
            )

    def _transform_msg_to_pose_msg(self, transform_msg):
        """
        Convert a geometry_msgs/Transform to geometry_msgs/Pose.
        """
        pose = Pose()
        pose.position.x = transform_msg.translation.x
        pose.position.y = transform_msg.translation.y
        pose.position.z = transform_msg.translation.z

        pose.orientation.x = transform_msg.rotation.x
        pose.orientation.y = transform_msg.rotation.y
        pose.orientation.z = transform_msg.rotation.z
        pose.orientation.w = transform_msg.rotation.w
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = HomeNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, shutting down.\n')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
