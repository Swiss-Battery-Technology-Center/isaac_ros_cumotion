#!/usr/bin/env python3

import sys
import os
import traceback
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, PoseStamped, Transform
from moveit_msgs.msg import MoveItErrorCodes
from tf2_ros import Buffer, TransformListener, TransformException
from isaac_ros_moveit_goal_setter.move_group_client import MoveGroupClient # Check import path
import rclpy.duration
from std_srvs.srv import Trigger
from control_msgs.action import GripperCommand
from action_msgs.msg import GoalStatus
from threading import Thread
import rclpy
import math
import spatialmath as sm
from isaac_ros_moveit_goal_setter.utilities.spatial_math_conversions import pose_msg_to_se3
from rclpy.executors import MultiThreadedExecutor

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
directories_to_add = ['sbtc-ros2-motion-commanding','sbtc-ros2-utilities']
for directory in directories_to_add:
    dir_path = os.path.join(parent_dir, directory)
    if os.path.isdir(dir_path):
        sys.path.append(dir_path)


from motion_commanding.motion_commander import MotionCommander, PathPlanners
from motion_commanding.robots import Robots

class HomeNode(Node):
    def __init__(self):

        super().__init__('home_node')
        node = rclpy.create_node("test_motion_commander")
        logger = node.get_logger()
        self.mc = MotionCommander(Robots.KINOVA_GEN3_7DOF_F85, node, path_planner=PathPlanners.CUMOTION)
        self.mc.set_cartesian_scaling(0.4)
        
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        Thread(target=executor.spin, daemon=True, args=()).start()

        # 1. Parameters
        self._world_frame = self.declare_parameter('world_frame', 'base_link').get_parameter_value().string_value
        self._home_frame = self.declare_parameter('home_frame', 'home_frame').get_parameter_value().string_value
        self._post_grasp_lift_frame = self.declare_parameter('post_grasp_lift_frame', 'post_grasp_lift_frame').get_parameter_value().string_value # NEW
        self._drop_off_frame = self.declare_parameter('drop_off_frame', 'drop_off_frame').get_parameter_value().string_value # NEW

        self._plan_timer_period = self.declare_parameter('plan_timer_period', 0.01).get_parameter_value().double_value
        planner_group_name = self.declare_parameter('planner_group_name', 'manipulator').get_parameter_value().string_value
        pipeline_id = self.declare_parameter('pipeline_id', 'isaac_ros_cumotion').get_parameter_value().string_value
        planner_id = self.declare_parameter('planner_id', 'cuMotion').get_parameter_value().string_value
        ee_link = self.declare_parameter('end_effector_link', 'tool_frame').get_parameter_value().string_value

        # Use planning_frame parameter if provided, otherwise default to world_frame
        planning_frame_param = self.declare_parameter('planning_frame', '').get_parameter_value().string_value
        self._planning_frame = planning_frame_param if planning_frame_param else self._world_frame

        # Gripper Parameters
        self._gripper_action_name = self.declare_parameter(
            'gripper_action_name', '/robotiq_gripper_controller/gripper_cmd' # Default, VERIFY THIS
        ).get_parameter_value().string_value
        self._gripper_open_pos = self.declare_parameter('gripper_open_position', 0.0).get_parameter_value().double_value # NEW
        self._gripper_close_pos = self.declare_parameter('gripper_close_position', 100.0).get_parameter_value().double_value # NEW
        self._gripper_max_effort = self.declare_parameter('gripper_max_effort', 100.0).get_parameter_value().double_value # NEW


        self.get_logger().info(f"World frame: {self._world_frame}")
        self.get_logger().info(f"Planning relative to frame: {self._planning_frame}")
        self.get_logger().info(f"End effector link: {ee_link}")
        self.get_logger().info(f"Target Frames: Home='{self._home_frame}', Lift='{self._post_grasp_lift_frame}', Drop='{self._drop_off_frame}'")
        self.get_logger().info(f"Using gripper action server: {self._gripper_action_name}")
        self.get_logger().info(f"Gripper positions: Open={self._gripper_open_pos}, Close={self._gripper_close_pos}, Effort={self._gripper_max_effort}")


        # 2. MoveIt client
        self.move_group_client = MoveGroupClient(self, planner_group_name, pipeline_id, planner_id, ee_link)

        # 3. TF buffer/listener
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # 4. State machine flags
        self._did_home = False
        self._grasp_triggered = False        # GraspNet service called
        self._grasp_tf_available = False   # estimated_grasp TF seen
        self._did_grasp_pose = False       # Reached estimated_grasp pose
        self._gripper_close_command_sent = False # Track if close command was sent
        self._gripper_closed = False           # Track if close command finished (success/fail)
        self._did_lift_pose = False            # NEW: Reached lift pose
        self._did_drop_off_pose = False        # NEW: Reached drop off pose
        self._gripper_open_command_sent = False  # NEW: Track if open command was sent
        self._gripper_opened = False           # NEW: Track if open command finished (success/fail)
        self._sequence_complete = False        # Final state


        # 5. Service Client (GraspNet Trigger)
        self.grasp_trigger_client = self.create_client(Trigger, 'trigger_grasp_calculation')
        service_timeout_sec = 15.0
        # ... (service wait logic remains the same) ...
        if not self.grasp_trigger_client.wait_for_service(timeout_sec=service_timeout_sec):
            self.get_logger().error(f'GraspNet service not available after {service_timeout_sec}s. Cannot proceed.')
            rclpy.shutdown(); sys.exit(1)
        else:
            self.get_logger().info('GraspNet service client connected.')

        # 6. Action Client (Gripper Control)
        self._gripper_client = ActionClient(self, GripperCommand, self._gripper_action_name)
        gripper_timeout_sec = 10.0
        # ... (action client wait logic remains the same) ...
        if not self._gripper_client.wait_for_server(timeout_sec=gripper_timeout_sec):
             self.get_logger().error(f"Gripper action server '{self._gripper_action_name}' not available. Cannot control gripper.")
             self._gripper_client = None # Disable gripper control
        else:
            self.get_logger().info('Gripper action client connected.')

        # 7. Timer
        self._timer = self.create_timer(self._plan_timer_period, self._tick)
        self.get_logger().info('Home node initialized. Starting state machine.')

    def _tick(self):
        """Called periodically by the timer to progress our state machine."""
        if self._sequence_complete:
            return # Do nothing if sequence is done

        if not self._did_home:
            self.get_logger().debug("State: Trying to reach home")
            self._try_go_frame(frame=self._home_frame, label='home')
        elif not self._grasp_triggered:
             self.get_logger().debug("State: Triggering GraspNet")
             self._trigger_grasp_service() # Triggered once on home success, handled by callback
        elif not self._grasp_tf_available:
             self.get_logger().debug("State: Waiting for grasp TF")
             self._check_grasp_tf() # Keep checking if TF is published
        elif not self._did_grasp_pose:
            self.get_logger().debug("State: Trying to reach grasp pose")
            self._try_lookup_and_go_grasp()
        elif not self._gripper_close_command_sent:
             self.get_logger().debug("State: Sending gripper close command")
             self._send_gripper_command(command_type='close')
        elif not self._gripper_closed:
             # Waiting for gripper close action result (handled by callbacks)
             self.get_logger().debug("State: Waiting for gripper close result...")
             pass
        elif not self._did_lift_pose: # NEW STATE
             self.get_logger().debug("State: Trying to reach post-grasp lift pose")
             self._try_go_frame(frame=self._post_grasp_lift_frame, label='post_grasp_lift')
        elif not self._did_drop_off_pose: # NEW STATE
             self.get_logger().debug("State: Trying to reach drop-off pose")
             self._try_go_frame(frame=self._drop_off_frame, label='drop_off')
        elif not self._gripper_open_command_sent: # NEW STATE
             self.get_logger().debug("State: Sending gripper open command")
             self._send_gripper_command(command_type='open')
        elif not self._gripper_opened: # NEresponseW STATE
             # Waiting for gripper open action result (handled by callbacks)
             self.get_logger().debug("State: Waiting for gripper open result...")
             pass
        else:
             # All done
             self.get_logger().info("Sequence complete (Home -> Grasp -> Close -> Lift -> Drop -> Open). Shutting down timer.")
             self._sequence_complete = True
             self.destroy_timer(self._timer)
             # Optionally shutdown node? rclpy.shutdown()

    def _try_go_frame(self, frame: str, label: str):
        """Attempts to look up a TF frame and move the robot's EE to it."""
        try:
            # Ensure we are looking up relative to the planning frame
            tf_msg = self._tf_buffer.lookup_transform(
                self._planning_frame, # Target frame for the transform lookup
                frame,                # Source frame (the goal frame name)
                rclpy.time.Time(seconds=0),
                timeout=rclpy.duration.Duration(seconds=0.5) # Reduced timeout for faster checks
            )
            goal_pose_in_planning_frame = self._transform_msg_to_pose_msg(tf_msg.transform)

        except TransformException as ex:
            self.get_logger().warn(f'Could not transform "{frame}" to "{self._planning_frame}". TF error: {ex}. Retrying next tick.')
            return

        # If TF lookup succeeded, create and send the goal
        goal_pose_stamped = PoseStamped()
        goal_pose_stamped.header.stamp = self.get_clock().now().to_msg()
        goal_pose_stamped.header.frame_id = self._planning_frame # Goal is relative to the planning frame
        goal_pose_stamped.pose = goal_pose_in_planning_frame

        self.get_logger().debug(f"Sending goal to reach {label} frame: {frame} relative to {self._planning_frame}")
        response = self.move_group_client.send_goal_pose(goal_pose_stamped)

        if response.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info(f'MoveIt SUCCESS: Reached {label} pose ({frame}) successfully!')
            # --- Update state based on which frame was reached ---
            if label == 'home' and not self._did_home:
                self._did_home = True
                # Trigger GraspNet service automatically after reaching home
                self.get_logger().info('Home pose reached. Triggering GraspNet service...')
                self._trigger_grasp_service()
            elif label == 'post_grasp_lift' and not self._did_lift_pose:
                self._did_lift_pose = True
            elif label == 'drop_off' and not self._did_drop_off_pose:
                self._did_drop_off_pose = True
            # Grasp pose is handled by _try_lookup_and_go_grasp
        else:
             # Log MoveIt errors (abbreviated for brevity)
             error_string = self.move_group_client.get_error_string(response.error_code.val)
             self.get_logger().error(f'MoveIt FAILED to reach {label} pose ({frame}). Error: {error_string} (code: {response.error_code.val}). Will retry.')
             # Do not advance state on failure

    def _check_grasp_tf(self):
        """Checks if the estimated_grasp TF is available."""
        if self._grasp_tf_available: return # Already seen
        try:
            # Check if the transform exists without timeout error
            self._tf_buffer.lookup_transform(
                 self._planning_frame, # Target frame
                 'estimated_grasp',    # Source frame published by GraspNet
                 rclpy.time.Time(seconds=0),
                 timeout=rclpy.duration.Duration(seconds=0.05) # Short timeout just to check existence
            )
            self.get_logger().info(f"Transform '{self._planning_frame}' -> 'estimated_grasp' is now available.")
            self._grasp_tf_available = True
        except TransformException:
             self.get_logger().debug(f"Waiting for transform '{self._planning_frame}' -> 'estimated_grasp'...")
             self._grasp_tf_available = False # Ensure it's false if lookup fails


    def _try_lookup_and_go_grasp(self):
        """Specific function to move to the dynamically published grasp pose using MotionCommander."""
        # Assumes _check_grasp_tf() has already confirmed TF availability
        target_frame = self._planning_frame
        source_frame = 'estimated_grasp'
        try:
            # Lookup the latest transform
            tf_grasp_in_target = self._tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time(seconds=0),
                timeout=rclpy.duration.Duration(seconds=0.5) # Short timeout okay now
            )
            self.get_logger().debug(f"Successfully looked up transform from {target_frame} to {source_frame} for grasp goal.")
            # Extract the Pose message from the Transform message
            goal_pose_msg = self._transform_msg_to_pose_msg(tf_grasp_in_target.transform)

        except TransformException as ex:
            self.get_logger().warning(f'Could not transform "{source_frame}" to "{target_frame}" even after check. TF error: {ex}. Retrying.')
            return # Should ideally not happen if _check_grasp_tf worked, but handle anyway

        # --- NEW: Convert ROS Pose message to spatialmath SE3 object ---
        try:
            # Use the conversion utility (ensure pose_msg_to_se3 exists and works)
            goal_pose_se3 = pose_msg_to_se3(goal_pose_msg)
            self.get_logger().debug(f"Converted grasp Pose message to SE3 object: {goal_pose_se3}")
        except Exception as e:
            self.get_logger().error(f"Failed to convert Pose message to SE3: {e}")
            traceback.print_exc() # Print detailed error for debugging
            return # Cannot proceed without conversion

        # --- Call MotionCommander with the SE3 object ---
        self.get_logger().info(f"Sending CARTESIAN goal to reach estimated grasp pose relative to {target_frame} using MotionCommander...")
        self.get_logger().info(f"Move to pose (SE3):\n{goal_pose_se3}") # Log the SE3 object being sent

        # --- IMPORTANT: Check how mc.move_to_pose reports success/failure ---
        # The MotionCommander code wasn't fully shown, so we need to assume how it indicates success.
        # Common ways:
        # 1. Returns True/False
        # 2. Raises an exception on failure
        # 3. Returns a status object/enum
        # Adjust the following based on your MotionCommander implementation.
        # Let's *assume* it returns True on success, False on failure for this example.

        try:
            success = self.mc.move_to_pose(goal_pose_se3, cartesian=True) # Pass the SE3 object
        except Exception as e:
            # Handle potential exceptions during the move command itself
            self.get_logger().error(f"MotionCommander move_to_pose failed with exception: {e}")
            traceback.print_exc()
            success = False # Treat exception as failure

        # --- Update state based on the result ---
        if success:
            self.get_logger().info('MotionCommander SUCCESS: Reached estimated grasp pose successfully (Cartesian)!')
            if not self._did_grasp_pose:
                 self._did_grasp_pose = True # Mark grasp pose as reached
                 # Proceed to close gripper in the next tick
        else:
            # MotionCommander failed to reach grasp pose
            self.get_logger().error('MotionCommander FAILED to reach estimated grasp pose (Cartesian). Will retry on next tick.')
            # DO NOT set self._did_grasp_pose = True here

    # --- Service Call and Response (GraspNet Trigger) ---
    def _trigger_grasp_service(self):
        if self._grasp_triggered:
             self.get_logger().warn("Grasp service already triggered.")
             return
        if self.grasp_trigger_client is None or not self.grasp_trigger_client.service_is_ready():
             self.get_logger().error('GraspNet service client not ready/available. Cannot trigger.')
             self._grasp_triggered = True # Mark as attempted (failed)
             # Decide how to handle failure - maybe stop sequence? For now, just log.
             return
        request = Trigger.Request()
        future = self.grasp_trigger_client.call_async(request)
        future.add_done_callback(self._handle_grasp_trigger_response)
        self.get_logger().info('GraspNet service trigger request sent.')
        self._grasp_triggered = True # Mark as sent

    def _handle_grasp_trigger_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'GraspNet service call successful: {response.message}. Now waiting for TF...')
                # State machine will proceed to check for TF in _tick
            else:
                self.get_logger().error(f'GraspNet service call failed: {response.message}. Cannot move to grasp.')
                # How to handle failure? Stop the whole sequence?
                self._sequence_complete = True # Stop the sequence
                self.destroy_timer(self._timer)
        except Exception as e:
            self.get_logger().error(f'Exception during GraspNet service call response: {e}')
            self._sequence_complete = True # Stop the sequence
            self.destroy_timer(self._timer)


    # --- Methods for Gripper Action Client ---
    def _send_gripper_command(self, command_type: str):
        """Sends a goal ('open' or 'close') to the GripperCommand action server."""
        if command_type == 'close':
            if self._gripper_close_command_sent:
                 self.get_logger().warn("Gripper close command already sent, skipping.")
                 return
            position = self._gripper_close_pos
            self._gripper_close_command_sent = True # Mark close command as sent
        elif command_type == 'open':
             if self._gripper_open_command_sent:
                 self.get_logger().warn("Gripper open command already sent, skipping.")
                 return
             position = self._gripper_open_pos
             self._gripper_open_command_sent = True # Mark open command as sent
        else:
            self.get_logger().error(f"Invalid gripper command type: {command_type}")
            return

        if self._gripper_client is None or not self._gripper_client.server_is_ready():
            self.get_logger().error(f"Gripper action server not available/ready. Cannot send {command_type} command.")
            # Mark corresponding state as finished (failed)
            if command_type == 'close': self._gripper_closed = True
            else: self._gripper_opened = True
            return

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = self._gripper_max_effort

        self.get_logger().info(f"Sending gripper {command_type} goal: position={position}, max_effort={self._gripper_max_effort}")

        send_goal_future = self._gripper_client.send_goal_async(
            goal_msg,
            feedback_callback=self._gripper_feedback_callback) # Optional feedback

        # Add context to the callback using lambda
        send_goal_future.add_done_callback(
            lambda future: self._gripper_goal_response_callback(future, command_type)
        )

    def _gripper_goal_response_callback(self, future, command_type: str):
        """Handle server's acceptance/rejection of the gripper goal."""
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'Exception while getting gripper goal handle for {command_type}: {e}')
            if command_type == 'close': self._gripper_closed = True
            else: self._gripper_opened = True
            return

        if not goal_handle.accepted:
            self.get_logger().error(f'Gripper {command_type} goal rejected by server.')
            if command_type == 'close': self._gripper_closed = True
            else: self._gripper_opened = True
            return

        self.get_logger().info(f'Gripper {command_type} goal accepted by server.')
        get_result_future = goal_handle.get_result_async()
        # Add context to the result callback
        get_result_future.add_done_callback(
             lambda future: self._gripper_get_result_callback(future, command_type)
        )

    def _gripper_feedback_callback(self, feedback_msg):
        """Handle feedback during gripper action execution."""
        # feedback = feedback_msg.feedback # feedback is GripperCommand.Feedback
        # self.get_logger().debug(f'Gripper feedback: Position={feedback.position:.3f}, Effort={feedback.effort:.1f}, Stalled={feedback.stalled}, Reached Goal={feedback.reached_goal}')
        pass # Usually not needed unless monitoring progress closely

    def _gripper_get_result_callback(self, future, command_type: str):
        """Handle the final result of the gripper action."""
        try:
            result_wrapper = future.result()
            status = result_wrapper.status
            result = result_wrapper.result # This is GripperCommand.Result
        except Exception as e:
             self.get_logger().error(f'Exception while getting gripper result for {command_type}: {e}')
             if command_type == 'close': self._gripper_closed = True
             else: self._gripper_opened = True
             return

        success = False
        if status == GoalStatus.STATUS_SUCCEEDED:
            # Check result fields if needed (e.g., result.reached_goal for some grippers)
            self.get_logger().info(f'Gripper {command_type} action SUCCEEDED. Reached Goal: {getattr(result, "reached_goal", "N/A")}, Position: {result.position:.3f}')
            success = True
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error(f'Gripper {command_type} action ABORTED.')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(f'Gripper {command_type} action CANCELED.')
        else:
             self.get_logger().error(f'Gripper {command_type} action finished with unknown status: {status}')

        # Update the correct state flag
        if command_type == 'close':
            self._gripper_closed = True # Mark close action as complete (success or fail)
            if not success:
                self.get_logger().error("Gripper close failed. Stopping sequence.")
                self._sequence_complete = True # Stop if close fails
                self.destroy_timer(self._timer)
        else: # command_type == 'open'
            self._gripper_opened = True # Mark open action as complete (success or fail)
            if not success:
                 self.get_logger().error("Gripper open failed, but sequence is otherwise complete.")
                 # Sequence ends here anyway

    # --- Utility Function ---
    def _transform_msg_to_pose_msg(self, transform_msg: Transform) -> Pose:
        pose = Pose()
        pose.position.x = transform_msg.translation.x
        pose.position.y = transform_msg.translation.y
        pose.position.z = transform_msg.translation.z
        pose.orientation.x = transform_msg.rotation.x
        pose.orientation.y = transform_msg.rotation.y
        pose.orientation.z = transform_msg.rotation.z
        pose.orientation.w = transform_msg.rotation.w
        return pose

# --- Main Execution ---
def main(args=None):
    rclpy.init(args=args)
    node = HomeNode()
    executor = MultiThreadedExecutor() # Essential for handling callbacks concurrently
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, shutting down.\n')
    except Exception as e:
        node.get_logger().fatal(f"Unhandled exception in HomeNode: {e}")
        traceback.print_exc()
    finally:
        node.get_logger().info('Shutting down HomeNode...')
        # Cleanly shutdown action/service clients? Usually handled by node destruction
        node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        node.get_logger().info('Shutdown complete.')

if __name__ == '__main__':
    main()