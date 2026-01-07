# In utilities/spatial_math_conversions.py

import spatialmath as sm
from spatialmath import SE3
from geometry_msgs.msg import Pose, Point, Quaternion as QuaternionMsg

def pose_msg_to_se3(pose_msg: Pose) -> SE3:
    """
    Converts a geometry_msgs/Pose message to a spatialmath SE3 object.
    """
    position = [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z]
    # spatialmath Quaternion expects [w, x, y, z] or uses scalar_last=False
    # geometry_msgs Quaternion is [x, y, z, w]
    quat = sm.UnitQuaternion(
        [pose_msg.orientation.w, pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z],
        check=True # Good practice to check if it's a valid unit quaternion
    )
    return SE3.Rt(quat.R, position)

# You might also need the inverse for se3_to_pose used elsewhere
def se3_to_pose_msg(pose_se3: SE3) -> Pose:
    """
    Converts a spatialmath SE3 object to a geometry_msgs/Pose message.
    """
    pose_msg = Pose()
    t = pose_se3.t # Translation vector [x, y, z]
    q = sm.UnitQuaternion(pose_se3.R) # Convert rotation matrix to UnitQuaternion

    pose_msg.position.x = t[0]
    pose_msg.position.y = t[1]
    pose_msg.position.z = t[2]

    # geometry_msgs Quaternion is [x, y, z, w]
    # spatialmath UnitQuaternion default representation might vary, .vec gives [w, x, y, z]
    pose_msg.orientation.x = q.vec[1]
    pose_msg.orientation.y = q.vec[2]
    pose_msg.orientation.z = q.vec[3]
    pose_msg.orientation.w = q.vec[0]

    return pose_msg

# Note: The original se3_to_pose in your traceback might have been intended
# to return a geometry_msgs.msg.PoseStamped, not just Pose.
# You might need to adjust based on its exact signature and usage elsewhere.
# The function below matches the traceback's usage:
import spatialmath.base as smb # As seen in traceback
from geometry_msgs.msg import PoseStamped
import std_msgs.msg

def se3_to_pose_stamped(pose_se3: SE3, frame_id: str = "base_link", stamp=None) -> PoseStamped:
    """
    Converts a spatialmath SE3 object to a geometry_msgs/PoseStamped message.
    Matches the likely intent of the function causing the original error.
    """
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = frame_id
    # Handle timestamp (optional) - Be careful with clock source if used across nodes
    if stamp is not None:
        pose_stamped.header.stamp = stamp
    # else: use current time? Depends on application needs

    pose_stamped.pose = se3_to_pose_msg(pose_se3) # Reuse the Pose conversion

    return pose_stamped