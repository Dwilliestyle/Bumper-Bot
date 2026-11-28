#!/usr/bin/env python3
"""
Bumperbot den patrol - drives a loop of waypoints on a saved map using Nav2.

Each waypoint is sent as a separate NavigateToPose goal, so the behavior tree
loaded in bt_navigator (e.g. bumperbot_nav.xml) handles planning and recovery
for every leg. The script only decides WHERE to go next and how long to wait.

Before running:
  1. Launch the real robot with the saved map + AMCL (not SLAM):
       ros2 launch bumperbot_bringup real_robot.launch.py
  2. Put Bumperbot on its HOME spot (the pose in HOME_POSE below).
  3. Source the workspace and run:
       python3 bumperbot_patrol.py
  Ctrl+C cancels the current goal and stops the robot cleanly.
"""
import math

import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


# ------------------------------------------------------------------ settings
# All poses are in the MAP frame: (x [m], y [m], yaw [degrees]).
# Get them from RViz: click "2D Goal Pose" on the spot, then read it from
#   ros2 topic echo /goal_pose --once
# (yaw = 2*atan2(z, w) of the orientation, in degrees).

HOME_POSE = (0.0, 0.0, 0.0)          # where Bumperbot starts every run

# (name, x, y, yaw_deg, wait_seconds_at_the_spot)
WAYPOINTS = [
    ("couch",    1.50,  0.00,   90.0, 3.0),
    ("doorway",  1.50,  1.20,  180.0, 3.0),
    ("desk",     0.00,  1.20,  -90.0, 3.0),
    ("home",     0.00,  0.00,    0.0, 0.0),
]

LAPS = 2                     # 0 = patrol forever (until Ctrl+C)
LEG_TIMEOUT_S = 120.0        # give up on one waypoint after this long
STOP_ON_FAILURE = False      # True = end the patrol if a waypoint fails

# Optional: use a specific behavior tree for these goals instead of the
# default_nav_to_pose_bt_xml from the params file. "" = use the default.
BEHAVIOR_TREE = ""
# ---------------------------------------------------------------------------


def make_pose(nav, x, y, yaw_deg):
    """Build a PoseStamped in the map frame from x, y and a yaw in degrees."""
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    yaw = math.radians(yaw_deg)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def go_to(nav, name, x, y, yaw_deg):
    """Drive to one waypoint. Returns True if it was reached."""
    nav.info(f"--> heading to '{name}' ({x:.2f}, {y:.2f}, {yaw_deg:.0f} deg)")
    nav.goToPose(make_pose(nav, x, y, yaw_deg), behavior_tree=BEHAVIOR_TREE)

    last_print = nav.get_clock().now()
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback is None:
            continue

        # Print progress every 2 seconds
        now = nav.get_clock().now()
        if now - last_print > Duration(seconds=2.0):
            nav.info(
                f"    {feedback.distance_remaining:.2f} m left, "
                f"recoveries so far: {feedback.number_of_recoveries}"
            )
            last_print = now

        # Give up on this leg if it takes too long
        if Duration.from_msg(feedback.navigation_time) > Duration(seconds=LEG_TIMEOUT_S):
            nav.warn(f"    '{name}' timed out after {LEG_TIMEOUT_S:.0f} s, cancelling")
            nav.cancelTask()

    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        nav.info(f"    reached '{name}'")
        return True
    if result == TaskResult.CANCELED:
        nav.warn(f"    '{name}' was cancelled")
    else:
        nav.error(f"    failed to reach '{name}'")
    return False


def wait_here(nav, seconds):
    """Pause at a waypoint while still letting ROS process messages."""
    end = nav.get_clock().now() + Duration(seconds=seconds)
    while nav.get_clock().now() < end:
        rclpy.spin_once(nav, timeout_sec=0.1)


def main():
    rclpy.init()
    nav = BasicNavigator()

    # Tell AMCL where the robot is starting, then wait for Nav2 to come up
    hx, hy, hyaw = HOME_POSE
    nav.setInitialPose(make_pose(nav, hx, hy, hyaw))
    nav.waitUntilNav2Active()

    lap = 0
    try:
        while LAPS == 0 or lap < LAPS:
            lap += 1
            nav.info(f"===== lap {lap}{'' if LAPS == 0 else f' of {LAPS}'} =====")

            for name, x, y, yaw_deg, wait_s in WAYPOINTS:
                reached = go_to(nav, name, x, y, yaw_deg)
                if not reached and STOP_ON_FAILURE:
                    nav.error("Stopping patrol because a waypoint failed")
                    return
                if reached and wait_s > 0.0:
                    nav.info(f"    waiting {wait_s:.0f} s at '{name}'")
                    wait_here(nav, wait_s)

        nav.info("Patrol finished")

    except KeyboardInterrupt:
        nav.warn("Ctrl+C - cancelling the current goal")
        nav.cancelTask()

    finally:
        nav.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
