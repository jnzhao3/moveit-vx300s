# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
# END IMPORTS

# MOVEIT UTILITIES
def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


# END MOVEIT UTILITIES

class Mover(object):

    def __init__(self):
        super(Mover, self).__init__()

        joint_state_topic = ['joint_states:=/vx300s/joint_states']

        # moveit_commander.roscpp_initialize(sys.argv)
        moveit_commander.roscpp_initialize(joint_state_topic)
        rospy.init_node("mover", anonymous=True)

        robot = moveit_commander.RobotCommander(robot_description="vx300s/robot_description")
        scene = moveit_commander.PlanningSceneInterface(ns="vx300s")

        group_name = "interbotix_arm"
        move_group = moveit_commander.MoveGroupCommander(robot_description="vx300s/robot_description", ns="vx300s", name=group_name, wait_for_servers=5.0)

        display_trajectory_publisher = rospy.Publisher(
            "mover/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20
        )

        # TODO: Consider other variables?

        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")

        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.mg = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        # TODO: how to work with multiple group names?

    def go_sleep(self):
        self.mg.set_named_target("Sleep")
        self.mg.go(wait=True)
        self.mg.stop()

    def go_home(self):
        self.mg.set_named_target("Home")
        self.mg.go(wait=True)
        self.mg.stop()
    
    def go_upright(self):
        self.mg.set_named_target("Upright")
        self.mg.go(wait=True)
        self.mg.stop()

            
    def go_to_joint_state(self, joint_goal): # TODO: implement one joint at a time?
        self.mg.go(joint_goal, wait=True)
        self.mg.stop()

        # For testing:
        current_joints = self.mg.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    
    def go_to_pose_goal(self, pg : list):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w, pose_goal.position.x, pose_goal.position.y, pose_goal.position.z = pg

        self.mg.set_pose_target(pose_goal)
        plan = self.mg.go(wait=True)
        self.mg.stop()
        self.mg.clear_pose_targets()

        current_pose = self.mg.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)
    
    def plan_cartesian_path(self, wp, scale=0.1): # TODO: fix this -- there is something wrong with this method

        waypoints = []

        wpose = self.mg.get_current_pose().pose
        wpose.orientation.w = 1 # TODO: check if correct?

        for point in wp:

            print(point)

            wpose.position.x += scale * point[0]
            wpose.position.y += scale * point[1]
            wpose.position.z += scale * point[2]

            print(wpose.position.x, wpose.position.y, wpose.position.z)

            waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.mg.compute_cartesian_path(
            waypoints, 0.01, 0.0 # TODO: edit threshold values
        )

        return plan, fraction
    
    def execute_plan(self, plan):
        self.mg.execute(plan, wait=True)
        self.mg.stop() # TODO: is this necessary?

    def display_trajectory(self, plan):
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)

        self.display_trajectory_publisher.publish(display_trajectory)

    def step(self, wp): # TODO: add scale variable?
        plan, fraction = self.plan_cartesian_path(wp)
        self.execute_plan(plan)
        # TODO: return somethign?

    def rotate_gripper(self, theta, scale=1):
        # ['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 
        # 'wrist_rotate', 'ee_arm', 'gripper_bar', 'ee_bar', 'ee_gripper']

        joint_values = self.mg.get_current_joint_values()
        joint_values[5] += theta # TODO: figure out how to stop shoulder angle from changing

        self.go_to_joint_state(joint_values)
        # joint_values = self.mg.get_current_joint_values()
        # plan = self.mg.go(wait=True)
        # self.mg.stop()
        # self.mg.clear_pose_targets()




def main():

    eve = Mover()

    import ipdb; ipdb.set_trace()


if __name__ == "__main__":
    main()