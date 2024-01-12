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
            queue_size=10, # TODO: Change to 20?
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
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

        # set plan start state using predefined state
        self.move_group.set_start_state("sleep")

        # set pose goal using predefined state
        self.move_group.set_goal_state("home")

        # plan to goal
        # plan_and_execute(panda, panda_arm, logger)

            
    def go_to_joint_state(self, joint_goal): # TODO: implement one joint at a time?
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        # move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        ## thing we want to do is move it to a slightly better configuration.
        ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        # We get the joint values from the group and change some of the values:
        # joint_goal = move_group.get_current_joint_values()
        # joint_goal[0] = 0
        # joint_goal[1] = -tau / 8
        # joint_goal[2] = 0
        # joint_goal[3] = -tau / 4
        # joint_goal[4] = 0
        # joint_goal[5] = tau / 6  # 1/6 of a turn
        # joint_goal[6] = 0

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        self.move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        current_joints = self.move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    
    def go_to_pose_goal(self, pg : list):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w, pose_goal.position.x, pose_goal.position.y, pose_goal.position.z = pg

        self.move_group.set_pose_target(pose_goal)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)
    
    def go_to_home(self):
        self.move_group.go()
    
    def plan_cartesian_path(self, wp, scale=0.1):

        waypoints = []

        wpose = self.move_group.get_current_pose().pose

        for point in wp:
            wpose.position.x += scale * point[0]
            wpose.position.y += scale * point[1]
            wpose.position.z += scale * point[2]

            waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0 # TODO: edit threshold values
        )

        return plan, fraction
    
    def execute_plane(self, plan):
        self.move_group.execute(plan, wait=True)

    def display_trajectory(self, plan):
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)

        self.display_trajectory_publisher.publish(display_trajectory)



def main():

    eve = Mover()
    # eve.move_group.go

    import ipdb; ipdb.set_trace()


if __name__ == "__main__":
    main()