import typing as typ
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import actionlib
import rospy

if typ.TYPE_CHECKING:
    from interbotix_copilot.copilot import Copilot


class CopilotClient:
    def __init__(self, address, joint_names):
        self.joint_names = joint_names
        self.client = actionlib.SimpleActionClient(address, FollowJointTrajectoryAction)
        rospy.logdebug(f"Waiting for server `{address}` ...")
        self.client.wait_for_server()
        rospy.logdebug(f"[{address}] Connected to server")

    def send_task(self, points: typ.List[float], t: typ.List[float]):
        task = FollowJointTrajectoryGoal()
        task.trajectory = JointTrajectory()
        task.trajectory.header = Header()
        task.trajectory.joint_names = self.joint_names
        for p, t in zip(points, t):
            # Add joint trajectory point to trajectory goal
            jtp = JointTrajectoryPoint()
            jtp.positions = p
            sec, nsec = int(t), int((t % 1) * 1e9)
            jtp.time_from_start = rospy.Duration(sec=sec, nsec=nsec)
            task.trajectory.points.append(jtp)
        self.client.send_goal(task)

    def cancel(self):
        self.client.cancel_all_goals()


class CopilotServer:
    def __init__(self, address: str, copilot: "Copilot"):
        self.copilot = copilot
        self.server = actionlib.SimpleActionServer(address, FollowJointTrajectoryAction, self.execute, False)
        self.server.start()

    def schedule_task(self, task: FollowJointTrajectoryGoal):
        ...

