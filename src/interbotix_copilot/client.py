import typing as t
from concurrent.futures import Future
from threading import Event
import time
try:
    import rospy
    from std_msgs.msg import Bool
    from interbotix_copilot.base_api import InterbotixArm, MODES, POSITION, VELOCITY
    from interbotix_copilot.base_api import STOPPED, CANCELLED, EXECUTED
    from interbotix_copilot.srv import Command, CommandResponse, CommandRequest
    from interbotix_xs_msgs.srv import *
    from std_srvs.srv import SetBool, SetBoolRequest
    # Task Action
    from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
    from std_msgs.msg import Header
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    import actionlib
except ImportError as e:
    print(f"ROS (incl. interbotix driver, interbotix_copilot) must be installed & sourced: {e}")
    raise e


class Client(InterbotixArm):
    def __init__(self, robot_type: str, name: str, group_name: str = "arm"):
        """Client for an interbotix robot arm.

        All writable commands flow through the copilot.

        .. note:: All motors in the group must have the same Operating Mode, Profile, Acceleration Profile,
                  and Velocity Profile.

        :param robot_type: Valid Interbotix robot type (e.g. "px150", "vx300s")
        :param name: Name given to the robot. Should match the name given during the driver launch.
        :param group_name: Name of the group of joint motors.
        """
        try:
            rospy.init_node(f"{name}_client")
        except rospy.exceptions.ROSException:
            pass

        # Call subclass constructor before initializing the copilot client.
        super().__init__(robot_type=robot_type, name=name, group_name=group_name)

        self._ft_event = Event()
        self._feedthrough = None

        # Wait for copilot services & task server
        try:
            rospy.wait_for_service(f"/{name}/copilot/set_motor_registers", timeout=5.0)
            rospy.wait_for_service(f"/{name}/get_motor_registers", timeout=5.0)
            rospy.wait_for_service(f"/{name}/get_robot_info", timeout=5.0)
            rospy.wait_for_service(f"/{name}/copilot/torque_enable", timeout=5.0)
            rospy.wait_for_service(f"/{name}/copilot/reboot_motors", timeout=5.0)
            rospy.wait_for_service(f"/{name}/copilot/write_commands", timeout=5.0)
            rospy.wait_for_service(f"/{name}/copilot/stop", timeout=5.0)
            self.sub_ft = rospy.Subscriber(f"/{name}/copilot/feedthrough", Bool, callback=self._toggle_feedthrough)
            self.srv_set_reg = rospy.ServiceProxy(f"/{name}/copilot/set_motor_registers", RegisterValues)
            self.srv_torque = rospy.ServiceProxy(f"/{name}/copilot/torque_enable", TorqueEnable)
            self.srv_reboot = rospy.ServiceProxy(f"/{name}/copilot/reboot_motors", Reboot)
            self.srv_write_commands = rospy.ServiceProxy(f"/{name}/copilot/write_commands", Command)
            self.srv_stop = rospy.ServiceProxy(f"/{name}/copilot/stop", SetBool)
            # Initialize task client
            address = f"/{name}/copilot/task"
            self.task_client = actionlib.SimpleActionClient(address, FollowJointTrajectoryAction)
            rospy.logdebug(f"Waiting for server `{address}` ...")
            self.task_client.wait_for_server(timeout=rospy.Duration(secs=5))
            rospy.logdebug(f"[{address}] Connected to server")

        except rospy.exceptions.ROSException as e:
            rospy.logerr(str(e.args[0]))
            rospy.logerr(
                f"The copilot for robot '{robot_type}' is not discoverable. "
                f"Is the entered robot_name `{name}` correct? "
                "Quitting...")
            raise e

    def wait_for_feedthrough(self, timeout: t.Optional[float] = None) -> Future:
        """Block until the writable commands will be feedthrough to the robot.

        If the internal flag is true on entry, return immediately. Otherwise,
        block until the feedthrough callback calls set() to set the flag to true, or until
        the optional timeout occurs.

        When the timeout argument is present and not None, it should be a
        floating point number specifying a timeout for the operation in seconds
        (or fractions thereof).

        This method returns the internal flag on exit, so it will always return
        True except if a timeout is given and the operation times out.

        :param timeout: timeout period in seconds.
        :return: Future that will be True if returned due to time out.
        """

        def _task_wait_for_feedthrough(arm: Client, _timeout: t.Optional[float] = None):
            if self.task_event.is_set():
                return CANCELLED
            rate = rospy.Rate(100)
            start = time.time()
            while not self._ft_event.is_set() or (_timeout is not None and time.time() - start < _timeout):
                # Check if we should cancel the task due to emergency stop
                if arm.task_event.is_set():
                    return STOPPED
                rate.sleep()
            return EXECUTED

        # Submit task
        f = self._submit_task(False, "wait for feedthrough", _task_wait_for_feedthrough, self, timeout)
        return f

    def _toggle_feedthrough(self, msg: Bool):
        # Return if feedthrough is already toggled to new status.
        # if self._feedthrough == msg.data:
        #     rospy.logwarn(f"[{get_ident()}] Feedthrough already toggled to same status: "
        #                   f"self._feedthrough={self._feedthrough} | msg.data={msg.data}")
        #     return
        # Toggle feedthrough
        self._feedthrough = msg.data
        # Toggle feedthrough routines
        if self._feedthrough:
            # Release lock to allow feedthrough of actions
            # rospy.loginfo(f"[{get_ident()}] Releasing lock: allowing feedthrough!")
            self._ft_event.set()
        else:
            # Acquire lock to block feedthrough actions
            # rospy.loginfo(f"[{get_ident()}] Acquiring lock: blocking feedthrough!")
            self._ft_event.clear()

    def _set_operating_mode(self, mode, profile_type, profile_velocity, profile_acceleration):
        """Is called in set_operating_mode().

        :param mode: https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#operating-mode
        :param profile_type: "time" or "velocity".
        :param profile_acceleration: Sets acceleration time of the Profile (see above). ‘0’ represents an infinite acceleration.
        :param profile_velocity: Sets velocity of the Profile (see above) . ‘0’ represents an infinite velocity.
        :return:
        """
        pass

    def set_motor_register(self, register: str, value: int, name: t.Optional[str] = None) -> Future:
        """Set the register value for either a single motor or a group of motors.

        **IMPORTANT** Register values are **always** passed through. Do not use this method to set
        register values that could significantly influence the control behavior when you are not sure
        whether the copilot is set to "feedthrough".

        See available registers:
        https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#control-table-of-eeprom-area
        https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#control-table-of-ram-area

        :param register: Desired register name. Naming convention is "Data Name" with spaces replaced with "_".
        :param value: Desired value for the above register.
        :param name: If specified, the method only applies for the specified motor name.
        :return: Future of the task. Allows for blocking behavior by calling f.result(timeout [s]).
                 This task will **still** executed when self.cancel_all_tasks() is called.
        """
        def _task_set_motor_register(arm: Client, register: str, value: int, name: t.Optional[str]):
            if name is None:
                self.srv_set_reg("group", self.GROUP_NAME, register, value)
            else:
                assert name in arm.INFO.joint_names, f"`{name}` is not a registered motor name in group `{arm.GROUP_NAME}`."
                self.srv_set_reg("single", name, register, value)
            return EXECUTED

        # Submit task
        f = self._submit_task(False, f"set motor register | register={register}| value={value}", _task_set_motor_register, self, register, value, name)
        return f

    def reboot(self, enable: bool = True, smart_reboot: bool = True, name: t.Optional[str] = None) -> Future:
        """Reboot a single motor or a group of motors.

        :param enable: True to torque on or False to leave torqued off after rebooting
        :param smart_reboot: Setting this to True will only reboot those motors that are in an error state,
                             as opposed to all motors within the group regardless of if they are in an error state.
        :param name: If specified, the method only applies for the specified motor name.
        :return: Future of the task. Allows for blocking behavior by calling f.result(timeout [s]).
                 This task will **still** executed when self.cancel_all_tasks() is called.
        """
        def _task_reboot(arm: Client, enable: bool, smart_reboot: bool, name: t.Optional[str]):
            if arm.task_event.is_set():
                return CANCELLED
            if name is None:
                self.srv_reboot("group", self.GROUP_NAME, enable, smart_reboot)
            else:
                self.srv_reboot("single", name, enable, smart_reboot)
            return EXECUTED

        # Submit task
        f = self._submit_task(False, "reboot", _task_reboot, self, enable, smart_reboot, name)
        return f

    def go_to_home(self) -> Future:
        """Go to home position.

        :return: Future of the task. Allows for blocking behavior by calling future.result(timeout [s]).
        """
        # Schedule a task to go to the home position in 4 seconds
        f = self.go_to(points=[self.INFO.num_joints * [0]], timestamps=[4.0], remap=False)
        return f

    def go_to_sleep(self) -> Future:
        """Go to sleep position.

        :return: Future of the task. Allows for blocking behavior by calling future.result(timeout [s]).
        """
        # Schedule a task to go to the sleep position in 4 seconds
        f = self.go_to(points=[list(self.INFO.joint_sleep_positions)], timestamps=[4.0], remap=False)
        return f

    def stop(self):
        """Stop the robot in its current position. Cancels all scheduled tasks."""
        def _task_stop(arm: InterbotixArm):
            if arm.task_event.is_set():
                return CANCELLED
            self.srv_stop(SetBoolRequest())
            return EXECUTED

        # Grab condition here (RLock is reentrant, so cancel_all_tasks(), _submit_task() can retake lock multiple times).
        # In this way, we ensure that all tasks are cancelled, directly followed by the stopping task in sequence.
        with self.task_cond:
            # We should cancel first, else stop command can be immediately overwritten
            self.cancel_all_tasks()
            # Submit stopping task to executor
            f = self._submit_task(False, "emergency stop", _task_stop, self)
            f.result()

    def write_commands(self, commands: t.List[float], remap: bool = True) -> Future:
        """Command a group of motors.

        :param commands: Desired list of commands. {velocity: [rad/s], position: [rad]}
        :param remap: True will remap the order according to set_joint_remapping().
        :return: Future of the task. Allows for blocking behavior by calling f.result(timeout [s]).
                 This task will **still** executed when self.cancel_all_tasks() is called.
                 True if the commands were successfully set, else False (e.g. if not in 'feedthrough' mode).
        """
        # Remap command to joint order.
        if remap and self._to is not None:
            indices = self._to
        else:
            indices = self.INFO.joint_state_indices
        cmd = [commands[i] for i in indices]

        if self._last_op_mode is None:
            raise ValueError("An operating mode should first be selected.")

        # Define command task
        def _task_write_commands(arm: Client, mode: int, profile_type: str, profile_acceleration: int, profile_velocity: int, _cmd: t.List[int]):
            if arm.task_event.is_set():
                return CANCELLED
            self.srv_write_commands(mode, profile_type, profile_acceleration, profile_velocity, self.INFO.joint_names, _cmd)
            # self.srv_write_commands(mode, profile_type, profile_acceleration, profile_velocity, _cmd)
            return EXECUTED

        # Submit task
        f = self._submit_task(False, f"write command | cmd={cmd}", _task_write_commands, self, *self._last_op_mode, cmd)
        return f

    def enable(self, name: t.Optional[str] = None) -> Future:
        """Torque a single motor or a group of motors to be on.

        :param name: If specified, the method only applies for the specified motor name.
        :return: Future of the task. Allows for blocking behavior by calling f.result(timeout [s]).
                 This task will **still** executed when self.cancel_all_tasks() is called.
        """
        def _task_enable(arm: Client, name: t.Optional[str]):
            if arm.task_event.is_set():
                return CANCELLED
            if name is None:
                self.srv_torque("group", self.GROUP_NAME, True)
            else:
                self.srv_torque("single", name, True)
            return EXECUTED

        # Submit task
        f = self._submit_task(False, "enable", _task_enable, self, name)
        return f

    def disable(self, name: t.Optional[str] = None):
        """Torque a single motor or a group of motors to be off.

        :param name: If specified, the method only applies for the specified motor name.
        :return: Future of the task. Allows for blocking behavior by calling f.result(timeout [s]).
                 This task will **still** executed when self.cancel_all_tasks() is called.
        """
        def _task_enable(arm: Client, name: t.Optional[str]):
            if arm.task_event.is_set():
                return CANCELLED
            if name is None:
                self.srv_torque("group", self.GROUP_NAME, False)
            else:
                self.srv_torque("single", name, False)
            return EXECUTED

        # Submit task
        f = self._submit_task(False, "enable", _task_enable, self, name)
        return f

    def go_to(self, points: t.List[t.List[float]], timestamps: t.List[float], remap: bool = True) -> Future:
        # Get remapping
        if remap and self._to is not None:
            indices = self._to
        else:
            indices = self.INFO.joint_state_indices
        # Remap positions
        points_remapped = []
        for p in points:
            points_remapped.append([p[i] for i in indices])

        def _go_to(arm: Client, _points, _timestamps):
            # Exit immediately, if task should be cancelled.
            if arm.task_event.is_set():
                return CANCELLED
            # Prepare task
            task = FollowJointTrajectoryGoal()
            task.trajectory = JointTrajectory()
            task.trajectory.header = Header()
            task.trajectory.joint_names = self.INFO.joint_names
            for p, t in zip(_points, _timestamps):
                # Add joint trajectory point to trajectory goal
                jtp = JointTrajectoryPoint()
                jtp.positions = p
                secs, nsecs = int(t), int((t % 1) * 1e9)
                jtp.time_from_start = rospy.Duration(secs=secs, nsecs=nsecs)
                task.trajectory.points.append(jtp)
            self.task_client.send_goal(task)
            # Periodically check if done/cancelled.
            rate = rospy.Rate(10)
            while self.task_client.get_state() not in [actionlib.GoalStatus.SUCCEEDED]:
                # Check if we should cancel the task (e.g. emergency stop)
                if arm.task_event.is_set():
                    # Cancel goal
                    self.task_client.cancel_goal()
                    return STOPPED
                rate.sleep()
            return EXECUTED

        # submit task
        tp = [(_t, _p) for _p, _t in zip(points_remapped, timestamps)]
        f = self._submit_task(False, f"go to | pos(t)={tp}", _go_to, self, points_remapped, timestamps)
        return f
