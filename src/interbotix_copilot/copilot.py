import typing as t
from concurrent.futures import Future
from threading import Event
import time

try:
    import rospy
    import rosparam
    from std_msgs.msg import Bool
    from interbotix_copilot.base_api import InterbotixArm, MODES, POSITION, VELOCITY, INTERBOTIX_MODES
    from interbotix_copilot.base_api import STOPPED, CANCELLED, EXECUTED, DISCARDED
    from interbotix_copilot.srv import Command, CommandResponse, CommandRequest
    from interbotix_xs_msgs.srv import *
    from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
    # Task server
    # todo: add actionlib, control_msgs to CMakeLists.txt & package.xml.
    from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
    from std_msgs.msg import Header
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    import actionlib
except ImportError as e:
    print(f"Hint: ROS (incl. interbotix driver, interbotix_copilot) must be installed & sourced: {e}")
    raise e


class Copilot(InterbotixArm):
    def __init__(self, robot_type: str, name: str, group_name: str = "arm"):
        """Copilot for an interbotix robot arm.

        There should only be one copilot, and it should run as a separate process in a stable script.

        .. note:: All motors in the group must have the same Operating Mode, Profile, Acceleration Profile,
                  and Velocity Profile.

        :param robot_type: Valid Interbotix robot type (e.g. "px150", "vx300s")
        :param name: Name given to the robot. Should match the name given during the driver launch.
        :param group_name: Name of the group of joint motors.
        """
        # Initialize as ros node (if not already initialized).
        try:
            rospy.init_node(f"{name}_copilot")
        except rospy.exceptions.ROSException:
            pass

        # Call subclass constructor before initializing the copilot server.
        super().__init__(robot_type=robot_type, name=name, group_name=group_name)

        # Feedthrough toggle.
        self.pub_ft = rospy.Publisher(f"/{name}/copilot/feedthrough", Bool, queue_size=0,
                                      latch=True)

        # GUI Application
        self.app = None

        # Initialize stopping event
        self._stopping_event = Event()

        # Initialize moveit
        if not self.use_sim:
            self._init_moveit()

        # Initialize copilot services
        self.srv_set_reg = rospy.Service(f"/{name}/copilot/set_motor_registers", RegisterValues, self._handler_set_reg)
        self.srv_torque = rospy.Service(f"/{name}/copilot/torque_enable", TorqueEnable, self._handler_torque)
        self.srv_reboot = rospy.Service(f"/{name}/copilot/reboot_motors", Reboot, self._handler_reboot)
        self.srv_write_commands = rospy.Service(f"/{name}/copilot/write_commands", Command, self._handler_write_commands)
        self.srv_stop = rospy.Service(f"/{name}/copilot/stop", SetBool, self._handler_stop)

        # Initialize task server
        self.task_server = actionlib.SimpleActionServer(f"/{name}/copilot/task", FollowJointTrajectoryAction, self._handler_go_to, False)
        self.task_server.start()

    def gui(self):
        try:
            from PyQt5.QtCore import QSize, Qt
            from PyQt5.QtWidgets import QApplication
            from interbotix_copilot.gui import CopilotGui
        except ImportError as e:
            print(f"Hint: PyQt5 must be installed: {e}")
            return

        self.app = QApplication(sys.argv)
        self.window = CopilotGui(self)
        self.window.show()
        self.app.exec()  # <---------- code blocks over here !

    def shutdown(self):
        if self.app:
            print("Shutdown app.")
            self.window.close()
            self.window.shutdown()
            self.app.quit()

    # todo: initialize moveit.
    def _init_moveit(self):
        import moveit_commander
        from geometry_msgs.msg import PoseStamped
        import yaml

        # MoveIt parameters
        self.current_position = None
        self.base_frame = rospy.get_param("~robot_base_frame")
        self.robot_effector_frame = rospy.get_param("~robot_effector_frame")
        self.joint_names = yaml.safe_load(rospy.get_param("~joint_names"))
        self.manipulator_group_name = rospy.get_param("~manipulator_group_name", "interbotix_arm")
        self.end_effector_group_name = rospy.get_param("~end_effector_group_name", "interbotix_gripper")

        self.ee_step = rospy.get_param("~ee_step", 0.01)
        self.jump_threshold = rospy.get_param("~jump_threshold", 0.0)
        self.collision_height = rospy.get_param("~collision_height", 0.1)
        self.base_length = rospy.get_param("~base_length", 0.4)
        self.workspace_length = rospy.get_param("~workspace_length", 2.4)
        self.velocity_scaling_factor = rospy.get_param("~velocity_scaling_factor", 0.3)
        self.acceleration_scaling_factor = rospy.get_param("~acceleration_scaling_factor", 0.3)

        # Initialize Moveit Commander and Scene
        moveit_commander.roscpp_initialize(sys.argv)
        scene = moveit_commander.PlanningSceneInterface(synchronous=True)

        self.manipulator_group = moveit_commander.MoveGroupCommander(self.manipulator_group_name)
        self.end_effector_group = moveit_commander.MoveGroupCommander(self.end_effector_group_name)
        self.manipulator_group.set_max_velocity_scaling_factor(self.velocity_scaling_factor)
        self.manipulator_group.set_max_acceleration_scaling_factor(self.acceleration_scaling_factor)

        # Add a collision object to the scenes
        object_length = (self.workspace_length - self.base_length) / 2.0
        p0 = PoseStamped()
        p0.header.frame_id = self.base_frame
        p0.pose.position.z = -0.051
        p0.pose.orientation.w = 1

        # Add a collision object to the scenes
        p1 = PoseStamped()
        p1.header.frame_id = self.base_frame
        p1.pose.position.x = (object_length + self.base_length) / 2.0
        p1.pose.position.z = self.collision_height / 2.0
        p1.pose.orientation.w = 1

        # Add a collision object to the scenes
        p2 = PoseStamped()
        p2.header.frame_id = self.base_frame
        p2.pose.position.x = -(object_length + self.base_length) / 2.0
        p2.pose.position.z = self.collision_height / 2.0
        p2.pose.orientation.w = 1

        # Add a collision object to the scenes
        p3 = PoseStamped()
        p3.header.frame_id = self.base_frame
        p3.pose.position.y = (object_length + self.base_length) / 2.0
        p3.pose.position.z = self.collision_height / 2.0
        p3.pose.orientation.w = 1

        # Add a collision object to the scenes
        p4 = PoseStamped()
        p4.header.frame_id = self.base_frame
        p4.pose.position.y = -(object_length + self.base_length) / 2.0
        p4.pose.position.z = self.collision_height / 2.0
        p4.pose.orientation.w = 1

        scene.add_box("base0", p0, size=(self.workspace_length, self.workspace_length, 0.1))
        scene.add_box("base1", p1, size=(object_length, self.base_length, self.collision_height))
        scene.add_box("base2", p2, size=(object_length, self.base_length, self.collision_height))
        scene.add_box("base3", p3, size=(self.workspace_length, object_length, self.collision_height))

        # todo: initialize moveit commander
        # todo: Files to launch:
        #  $(find interbotix_xsarm_ros_control)/launch/xsarm_ros_control.launch
        #  $(find interbotix_xsarm_moveit)/launch/move_group.launch
        #  $(findinterbotix_xsarm_control)/launch/xsarm_control.launch
        #  $(find interbotix_copilot)/launch/copilot.launch
        # https://github.com/jelledouwe/interbotix_calibration/blob/main/interbotix_calibration/launch/interbotix_calibration.launch
        # https://github.com/jelledouwe/interbotix_calibration/blob/main/interbotix_calibration/src/interbotix_calibration/interbotix_calibration_node.py#L144
        pass

    def enable_feedthrough(self):
        # Wait for last task to be executed
        if self._last_task:
            self._last_task.result()
        # Set feedthrough to True
        self._feedthrough = True
        self.pub_ft.publish(Bool(True))

    def disable_feedthrough(self):
        self._feedthrough = False
        self.pub_ft.publish(Bool(False))

    def _set_operating_mode(self, mode: int, profile_type: str, profile_velocity: int, profile_acceleration: int):
        """Is called in set_operating_mode().

        :param mode: https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#operating-mode
        :param profile_type: "time" or "velocity".
        :param profile_acceleration: Sets acceleration time of the Profile (see above). ‘0’ represents an infinite acceleration.
        :param profile_velocity: Sets velocity of the Profile (see above) . ‘0’ represents an infinite velocity.
        :return:
        """
        self.dxl.robot_set_operating_modes("group", "arm",
                                           mode=INTERBOTIX_MODES[mode],
                                           profile_type=profile_type,
                                           profile_velocity=profile_velocity,
                                           profile_acceleration=profile_acceleration)

    def set_motor_register(self, register: str, value: int, name: t.Optional[str] = None, is_feedthrough: bool = False) -> Future:
        """Set the register value for either a single motor or a group of motors.

        See available registers:
        https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#control-table-of-eeprom-area
        https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#control-table-of-ram-area

        :param register: Desired register name. Naming convention is "Data Name" with spaces replaced with "_".
        :param value: Desired value for the above register.
        :param name: If specified, the method only applies for the specified motor name.
        :param is_feedthrough: True if task is requested by a client.
        :return: Future of the task. Allows for blocking behavior by calling f.result(timeout [s]).
                 This task will **still** executed when self.cancel_all_tasks() is called.
        """
        def _task_set_motor_register(arm: Copilot, register: str, value: int, name: t.Optional[str]):
            if name is None:
                arm.dxl.robot_set_motor_registers("group", arm.GROUP_NAME, register, value)
            else:
                assert name in arm.INFO.joint_names, f"`{name}` is not a registered motor name in group `{arm.GROUP_NAME}`."
                arm.dxl.robot_set_motor_registers("single", name, register, value)
            return EXECUTED

        # Submit task
        f = self._submit_task(is_feedthrough, f"set motor register | register={register}| value={value}", _task_set_motor_register, self, register, value, name)
        return f

    def reboot(self, enable: bool = True,
               smart_reboot: bool = True,
               name: t.Optional[str] = None,
               is_feedthrough: bool = False) -> Future:
        """Reboot a single motor or a group of motors.

        :param enable: True to torque on or False to leave torqued off after rebooting
        :param smart_reboot: Setting this to True will only reboot those motors that are in an error state,
                             as opposed to all motors within the group regardless of if they are in an error state.
        :param name: If specified, the method only applies for the specified motor name.
        :param is_feedthrough: True if task is requested by a client.
        :return: Future of the task. Allows for blocking behavior by calling f.result(timeout [s]).
                 This task will **still** executed when self.cancel_all_tasks() is called.
        """
        def _task_reboot(arm: Copilot, enable: bool, smart_reboot: bool, name: t.Optional[str]):
            if is_feedthrough and not arm._feedthrough:
                return DISCARDED
            if arm.task_event.is_set():
                return CANCELLED
            self.check_status()
            if name is None:
                arm.dxl.robot_reboot_motors("group", arm.GROUP_NAME, enable=enable, smart_reboot=smart_reboot)
            else:
                arm.dxl.robot_reboot_motors("single", name, enable=enable, smart_reboot=smart_reboot)
            return EXECUTED

        # Submit task
        f = self._submit_task(is_feedthrough, "reboot", _task_reboot, self, enable, smart_reboot, name)
        return f

    def pause(self, duration: float, is_feedthrough: bool = False) -> Future:
        """Pause execution for the specified duration.

        :param duration: Duration to pause in seconds.
        :param is_feedthrough: True if task is requested by a client.
        :return: Future of the task. Allows for blocking behavior by calling future.result(timeout [s]).
        """

        def _task_pause(arm: Copilot, d: float):
            if is_feedthrough and not arm._feedthrough:
                return DISCARDED
            if arm.task_event.is_set():
                return CANCELLED
            rate = rospy.Rate(100)
            start = time.time()
            while time.time() - start < d:
                # Check if we should cancel the task due to emergency stop
                if arm.task_event.is_set():
                    return STOPPED
                rate.sleep()
            return EXECUTED

        # Submit task to executor
        f = self._submit_task(is_feedthrough, f"pause | duration={duration}", _task_pause, self, duration)
        return f

    # todo: make option to use MoveIt for this.
    def go_to_home(self, is_feedthrough: bool = False) -> Future:
        """Go to home position.

        :param is_feedthrough: True if task is requested by a client.
        :return: Future of the task. Allows for blocking behavior by calling future.result(timeout [s]).
        """
        # Switch to position operating mode (but remember operating mode before switch)
        mode = (POSITION, "time", 300, 2000)
        # Define how we go to the home position
        with self.task_cond:
            _f_cmd = self.write_commands(self.INFO.num_joints * [0], remap=False, operating_mode=mode, is_feedthrough=is_feedthrough)
            f_pause = self.pause(duration=5.0, is_feedthrough=is_feedthrough)
        return f_pause

    # todo: make option to use MoveIt for this.
    def go_to_sleep(self, is_feedthrough: bool = False) -> Future:
        """Go to sleep position.

        :param is_feedthrough: True if task is requested by a client.
        :return: Future of the task. Allows for blocking behavior by calling future.result(timeout [s]).
        """
        # Switch to position operating mode (but remember operating mode before switch)
        mode = (POSITION, "time", 300, 2000)

        # Define how we go to the sleep position
        with self.task_cond:
            _f_cmd = self.write_commands(self.INFO.joint_sleep_positions, remap=False, operating_mode=mode, is_feedthrough=is_feedthrough)
            f_pause = self.pause(duration=5.0, is_feedthrough=is_feedthrough)
        return f_pause

    def proceed(self) -> None:
        """
        Continue operating after having entered in a stopping procedure.

        All task scheduling is blocked until this function is called.

        This method should be called from a separate thread.
        """
        self._stopping_event.set()

    # todo: remove timeout
    def stop(self, is_feedthrough: bool = False):
        """Stop the robot in its current position. Cancels all scheduled tasks.

        :param is_feedthrough: True if task is requested by a client.
        :returns:
        """

        def _task_stop(arm: Copilot):
            if is_feedthrough and not arm._feedthrough:
                return DISCARDED
            if arm.task_event.is_set():
                return CANCELLED
            # Get current operating mode
            operating_mode = arm.get_operating_mode()
            try:
                mode, profile_type, profile_acceleration, profile_velocity = operating_mode
            except TypeError as e:
                print(f"Did you press/call `stop` immediately in simulation? Then, operating mode might not yet be defined: {e}")
                raise e

            # Initiate emergency stop that is fastest to execute within the current operating mode
            if mode in [POSITION]:  # Position mode
                # Get current position
                position = self.get_joint_states(remap=False).position
                # Apply emergency break (by-pass task scheduling, and directly send command).
                arm.dxl.robot_write_commands(arm.GROUP_NAME, position)
            elif mode in [VELOCITY]:  # Velocity mode
                # Apply emergency break (by-pass task scheduling, and directly send command).
                arm.dxl.robot_write_commands(arm.GROUP_NAME, self.INFO.num_joints * [0])
            else:  # Switch to velocity mode
                # Switch (by-pass task scheduling, and directly switch operating mode)
                arm._set_operating_mode(mode=arm.VELOCITY,
                                        profile_type="time",
                                        profile_velocity=2000,
                                        profile_acceleration=300)
                # Apply emergency break (by-pass task scheduling, and directly send command).
                arm.dxl.robot_write_commands(arm.GROUP_NAME, self.INFO.num_joints * [0])
            # Wait until exit from stopping state (must always be called from separate thread).
            self._stopping_event.wait()
            # Return whether we timed out the stopping procedure
            return EXECUTED

        # Grab condition here (RLock is reentrant, so cancel_all_tasks(), _submit_task() can retake lock multiple times).
        # In this way, we ensure that all tasks are cancelled, directly followed by the stopping task in sequence.

        with self.task_cond:
            # We should cancel first, else stop command can be immediately overwritten
            self.cancel_all_tasks()
            # Enter stopping state
            self._stopping_event.clear()
            # Submit stopping task to executor
            f = self._submit_task(is_feedthrough, "emergency stop", _task_stop, self)
            # f.result()

    def write_commands(self,
                       commands: t.List[float],
                       remap: bool = True,
                       operating_mode: t.Optional[t.Tuple] = None,
                       is_feedthrough: bool = False) -> Future:
        """Command a group of motors.

        :param commands: Desired list of commands. {velocity: [rad/s], position: [rad]}
        :param remap: True will remap the order according to the copilot's set_joint_remapping().
        :param operating_mode: The desired operating mode for executing the command.
                               If unspecified, will take the last scheduled operating mode by the copilot.
        :param is_feedthrough: True if task is requested by a client.
        :return: Future of the task. Allows for blocking behavior by calling f.result(timeout [s]).
                 This task will **still** executed when self.cancel_all_tasks() is called.
                 True if the commands were successfully set, else False (e.g. if not in 'feedthrough' mode).
        """
        # Remap command to joint order (should only apply for commands, directly commanded by copilot).
        if remap and self._to is not None:
            indices = self._to
        else:
            indices = self.INFO.joint_state_indices
        cmd = [commands[i] for i in indices]

        # Ensure there is an operating mode context associated with the command.
        if self._last_op_mode is None and operating_mode is None:
            raise ValueError("An operating mode should first be selected.")

        # Define command task
        def _task_write_commands(arm: Copilot, cmd: t.List[float]):
            if is_feedthrough and not arm._feedthrough:
                return DISCARDED
            if arm.task_event.is_set():
                return CANCELLED
            arm.dxl.robot_write_commands(arm.GROUP_NAME, cmd)
            return EXECUTED  # copilot always successfully sets commands.

        # Grab condition here (RLock is reentrant, so _submit_task can take the lock multiple times).
        # In this way, we ensure that the operating mode switch & command are scheduled in sequence.
        with self.task_cond:
            # Ensure that
            if operating_mode is None:
                _f_op = self.set_operating_mode(*self._last_op_mode, is_feedthrough=is_feedthrough)
            else:
                _f_op = self.set_operating_mode(*operating_mode, is_feedthrough=is_feedthrough)

            # Submit task
            f = self._submit_task(is_feedthrough, f"write command | cmd={cmd}", _task_write_commands, self, cmd)

        return f

    def enable(self, name: t.Optional[str] = None, is_feedthrough: bool = False) -> Future:
        """Torque a single motor or a group of motors to be on.

        :param name: If specified, the method only applies for the specified motor name.
        :param is_feedthrough: True if task is requested by a client.
        :return: Future of the task. Allows for blocking behavior by calling f.result(timeout [s]).
                 This task will **still** executed when self.cancel_all_tasks() is called.
        """
        def _task_enable(arm: Copilot, name: t.Optional[str]):
            if is_feedthrough and not arm._feedthrough:
                return DISCARDED
            if arm.task_event.is_set():
                return CANCELLED
            if name is None:
                arm.dxl.robot_torque_enable("group", arm.GROUP_NAME, True)
            else:
                arm.dxl.robot_torque_enable("single", name, True)
            return EXECUTED

        # Submit task
        f = self._submit_task(is_feedthrough, "enable", _task_enable, self, name)
        return f

    def disable(self, name: t.Optional[str] = None, is_feedthrough: bool = False) -> Future:
        """Torque a single motor or a group of motors to be off.

        :param name: If specified, the method only applies for the specified motor name.
        :param is_feedthrough: True if task is requested by a client.
        :return: Future of the task. Allows for blocking behavior by calling f.result(timeout [s]).
                 This task will **still** executed when self.cancel_all_tasks() is called.
        """
        def _task_disable(arm: Copilot, name: t.Optional[str]):
            if is_feedthrough and not arm._feedthrough:
                return DISCARDED
            if arm.task_event.is_set():
                return CANCELLED
            if name is None:
                arm.dxl.robot_torque_enable("group", arm.GROUP_NAME, False)
            else:
                arm.dxl.robot_torque_enable("single", name, False)
            return EXECUTED

        # Submit task
        f = self._submit_task(is_feedthrough, "disable", _task_disable, self, name)
        return f

    def _handler_set_reg(self, req: RegisterValuesRequest) -> RegisterValuesResponse:
        """**IMPORTANT** New register values are **always** passed through."""
        name = None if req.cmd_type == "group" else req.name
        f = self.set_motor_register(req.reg, req.value, name=name, is_feedthrough=True)
        # Return service only when task has completed/cancelled.
        _success = True if f.result() in [EXECUTED] else False
        return RegisterValuesResponse()

    def _handler_torque(self, req: TorqueEnableRequest) -> TorqueEnableResponse:
        name = None if req.cmd_type == "group" else req.name
        if req.enable:
            f = self.enable(name=name, is_feedthrough=True)
        else:
            f = self.disable(name=name, is_feedthrough=True)
        # Return service only when task has completed/cancelled.
        _success = True if f.result() in [EXECUTED] else False
        return TorqueEnableResponse()

    def _handler_reboot(self, req: RebootRequest) -> RebootResponse:
        name = None if req.cmd_type == "group" else req.name
        f = self.reboot(enable=req.enable, smart_reboot=req.smart_reboot, name=name, is_feedthrough=True)
        # Return service only when task has completed/cancelled.
        _success = True if f.result() in [EXECUTED] else False
        return RebootResponse()

    def _handler_write_commands(self, req: CommandRequest) -> CommandResponse:
        """Returns False if command is not fed through or cancelled."""
        # Get operating context
        op_mode = (req.mode, req.profile_type, req.profile_acceleration, req.profile_velocity)
        # Schedule command (schedules operating mode switch + command in sequence).
        f = self.write_commands(req.cmd, remap=False, operating_mode=op_mode, is_feedthrough=True)
        _success = True if f.result() in [EXECUTED] else False
        return CommandResponse(_success)

    def _handler_stop(self, req: SetBoolRequest) -> SetBoolResponse:
        f = self.stop()
        # Return service only when task has completed/cancelled.
        _success = True if f.result() in [EXECUTED] else False
        return SetBoolResponse(success=_success)

    # todo: how to schedule moveit task?
    # todo: use duration & multiple points
    def _handler_go_to(self, task: FollowJointTrajectoryGoal):
        if self.task_event.is_set():
            status = CANCELLED
        else:
            status = EXECUTED
            for jtp in task.trajectory.points:
                pos = jtp.positions
                # todo: use duration
                secs, nsecs = jtp.time_from_start.secs, jtp.time_from_start.nsecs
                if pos == self.INFO.joint_sleep_positions:
                    f = self.go_to_sleep(is_feedthrough=True)
                    # Wait until we've reached the goal
                    status = f.result()
                elif list(pos) == self.INFO.num_joints * [0]:
                    f = self.go_to_home(is_feedthrough=True)
                    # Wait until we've reached the goal
                    status = f.result()
                else:
                    # todo: use moveit to go to arbitrary positions
                    pass
        self.task_server.set_succeeded()



