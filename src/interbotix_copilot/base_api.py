"""
Parts of this copilot were heavily inspired by the interbotix_xs_modules ROS package of trossenrobotics.
"""

import abc
import time
import typing as t
import numpy as np
from threading import Event, Condition
from concurrent.futures import Future, ThreadPoolExecutor
from typing import Tuple, Union, List
import logging

try:
    import modern_robotics as mr
except ImportError as e:
    print(f"Hint: the modern robotics python package must be installed: {e}")
    raise e

try:
    import rospy
    import rosparam
    from rosgraph.masterapi import MasterError
    from sensor_msgs.msg import JointState
    from roscpp.srv import SetLoggerLevel
    from interbotix_xs_modules import core
    import interbotix_xs_modules.mr_descriptions as mrd
    from interbotix_copilot.logging_utils import add_logging_level
    from std_msgs.msg import Bool
except ImportError as e:
    print(f"Hint: ROS (incl. interbotix driver, interbotix_copilot) must be installed & sourced: {e}")
    raise e

# Operating modes:
# https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#operating-mode
EXT_POSITION = 16
PWM = 4
POSITION = 3
VELOCITY = 1
MODES = {POSITION: "Velocity Control Mode",  # This mode controls velocity and ideal for wheel operation.
         VELOCITY: "Position Control Mode",
         # Operating position range is limited by Max Position Limit(48) and Min Position Limit(52).
         EXT_POSITION: "Extended Control Mode",  # This mode controls position and identical to Multi-turn Mode.
         PWM: "PWM Control Mode",  # This mode directly controls PWM output (Voltage Control Mode)
         }
INTERBOTIX_MODES = {POSITION: "position",
                    VELOCITY: "velocity",
                    EXT_POSITION: "ext_position",
                    PWM: "pwm"}

# Error states:
ERROR_STATES = {0: "Input Voltage Error",
                # Bit 0 	Input Voltage Error 	Detects that input voltage exceeds the configured operating voltage
                2: "Overheating Error",
                # Bit 2 	Overheating Error(default) 	Detects that internal temperature exceeds the configured operating temperature
                3: "Motor Encoder Error",
                # Bit 3 	Motor Encoder Error 	Detects malfunction of the motor encoder
                4: "Electrical Shock Error",
                # Bit 4 	Electrical Shock Error(default) 	Detects electric shock on the circuit or insufficient power to operate the motor
                5: "Overload Error"
                # Bit 5 	Overload Error(default) 	Detects that persistent load that exceeds maximum output
                }

# Future states:

DISCARDED = -2
STOPPED = -1
CANCELLED = 0
EXECUTED = 1

STATE_TO_DESCRIPTION_MAP = {
    DISCARDED: "discarded",
    STOPPED: "stopped",
    CANCELLED: "cancelled",
    EXECUTED: "executed",
}


class InterbotixArm:
    VELOCITY = VELOCITY
    POSITION = POSITION
    SUPPORTED_MODES = {POSITION: MODES[POSITION], VELOCITY: MODES[VELOCITY]}

    def __init__(self, robot_type: str, name: str, group_name: str = "arm", gripper_name: str = "gripper"):
        """API to an interbotix robot arm.

        .. note:: All motors in the group must have the same Operating Mode, Profile, Acceleration Profile,
                  and Velocity Profile.

        :param robot_type: Valid Interbotix robot type (e.g. "px150", "vx300s")
        :param name: Name given to the robot. Should match the name given during the driver launch.
        :param group_name: Name of the group of joint motors.
        """
        # Initialize as ros node (if not already initialized).
        try:
            rospy.init_node(f"{name}_interbotix_arm")
        except rospy.exceptions.ROSException:
            pass

        # Initialize interbotix_sdk API
        self.dxl = core.InterbotixRobotXSCore(robot_type, name, False)

        # Extract relevant gripper parameters
        self.GRIPPER_NAME = gripper_name
        self.GRIPPER_INFO = self.dxl.srv_get_info("single", gripper_name)
        self._pressure_op = 0.5
        self._pressure_low = 150
        self._pressure_high = 350

        # Extract relevant arm parameters
        self.NAME = name
        self.GROUP_NAME = group_name
        self.INFO = self.get_info()
        self.task_loggers = self.create_task_loggers()
        self._feedthrough = None
        self.robot_des = getattr(mrd, robot_type)

        # Initialize log level services
        self.srv_set_logger_level = rospy.ServiceProxy(f"/{name}/xs_sdk/set_logger_level", SetLoggerLevel)
        rospy.wait_for_service(f"/{name}/xs_sdk/set_logger_level", timeout=2.0)

        # Determine if dealing with simulation
        self._last_op_mode = None
        self._sim_op_mode = (self.POSITION, "time", 300, 600)  # initial simulation mode
        try:
            self.use_sim = rosparam.get_param("copilot/use_sim")
        except MasterError:
            self.use_sim = False if len(self.get_motor_register("Operating_Mode")) > 0 else True

        # Task executor (one at a time, hence max_workers=1).
        self.task_event = Event()
        self.task_cond = Condition()
        self._last_task = None
        self.executor = ThreadPoolExecutor(max_workers=1)

        # Remap joint measurement/commands based on mapping
        self._to = None
        self._from = None

        # Check status
        if len(self.check_status()) > 0:
            self.reboot(enable=True, smart_reboot=True)
            time.sleep(2.0)
        assert len(self.check_status()) == 0, f"Cannot initialize group `{self.GROUP_NAME}` of robot `{self.NAME}`."

    def open(self) -> Future:
        """Opens the gripper (when in 'pwm' control mode).

        :return: Future of the task. Allows for blocking behavior by calling future.result(timeout [s]).
        """
        f = self.set_pressure(self._pressure_op)
        return f

    def close(self) -> Future:
        """Closes the gripper (when in 'pwm' control mode).

        :return: Future of the task. Allows for blocking behavior by calling future.result(timeout [s]).
        """
        f = self.set_pressure(-self._pressure_op)
        return f

    @abc.abstractmethod
    def _write_gripper_pwm_command(self, pwm: int, is_feedthrough: bool = False) -> Future:
        """Is called in .set_pressure().

        :param pwm: The requested pwm for the gripper.
        :param is_feedthrough: True if task is requested by a client. For clients, this is always False.
        :return: Future of the task. Allows for blocking behavior by calling future.result(timeout [s]).
        """
        pass

    def set_pressure(self, pressure: float) -> Future:
        """
        Set the amount of pressure that the gripper should use when opening/closing the gripper.

        :param pressure: A scaling factor from -1 to 1 where the pressure increases as the absolute factor increases.
                         A value < 0: closes the gripper. A value > 0: opens the gripper.
        :return: Future of the task. Allows for blocking behavior by calling future.result(timeout [s]).
        """
        if pressure < -1 or pressure > 1:
            raise ValueError("The gripper pressure scaling factor must be between [-1, 1]")

        # Calculate pressure pwm signal
        pwm = self._pressure_low + (abs(pressure) * (self._pressure_high - self._pressure_low))

        # Determine the sign, based on scaling factor sign
        if pressure < 0:
            f = self._write_gripper_pwm_command(-int(pwm), is_feedthrough=False)
        else:
            f = self._write_gripper_pwm_command(int(pwm), is_feedthrough=False)
        return f

    def set_pressure_limits(self, operating_pressure: float, low: int = 150, high: int = 350, is_feedthrough: bool = False) -> Future:
        """Set the amount of pressure that the gripper should use when grasping an object (when in 'effort' control mode).

        **IMPORTANT** Pressure limits are **always** set. Be careful with setting the limit too high.

        :param operating_pressure: Fraction between [0, 1] where '0' means the gripper operates at 'low' pressure
                                   and '1' means the gripper operates at 'high' when opening/closing the gripper.
        :param low: Lowest 'effort' that should be applied to the gripper if gripper_pressure is set to 0; it should be high
                    enough to open/close the gripper (~150 PWM).
        :param high: Largest 'effort' that should be applied to the gripper if gripper_pressure is set to 1; it should be low
                     enough that the motor doesn't 'overload' when gripping an object for a few seconds (~350 PWM).
        :param is_feedthrough: True if task is requested by a client. Only concerns the copilot, and always False for client.
        :return: Future of the task. Allows for blocking behavior by calling future.result(timeout [s]).
        """

        def _task_set_pressure_limits(arm: InterbotixArm, _operating_pressure: float, _low: int, _high: int):
            if high < low:
                raise ValueError(f"The upper pressure limit '{_high}' should be higher than the lower pressure limit '{_low}'.")
            if low < 150:
                rospy.logwarn(f"Lower limit `{_low}` may not be high enough to open/close the gripper (low>~150 PWM).")
            if high > 350:
                rospy.logwarn(f"Upper limit `{_high}` may be too high that the motor 'overloads' when gripping an object for a "
                              "few seconds (high<~350 PWM).")
            if _operating_pressure > 1 or _operating_pressure < 0:
                rospy.logerr(f"The operating pressure must be within [0, 1]. Not setting the operating pressure.")
            else:
                arm._pressure_op = _operating_pressure
            arm._pressure_low = _low
            arm._pressure_high = _high
            return EXECUTED

        f = self._submit_task(is_feedthrough, f"set pressure limits | low={low} | high={high} |) ", _task_set_pressure_limits, self, operating_pressure, low, high)
        return f

    def get_gripper_state(self) -> JointState:
        """Get the current gripper position

        :return:
        """
        """Get the current joint states (position, velocity, effort) of all Dynamixel motors.

        :param remap: True will remap the order according to set_joint_remapping().
        :return: JointState ROS message. Refer to online documentation to see its structure.
        """
        indices = self.GRIPPER_INFO.joint_state_indices
        s = self.dxl.robot_get_joint_states()
        if self.use_sim:  # velocities & efforts are not simulated
            s.velocity = [0.0] * len(s.name)
            s.effort = [0.0] * len(s.name)
        # Only include measurements from group.
        s.name = [s.name[i] for i in indices]
        s.position = [s.position[i] for i in indices]
        s.velocity = [s.velocity[i] for i in indices]
        s.effort = [s.effort[i] for i in indices]
        return s

    def create_task_loggers(self):
        # Loggers
        rospy_logger = logging.getLoggerClass()
        logging.setLoggerClass(logging.Logger)
        executedFormat = '<span style="color:green;">{}</span>'
        submittedFormat = '<span style="color:orange;">{}</span>'
        feedthroughFormat = '<span style="color:blue;">{}</span>'
        discardedFormat = '<span style="color:red;">{}</span>'
        cancelledFormat = '<span style="color:red;">{}</span>'
        stoppedFormat = '<span style="color:red;">{}</span>'
        add_logging_level(executedFormat.format("EXECUTED"), logging.INFO+1, "executed")
        add_logging_level(submittedFormat.format("SUBMITTED"), logging.INFO+2, "submitted")
        add_logging_level(feedthroughFormat.format("FEEDTHROUGH"), logging.INFO+3, "feedthrough")
        add_logging_level(discardedFormat.format("DISCARDED"), logging.INFO+4, "discarded")
        add_logging_level(cancelledFormat.format("CANCELLED"), logging.INFO+5, "cancelled")
        add_logging_level(stoppedFormat.format("STOPPED"), logging.INFO+6, "stopped")
        logging.setLoggerClass(rospy_logger)
        loggers = {"FINISHED": logging.getLogger("FINISHED"),
                   "SCHEDULED": logging.getLogger("SCHEDULED")}
        return loggers

    def set_logger_level(self, level: str = "DEBUG"):
        self.srv_set_logger_level(logger="ros.interbotix_xs_sdk", level=level)

    def set_joint_remapping(self, joint_names: t.List[str]):
        """Remap joint measurement/commands based on this index mapping

        :param joint_names: joint names.
        :return:
        """
        assert len(joint_names) == self.INFO.num_joints, "The number of provided joint_names should exactly match the " \
                                                         f"number of registered joint names in group `{self.GROUP_NAME}`."
        self._from = []
        for n in joint_names:
            assert n in self.INFO.joint_names, f"`{n}` is not a registered joint name in group `{self.GROUP_NAME}`."
            i = self.INFO.joint_names.index(n)
            self._from.append(self.INFO.joint_state_indices[i])

        self._to = []
        for n in self.INFO.joint_names:
            assert n in joint_names, f"`{n}` was not provided in the joint name remapping for group `{self.GROUP_NAME}`."
            i = joint_names.index(n)
            self._to.append(i)

    def get_info(self, name: t.Optional[str] = None):
        """Get information about the robot - mostly joint limit data.

        :param name: If specified, the method only applies for the specified motor name.
        :return:
        """
        if name is None:
            return self.dxl.robot_get_robot_info("group", self.GROUP_NAME)
        else:
            return self.dxl.robot_get_robot_info("single", name)

    def get_profile_type(self, name: t.Optional[str] = None) -> str:
        """Get currently activated profile type for either a single motor or a group of motors

        :param name: If specified, the method only applies for the specified motor name.
        :return: The profile type for either a single motor or a group of motors.
        """
        drive_mode = self.get_motor_register("Drive_Mode", name=name)
        if isinstance(drive_mode, tuple):
            profile_type = []
            for value in drive_mode:
                # Bit 2: Profile Configuration  [0] Velocity-based Profile, [1] Time-based Profile
                # https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#drive-mode10
                if bool(value & (1 << 2)):
                    profile_type.append("time")
                else:
                    profile_type.append("velocity")
            assert len(
                set(profile_type)) == 1, f"Different profile types detected in group `{self.GROUP_NAME}` of robot `{self.NAME}`."
            profile_type = profile_type[0]
        else:
            profile_type = "time" if bool(drive_mode & (1 << 2)) else "velocity"
        return profile_type

    def get_operating_mode(self) -> t.Tuple[int, str, int, int]:
        """Get currently activated operating mode for the whole group of motors

        :return: A tuple (operating mode, profile_type, profile_acceleration, profile_velocity) for the motor group.
        """
        if self.use_sim:
            return self._sim_op_mode

        # Get operating mode.
        mode = self.get_motor_register(register="Operating_Mode")
        assert len(
            set(mode)) == 1, f"Different operating modes detected in group `{self.GROUP_NAME}` of robot `{self.NAME}`."
        assert mode[
                   0] in MODES, f"Unknown operating mode detected in group `{self.GROUP_NAME}` of robot `{self.NAME}`. "

        # Get profile
        profile_type = self.get_profile_type()
        profile_acceleration = self.get_motor_register(register="Profile_Acceleration")
        profile_velocity = self.get_motor_register(register="Profile_Velocity")
        assert len(
            set(profile_acceleration)) == 1, f"Different acceleration profiles detected in group `{self.GROUP_NAME}` of robot `{self.NAME}`."
        assert len(
            set(profile_velocity)) == 1, f"Different velocity profiles detected in group `{self.GROUP_NAME}` of robot `{self.NAME}`."

        # Output the register value that is valid for the whole group.
        mode = mode[0]
        profile_acceleration = profile_acceleration[0]
        profile_velocity = profile_velocity[0]

        return mode, profile_type, profile_acceleration, profile_velocity

    def get_ee_pose(self) -> t.Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Get the actual end-effector pose w.r.t the base frame.

        Note that the end-effector is positioned at '<robot_name>/ee_gripper_link'
        and that the Space frame is positioned at '<robot_name>/base_link'.

        :returns: <3x3> Rotation matrix, position vector w.r.t base, <4x4> homogeneous transformation matrix.
        """
        joint_states = [self.dxl.joint_states.position[self.dxl.js_index_map[name]] for name in
                        self.INFO.joint_names]
        T_sb = mr.FKinSpace(self.robot_des.M, self.robot_des.Slist, joint_states)
        rot_mat, position = mr.TransToRp(T_sb)
        return rot_mat.astype("float32"), position.astype("float32"), T_sb.astype("float32")

    @abc.abstractmethod
    def _set_operating_mode(self, current_mode, mode, profile_type, profile_velocity, profile_acceleration):
        """Is called in set_operating_mode().

        :param current_mode: Current operating mode (mode, profile_type, profile_acceleration, profile_velocity).
        :param mode: https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#operating-mode
        :param profile_type: "time" or "velocity".
        :param profile_acceleration: Sets acceleration time of the Profile (see above). ‘0’ represents an infinite acceleration.
        :param profile_velocity: Sets velocity of the Profile (see above) . ‘0’ represents an infinite velocity.
        :return:
        """
        pass

    def set_operating_mode(self, mode: int, profile_type: str = "time", profile_acceleration: int = 300, profile_velocity: int = 2000, is_feedthrough: bool = False) -> Future:
        """Set the operating mode for the whole group of motors.

         .. note:: Velocity Control Mode only uses profile_acceleration.

         Trapezoidal Profiles (t0, t1, t2, t3)
         https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#what-is-the-profile
         https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#profile-acceleration108
         https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#profile-velocity112
         t1: Acceleration time [ms]
            - Time-based profile: profile_acceleration
            - Velocity-based profile: 64 * (profile_velocity / profile_acceleration)
         t2: Constant velocity [ms]
            - Time-based profile: t3 - t
            - Velocity-based profile: 64 * (goal_pos - pos) / profile_velocity
         t3: Total profile time [ms]
            - Time-based profile: profile_velocity
            - Velocity-based profile: t1 + t2
         t3-t2=t1: Deceleration time [ms]
            - Time-based profile: profile_acceleration
            - Velocity-based profile: 64 * (profile_velocity / profile_acceleration)

        :param mode: https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#operating-mode
        :param profile_type: "time" or "velocity".
        :param profile_acceleration: Sets acceleration time of the Profile (see above). ‘0’ represents an infinite acceleration.
        :param profile_velocity: Sets velocity of the Profile (see above) . ‘0’ represents an infinite velocity.
                                 Appears to be ignored in VELOCITY mode. Is overwritten to a value of `0` if mode is VELOCITY.
        :param is_feedthrough: True if task is requested by a client. Only concerns the copilot, and always False for client.
        :return: Future of the task. Allows for blocking behavior by calling f.result(timeout [s]).
                 This task will **still** executed when self.cancel_all_tasks() is called.
        """
        # Overwrite profile_velocity if mode= Velocity (else, it is overwritten in sdk)
        profile_velocity = 0 if mode in [VELOCITY] else profile_velocity

        # Save last commanded op_mode
        _operating_mode = (mode, profile_type, profile_acceleration, profile_velocity)
        self._last_op_mode = _operating_mode

        def _task_set_operating_mode(arm: InterbotixArm, operating_mode: t.Tuple):
            if is_feedthrough and not arm._feedthrough:
                return DISCARDED
            if arm.task_event.is_set():
                return CANCELLED
            _mode, _profile_type, _profile_acceleration, _profile_velocity = operating_mode
            # Switch operating mode
            if mode in arm.SUPPORTED_MODES:
                current_mode = arm.get_operating_mode()
                if not operating_mode == current_mode:
                    rospy.logwarn(f"from={current_mode} | to={operating_mode}")
                    # Set simulation operating mode
                    self._sim_op_mode = operating_mode
                    # Available operating modes in xs_sdk_obj.cpp: 'position', 'linear_position', 'ext_position', 'velocity',
                    # 'pwm', 'current', or 'current_based_position'). I.e. valid values for mode_str.
                    # print("_task_set_operating_mode: ", _mode, _profile_type, _profile_acceleration, _profile_velocity)
                    arm._set_operating_mode(current_mode=current_mode,
                                            mode=mode,
                                            profile_type=profile_type,
                                            profile_velocity=profile_velocity,
                                            profile_acceleration=profile_acceleration)
                    arm.MODE = mode
            elif mode in MODES:
                # Available operating modes in xs_sdk_obj.cpp: 'position', 'linear_position', 'ext_position', 'velocity',
                # 'pwm', 'current', or 'current_based_position'). I.e. valid values for mode_str.
                raise NotImplementedError(f"Mode `{MODES[mode]}` not yet supported.")
            else:
                raise ValueError(f"Unknown operating mode `{mode}`.")
            return EXECUTED

        f = self._submit_task(is_feedthrough, f"set operating mode | mode={INTERBOTIX_MODES[mode]} | type={profile_type} | pa={profile_acceleration} | pv={profile_velocity} |) ", _task_set_operating_mode, self, _operating_mode)
        return f

    def _submit_task(self, is_feedthrough: bool, description: str, task: t.Callable, *args, **kwargs) -> Future:
        """Submit a task.

        :param is_feedthrough: True if task is requested by a client.
        :param description: Description of the scheduled task.
        :param task: Task callable.
        :param args: Arguments to call the task callable with.
        :param kwargs: Optional arguments to call the task callable with.
        :return: A future of the scheduled task.
        """
        with self.task_cond:
            if is_feedthrough:
                status = "feedthrough"
            else:
                status = "submitted"
            self._log_cb(description, status=status)()
            f = self.executor.submit(task, *args, **kwargs)
            f.add_done_callback(self._log_cb(description))
            self._last_task = f
        return f

    def _log_cb(self, description: str, status: t.Optional[str] = None):
        def _log(f: Future = None):
            if isinstance(f, Future):
                log_fn = STATE_TO_DESCRIPTION_MAP[f.result()]
                logger = "FINISHED"
            else:
                log_fn = status
                logger = "SCHEDULED"
            getattr(self.task_loggers[logger], log_fn)(f"{description}")
        return _log

    def get_pid_gains(self, name: t.Optional[str] = None) -> Tuple[
        Union[int, List[int]], Union[int, List[int]], Union[int, List[int]], Union[int, List[int]], Union[
            int, List[int]], Union[int, List[int]], Union[int, List[int]]]:
        """Get the gains for either a single motor or a group of motors.

        source: https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#position-pid-gain80-82-84-feedforward-1st2nd-gains88-90
                    Controller      Gain 	Conversion Equations 	Range 	Description
        Position D  Gain(80) 	    KPD 	KPD = KPD(TBL) / 16 	0 ~ 16,383 	D Gain
        Position I  Gain(82) 	    KPI 	KPI = KPI(TBL) / 65,536 0 ~ 16,383 	I Gain
        Position P  Gain(84) 	    KPP 	KPP = KPP(TBL) / 128 	0 ~ 16,383 	P Gain
        Feedforward 2nd Gain(88) 	KFF2nd 	KFF2nd(TBL) / 4 	    0 ~ 16,383 	Feedforward Acceleration Gain
        Feedforward 1st Gain(90) 	KFF1st 	KFF1st(TBL) / 4 	    0 ~ 16,383 	Feedforward Velocity Gain

        :param name: If specified, the method only applies for the specified motor name.
        :return: The gains for either a single motor or a group of motors.
        """
        kp_pos = self.get_motor_register("Position_P_Gain", name=name)
        ki_pos = self.get_motor_register("Position_I_Gain", name=name)
        kd_pos = self.get_motor_register("Position_D_Gain", name=name)
        kp_vel = self.get_motor_register("Velocity_P_Gain", name=name)
        ki_vel = self.get_motor_register("Velocity_I_Gain", name=name)
        ff_acc = self.get_motor_register("Feedforward_2nd_Gain", name=name)
        ff_vel = self.get_motor_register("Feedforward_1st_Gain", name=name)
        return kp_pos, ki_pos, kd_pos, kp_vel, ki_vel, ff_acc, ff_vel

    def set_pid_gains(self,
                      kp_pos: t.Optional[int] = None,
                      ki_pos: t.Optional[int] = None,
                      kd_pos: t.Optional[int] = None,
                      kp_vel: t.Optional[int] = None,
                      ki_vel: t.Optional[int] = None,
                      ff_acc: t.Optional[int] = None,
                      ff_vel: t.Optional[int] = None,
                      name: t.Optional[str] = None) -> t.List[Future]:
        """Set the internal PID gains for either a single motor or a group of motors.

        :param kp_pos: Passthrough to the Position_P_Gain register.
        :param ki_pos: Passthrough to the Position_I_Gain register.
        :param kd_pos: Passthrough to the Position_D_Gain register.
        :param kp_vel: Passthrough to the Velocity_P_Gain register.
        :param ki_vel: Passthrough to the Velocity_I_Gain register.
        :param ff_acc: Passthrough to the Feedforward_2nd_Gain register.
        :param ff_vel: Passthrough to the Feedforward_1st_Gain register.
        :param name: If specified, the method only applies for the specified motor name.
        :return: Futures of the tasks. Allows for blocking behavior by calling f.result(timeout [s]).
                 This task will **still** executed when self.cancel_all_tasks() is called.
        """
        f = []
        if kp_pos is not None:
            f.append(self.set_motor_register("Position_P_Gain", kp_pos, name=name))
        if ki_pos is not None:
            f.append(self.set_motor_register("Position_I_Gain", ki_pos, name=name))
        if kd_pos is not None:
            f.append(self.set_motor_register("Position_D_Gain", kd_pos, name=name))
        if kp_vel is not None:
            f.append(self.set_motor_register("Velocity_P_Gain", kp_vel, name=name))
        if ki_vel is not None:
            f.append(self.set_motor_register("Velocity_I_Gain", ki_vel, name=name))
        if ff_acc is not None:
            f.append(self.set_motor_register("Feedforward_2nd_Gain", ff_acc, name=name))
        if ff_vel is not None:
            f.append(self.set_motor_register("Feedforward_1nd_Gain", ff_vel, name=name))
        return f

    def get_motor_register(self, register: str, name: t.Optional[str] = None) -> t.Union[int, t.List[int]]:
        """Get the register value for either a single motor or a group of motors.

        See available registers:
        https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#control-table-of-eeprom-area
        https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#control-table-of-ram-area

        :param register: Desired register name. Naming convention is "Data Name" with spaces replaced with "_".
        :param name: If specified, the method only applies for the specified motor name.
        :return: The register values for either a single motor or a group of motors.
        """
        if name is None:
            return self.dxl.robot_get_motor_registers("group", self.GROUP_NAME, register).values
        else:
            assert name in self.INFO.joint_names, f"`{name}` is not a registered motor name in group `{self.GROUP_NAME}`."
            v = self.dxl.robot_get_motor_registers("single", name, register).values
            if len(v) > 0:
                return v[0]
            else:
                assert self.use_sim, "We should only receive empty register values in simulation."
                return tuple()

    @abc.abstractmethod
    def set_motor_register(self, register: str, value: int, name: t.Optional[str] = None) -> Future:
        """Set the register value for either a single motor or a group of motors.

        See available registers:
        https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#control-table-of-eeprom-area
        https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#control-table-of-ram-area

        :param register: Desired register name. Naming convention is "Data Name" with spaces replaced with "_".
        :param value: Desired value for the above register.
        :param name: If specified, the method only applies for the specified motor name.
        :return:
        """
        pass

    def check_status(self) -> t.Union[int, t.List[str]]:
        """Gets the HardwareError status for all motors in the group.

        More info:
        https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#hardware-error-status70

        :return: List of motor names that registered a hardware error.
        """
        motor_error = []
        states = self.get_motor_register("Hardware_Error_Status")
        for name, state in zip(self.INFO.joint_names, states):
            for n, error in ERROR_STATES.items():
                if bool(state & (1 << n)):
                    motor_error.append(name)
                    print(f"[{name}]: {error}", )
        return motor_error

    @abc.abstractmethod
    def reboot(self, enable: bool = True, smart_reboot: bool = True, name: t.Optional[str] = None) -> Future:
        """Reboot a single motor or a group of motors.

        :param enable: True to torque on or False to leave torqued off after rebooting
        :param smart_reboot: Setting this to True will only reboot those motors that are in an error state,
                             as opposed to all motors within the group regardless of if they are in an error state.
        :param name: If specified, the method only applies for the specified motor name.
        :return:
        """
        pass

    @abc.abstractmethod
    def go_to_home(self) -> Future:
        """Go to home position.

        :return: Future of the task. Allows for blocking behavior by calling future.result(timeout [s]).
        """
        pass

    @abc.abstractmethod
    def go_to_sleep(self) -> Future:
        """Go to sleep position.

        :return: Future of the task. Allows for blocking behavior by calling future.result(timeout [s]).
        """
        pass

    def cancel_all_tasks(self):
        """Cancel all scheduled tasks."""
        with self.task_cond:
            # Cancel all running tasks.
            self.task_event.set()
            # Wait for last future to return.
            if isinstance(self._last_task, Future):
                self._last_task.result()
            # Clear cancel event.
            self.task_event.clear()

    @abc.abstractmethod
    def stop(self) -> Future:
        """Stop the robot in its current position. Cancels all scheduled tasks.

        :return: Future of the task. Allows for blocking behavior by calling future.result(timeout [s])."""
        pass

    def get_joint_states(self, remap: bool = True) -> JointState:
        """Get the current joint states (position, velocity, effort) of all Dynamixel motors.

        :param remap: True will remap the order according to set_joint_remapping().
        :return: JointState ROS message. Refer to online documentation to see its structure.
        """
        if remap and self._from is not None:
            indices = self._from
        else:
            indices = self.INFO.joint_state_indices
        s = self.dxl.robot_get_joint_states()
        if self.use_sim:  # velocities & efforts are not simulated
            s.velocity = [0.0] * len(s.name)
            s.effort = [0.0] * len(s.name)
        # Only include measurements from group.
        s.name = [s.name[i] for i in indices]
        s.position = [s.position[i] for i in indices]
        s.velocity = [s.velocity[i] for i in indices]
        s.effort = [s.effort[i] for i in indices]
        return s

    @abc.abstractmethod
    def write_commands(self, commands: t.List[float], remap: bool = True) -> Future:
        """Command a group of motors.

        :param commands: Desired list of commands. {velocity: [rad/s], position: [rad]}
        :param remap: True will remap the order according to set_joint_remapping().
        :return: True if the command was successfully set, else False (e.g. if not in 'feedthrough' mode).
        """
        pass

    def is_busy(self):
        """Check if all scheduled tasks have been completed."""
        return not self._last_task.done()

    @abc.abstractmethod
    def enable(self, name: t.Optional[str] = None) -> Future:
        """Torque a single motor or a group of motors to be on.

        :param name: If specified, the method only applies for the specified motor name.
        :return:
        """
        pass

    @abc.abstractmethod
    def disable(self, name: t.Optional[str] = None) -> Future:
        """Torque a single motor or a group of motors to be off.

        :param name: If specified, the method only applies for the specified motor name.
        :return:
        """
        pass
