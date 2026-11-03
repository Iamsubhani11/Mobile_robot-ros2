"""Shared by the keyboard and joystick teleops: base twist and arm target publishing."""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from amr_control.arm_model import ARM_JOINTS, GRIPPER, POSES, ArmModel


class TeleopCore(Node):
    def __init__(self, name):
        super().__init__(name)
        for p, d in (('linear_speed', 0.3), ('angular_speed', 1.0), ('arm_speed', 0.8), ('gripper_speed', 1.2)):
            self.declare_parameter(p, d)
        self.linear = self.get_parameter('linear_speed').value
        self.angular = self.get_parameter('angular_speed').value
        self.arm_speed = self.get_parameter('arm_speed').value
        self.grip_speed = self.get_parameter('gripper_speed').value
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_teleop', 10)
        self.mode_pub = self.create_publisher(String, 'mode_cmd', 10)
        self.mission_pub = self.create_publisher(String, 'mission/command', 10)
        self.mode = 'MANUAL'; self.latest = {}
        self.arm_pub = self.create_publisher(JointTrajectory, 'arm_controller/joint_trajectory', 10)
        self.grip_pub = self.create_publisher(JointTrajectory, 'gripper_controller/joint_trajectory', 10)
        self.model = None; self.q = None; self.grip = 0.0; self.goal = None; self.dirty = False
        self.blocked = False
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'robot_description', self.on_description, latched)
        self.create_subscription(JointState, 'joint_states', self.on_joint_states, 20)
        self.create_subscription(String, 'mode', self.on_mode, latched)

    def on_description(self, msg):
        if self.model is None:
            self.model = ArmModel(msg.data)
            self.get_logger().info('Arm limits loaded from robot_description')

    def on_mode(self, msg):
        if msg.data != self.mode:
            self.mode = msg.data
            self.goal = None
            self.q = None     # next joint state re-syncs, so manual picks up from where AUTO left the arm

    def on_joint_states(self, msg):
        self.latest = dict(zip(msg.name, msg.position))
        if self.q is None and all(j in self.latest for j in ARM_JOINTS):     # start from where the arm actually is
            self.q = {j: self.latest[j] for j in ARM_JOINTS}
            self.grip = self.latest.get(GRIPPER, 0.0)

    def set_mode(self, mode):
        self.mode_pub.publish(String(data=mode))

    def send_mission(self, text):
        self.mission_pub.publish(String(data=text))

    @property
    def ready(self):
        return self.model is not None and self.q is not None

    def nudge(self, rates, grip_rate, dt):
        """Move arm targets by joint rates (rad/s). Any manual input cancels a named-pose move."""
        if not self.ready or self.mode != 'MANUAL':
            return
        if any(rates.values()) or grip_rate:
            self.goal = None
        for j, r in rates.items():
            if not r:
                continue
            trial = dict(self.q)
            trial[j] = self.model.clamp(j, trial[j] + r * dt)
            self.blocked = self.model.collides(trial) and not self._escaping(trial)
            if not self.blocked:
                self.q = trial; self.dirty = True
        if grip_rate:
            self.grip = self.model.clamp(GRIPPER, self.grip + grip_rate * dt); self.dirty = True

    def _escaping(self, trial):
        return self.model.collides(self.q)   # already inside the keep-out zone: let the operator back out

    def go_to(self, name):
        if self.ready and name in POSES:
            self.goal = POSES[name]

    def step_goal(self, dt):
        if self.goal is None or not self.ready or self.mode != 'MANUAL':
            return
        done = True
        for j, target in self.goal.items():
            err = target - self.q[j]
            step = self.arm_speed * dt
            if abs(err) > step:
                done = False
            self.q[j] += max(-step, min(step, err))
        self.dirty = True
        if done:
            self.goal = None

    def publish_arm(self, horizon=0.12):
        if not self.dirty or not self.ready or self.mode != 'MANUAL':
            return
        self.dirty = False
        duration = Duration(sec=0, nanosec=int(horizon * 1e9))
        arm = JointTrajectory(joint_names=list(ARM_JOINTS))
        arm.points = [JointTrajectoryPoint(positions=[self.q[j] for j in ARM_JOINTS], time_from_start=duration)]
        self.arm_pub.publish(arm)
        grip = JointTrajectory(joint_names=[GRIPPER])
        grip.points = [JointTrajectoryPoint(positions=[self.grip], time_from_start=duration)]
        self.grip_pub.publish(grip)

    def drive(self, vx, vy, wz):
        t = Twist(); t.linear.x = float(vx); t.linear.y = float(vy); t.angular.z = float(wz)
        self.cmd_pub.publish(t)
