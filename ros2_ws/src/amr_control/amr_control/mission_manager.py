#!/usr/bin/env python3
"""AUTO mode: go to a goal, then pick or place there.

Commands (std_msgs/String on mission/command, or use `ros2 run amr_control mission ...`):
  pick [color]                      pick the object in front of the robot
  place                             put the carried object down in front of the robot
  go x y yaw                        drive there in the map frame
  fetch x y yaw [color]             drive so the object at (x,y) is in front, then pick
  deliver x y yaw                   drive so (x,y) is in front, then place
  transfer ax ay ayaw bx by byaw [color]    fetch from a, deliver to b
  cancel

RViz "2D Goal Pose" in AUTO does a fetch, or a deliver when the robot is already carrying something.
The arrow gives the direction of approach.

Each mission is a Python generator that yields once per tick, so it reads top to bottom and
stops cleanly when the mode leaves AUTO.
"""
import json, math, time
import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from amr_control.arm_model import ARM_JOINTS, GRIPPER, POSES, ArmModel

TICK = 0.05


class MissionFailed(Exception):
    pass


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        defaults = dict(arm_speed=0.6, slow_speed=0.25, carry_speed=0.4, grip_speed=0.5, grasp_x=0.31, grasp_y=0.0,
                        tip_height=0.010, hover_height=0.10, place_height=0.014,
                        grip_open=0.40, grip_closed=1.06, standoff=0.45, map_frame='map',
                        align_tolerance=0.006, align_speed=0.07, align_gain=1.2, position_tolerance=0.012,
                        detect_timeout=4.0, nav_timeout=180.0)
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        self.p = lambda n: self.get_parameter(n).value
        self.model = None; self.js = {}; self.mode = 'MANUAL'; self.task = None; self.task_name = None
        self.return_manual = False   # a commanded mission hands control back to the operator when it ends
        self.detections = []; self.det_time = 0.0; self.carrying = False
        self.q_cmd = None; self.grip_cmd = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, 'mission/status', latched)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_auto', 10)
        self.arm_pub = self.create_publisher(JointTrajectory, 'arm_controller/joint_trajectory', 10)
        self.grip_pub = self.create_publisher(JointTrajectory, 'gripper_controller/joint_trajectory', 10)
        self.mode_pub = self.create_publisher(String, 'mode_cmd', 10)
        # Heard only by sim_grasp_assist in Gazebo. On the real robot nothing listens and friction does the job.
        self.grasp_pub = self.create_publisher(String, 'gripper/grasp', 10)
        self.held = None
        self.create_subscription(String, 'robot_description', self.on_description, latched)
        self.create_subscription(String, 'mode', self.on_mode, latched)
        self.create_subscription(JointState, 'joint_states', lambda m: self.js.update(zip(m.name, m.position)), 20)
        self.create_subscription(String, 'object/detections', self.on_detections, 10)
        self.create_subscription(String, 'mission/command', self.on_command, 10)
        self.create_subscription(PoseStamped, 'goal_pose', self.on_goal_pose, 10)
        self.tf_buf = Buffer(); self.tf = TransformListener(self.tf_buf, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_goal = None
        self.create_timer(TICK, self.tick)
        self.say('IDLE')

    # ------------------------------------------------------------- plumbing
    def say(self, text):
        self.get_logger().info(text)
        self.status_pub.publish(String(data=json.dumps(dict(status=text, carrying=self.carrying, task=self.task_name))))

    def on_description(self, msg):
        if self.model is None:
            self.model = ArmModel(msg.data)

    def on_mode(self, msg):
        self.mode = msg.data
        if self.mode != 'AUTO':
            self.abort(f'mode changed to {self.mode}')

    def on_detections(self, msg):
        self.detections = json.loads(msg.data); self.det_time = time.monotonic()

    def abort(self, why):
        if self.task is not None:
            self.task.close(); self.task = None; self.task_name = None
            self.cmd_pub.publish(Twist())
            self.cancel_nav()
            self.say(f'ABORTED: {why}')

    def cancel_nav(self):
        if self.nav_goal is not None:
            self.nav_goal.cancel_goal_async(); self.nav_goal = None

    def release_mode(self):
        if self.return_manual:
            self.return_manual = False
            self.mode_pub.publish(String(data='MANUAL'))

    def on_goal_pose(self, msg):
        if self.mode != 'AUTO':
            self.get_logger().warning('Goal ignored: press AUTO first')
            return
        self.return_manual = False   # the operator chose AUTO and may send more goals, so stay in AUTO
        o = msg.pose.orientation
        yaw = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))
        x, y = msg.pose.position.x, msg.pose.position.y
        self.start('deliver' if self.carrying else 'fetch', self.deliver(x, y, yaw) if self.carrying else self.fetch(x, y, yaw, None))

    def on_command(self, msg):
        a = msg.data.split()
        if not a:
            return
        cmd, args = a[0], a[1:]
        try:
            if cmd == 'cancel':
                self.abort('cancelled')
                self.release_mode()
                return
            self.return_manual = True
            if self.mode != 'AUTO':
                self.mode_pub.publish(String(data='AUTO'))
                self.mode = 'AUTO'
            n = [float(v) for v in args if v.replace('.', '', 1).replace('-', '', 1).isdigit()]
            color = next((v for v in args if v.isalpha()), None)
            if cmd == 'pick':
                self.start('pick', self.pick(color))
            elif cmd == 'place':
                self.start('place', self.place())
            elif cmd == 'go':
                self.start('go', self.goto(*n))
            elif cmd == 'fetch':
                self.start('fetch', self.fetch(n[0], n[1], n[2], color))
            elif cmd == 'deliver':
                self.start('deliver', self.deliver(*n))
            elif cmd == 'transfer':
                self.start('transfer', self.transfer(n, color))
            else:
                self.get_logger().error(f'unknown command {msg.data!r}')
                self.release_mode()
        except (TypeError, IndexError):
            self.get_logger().error(f'bad arguments in {msg.data!r}')
            self.release_mode()

    def start(self, name, gen):
        if self.task is not None:
            self.abort('replaced by a new mission')
        self.task = gen; self.task_name = name
        self.q_cmd = None; self.grip_cmd = None
        self.say(f'START {name}')

    def tick(self):
        if self.task is None:
            return
        if self.model is None or not all(j in self.js for j in ARM_JOINTS):
            return
        try:
            next(self.task)
        except StopIteration:
            self.task = None; name = self.task_name; self.task_name = None
            self.cmd_pub.publish(Twist())
            self.say(f'DONE {name}')
            self.release_mode()
        except MissionFailed as e:
            self.task = None; self.task_name = None
            self.cmd_pub.publish(Twist())
            self.cancel_nav()
            self.say(f'FAILED: {e}')
            self.release_mode()

    # ------------------------------------------------------------- arm primitives
    def publish_arm(self):
        d = Duration(sec=0, nanosec=int(0.12e9))
        self.arm_pub.publish(JointTrajectory(joint_names=list(ARM_JOINTS), points=[
            JointTrajectoryPoint(positions=[float(self.q_cmd[j]) for j in ARM_JOINTS], time_from_start=d)]))

    def publish_grip(self):
        d = Duration(sec=0, nanosec=int(0.12e9))
        self.grip_pub.publish(JointTrajectory(joint_names=[GRIPPER], points=[
            JointTrajectoryPoint(positions=[float(self.grip_cmd)], time_from_start=d)]))

    def arm_to(self, target, speed=None):
        """Joint-space move at a limited speed, all joints arriving together."""
        speed = speed or self.p('arm_speed')
        if self.q_cmd is None:
            self.q_cmd = {j: self.js[j] for j in ARM_JOINTS}
        start = dict(self.q_cmd)
        span = max(abs(target[j] - start[j]) for j in ARM_JOINTS)
        steps = max(1, int(span / speed / TICK))
        for i in range(1, steps + 1):
            f = i / steps
            f = f * f * (3 - 2 * f)     # smooth start and stop
            self.q_cmd = {j: start[j] + (target[j] - start[j]) * f for j in ARM_JOINTS}
            self.publish_arm()
            yield
        for _ in range(int(0.4 / TICK)):     # settle
            self.publish_arm()
            yield

    def grip_to(self, value, speed=None):
        speed = speed or self.p('grip_speed')
        if self.grip_cmd is None:
            self.grip_cmd = self.js.get(GRIPPER, 0.0)
        start = self.grip_cmd
        steps = max(1, int(abs(value - start) / speed / TICK))
        for i in range(1, steps + 1):
            self.grip_cmd = start + (value - start) * i / steps
            self.publish_grip()
            yield
        for _ in range(int(0.4 / TICK)):
            self.publish_grip()
            yield

    def ik(self, x, y, z):
        q = self.model.solve_ik(x, y, z)
        if q is None:
            raise MissionFailed(f'({x:.3f}, {y:.3f}, {z:.3f}) is outside the arm workspace')
        return q

    # ------------------------------------------------------------- navigation primitives
    def pose_in(self, frame):
        t = self.tf_buf.lookup_transform(frame, 'base_link', Time())
        q = t.transform.rotation
        return t.transform.translation.x, t.transform.translation.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def goto(self, x, y, yaw):
        """Nav2 for the route, then direct holonomic control for the last centimetres."""
        yield from self.arm_to(POSES['stow'])
        if not self.nav.wait_for_server(timeout_sec=2.0):
            raise MissionFailed('Nav2 is not running: start with nav:=true')
        for attempt in range(1, 4):
            self.say(f'NAVIGATE to ({x:.2f}, {y:.2f}, {yaw:.2f}), try {attempt}')
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = self.p('map_frame'); goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x = float(x); goal.pose.pose.position.y = float(y)
            goal.pose.pose.orientation.z = math.sin(yaw / 2); goal.pose.pose.orientation.w = math.cos(yaw / 2)
            fut = self.nav.send_goal_async(goal)
            while not fut.done():
                yield
            self.nav_goal = fut.result()
            if not self.nav_goal.accepted:
                raise MissionFailed('Nav2 rejected the goal')
            res = self.nav_goal.get_result_async(); start = time.monotonic()
            while not res.done():
                if time.monotonic() - start > self.p('nav_timeout'):
                    raise MissionFailed('navigation timed out')
                yield
            self.nav_goal = None
            if res.result().status == GoalStatus.STATUS_SUCCEEDED:
                break
            self.get_logger().warning(f'navigation ended with status {res.result().status}')
            for _ in range(int(2.0 / TICK)):     # let the map and costmaps update, then try again
                yield
        else:
            raise MissionFailed('navigation failed three times')
        yield from self.fine_position(x, y, yaw)

    def fine_position(self, x, y, yaw):
        frame = self.p('map_frame'); tol = self.p('position_tolerance')
        start = time.monotonic(); settled = 0
        while settled < 8:
            px, py, pth = self.pose_in(frame)
            ex, ey = x - px, y - py
            eth = math.atan2(math.sin(yaw - pth), math.cos(yaw - pth))
            c, s = math.cos(pth), math.sin(pth)
            bx, by = c * ex + s * ey, -s * ex + c * ey
            t = Twist()
            t.linear.x = max(-0.1, min(0.1, 1.2 * bx)); t.linear.y = max(-0.1, min(0.1, 1.2 * by))
            t.angular.z = max(-0.4, min(0.4, 1.5 * eth))
            self.cmd_pub.publish(t)
            settled = settled + 1 if (math.hypot(ex, ey) < tol and abs(eth) < 0.03) else 0
            if time.monotonic() - start > 15.0:
                self.get_logger().warning('fine positioning timed out, carrying on')
                break
            yield
        self.cmd_pub.publish(Twist())
        for _ in range(int(0.5 / TICK)):
            yield

    # ------------------------------------------------------------- vision
    def nearest(self, color):
        if time.monotonic() - self.det_time > 1.0:
            return None
        items = [d for d in self.detections if color is None or d['color'] == color]
        return min(items, key=lambda d: math.hypot(d['x'], d['y'])) if items else None

    def find_object(self, color):
        """Look straight ahead, then turn a little each way."""
        wz = 0.35
        for rate, secs in ((0, self.p('detect_timeout') / 2), (wz, 1.5), (-wz, 3.0), (wz, 1.5)):
            start = time.monotonic()
            while time.monotonic() - start < secs:
                t = Twist(); t.angular.z = float(rate)
                self.cmd_pub.publish(t)
                d = self.nearest(color)
                if d is not None:
                    self.cmd_pub.publish(Twist())
                    return d
                yield
        self.cmd_pub.publish(Twist())
        raise MissionFailed(f'no {color or "object"} visible')

    def align(self, color):
        """Strafe and creep until the object sits exactly in the grasp spot."""
        gx, gy = self.p('grasp_x'), self.p('grasp_y')
        tol, vmax, gain = self.p('align_tolerance'), self.p('align_speed'), self.p('align_gain')
        start = time.monotonic(); settled = 0; lost = 0
        while settled < 10:
            if time.monotonic() - start > 40.0:
                raise MissionFailed('could not line up with the object')
            d = self.nearest(color)
            if d is None:
                lost += 1
                self.cmd_pub.publish(Twist())
                if lost > 40:
                    raise MissionFailed('lost sight of the object while lining up')
                yield
                continue
            lost = 0
            ex, ey = d['x'] - gx, d['y'] - gy
            t = Twist()
            t.linear.x = max(-vmax, min(vmax, gain * ex)); t.linear.y = max(-vmax, min(vmax, gain * ey))
            self.cmd_pub.publish(t)
            settled = settled + 1 if math.hypot(ex, ey) < tol else 0
            yield
        self.cmd_pub.publish(Twist())
        for _ in range(int(0.8 / TICK)):     # let the picture catch up with the stopped robot
            yield
        return self.nearest(color)

    # ------------------------------------------------------------- missions
    def pick(self, color):
        if self.carrying:
            raise MissionFailed('already carrying something')
        yield from self.arm_to(POSES['stow'])
        yield from self.grip_to(self.p('grip_open'))
        self.say(f'LOOK for {color or "any object"}')
        d = yield from self.find_object(color)
        color = d['color']
        self.say(f'ALIGN on {color} at ({d["x"]:.3f}, {d["y"]:.3f})')
        d = yield from self.align(color)
        if d is None:
            raise MissionFailed('lost sight of the object after lining up')
        x, y = d['x'], d['y']
        self.say(f'PICK {color} at ({x:.3f}, {y:.3f})')
        yield from self.arm_to(self.ik(x, y, self.p('hover_height')))
        yield from self.arm_to(self.ik(x, y, self.p('tip_height')), self.p('slow_speed'))
        yield from self.grip_to(self.p('grip_closed'), 0.25)
        self.grasp_pub.publish(String(data=f'attach {color}'))
        self.held = color
        for _ in range(int(0.5 / TICK)):
            self.publish_grip()
            yield
        yield from self.arm_to(self.ik(x, y, self.p('hover_height')), self.p('slow_speed'))
        yield from self.arm_to(POSES['stow'], self.p('carry_speed'))
        for _ in range(int(1.5 / TICK)):
            yield
        left = self.nearest(color)     # with the arm stowed the camera can see the floor again
        if left is not None and math.hypot(left['x'] - x, left['y'] - y) < 0.12:
            self.grasp_pub.publish(String(data=f'detach {color}')); self.held = None
            yield from self.grip_to(self.p('grip_open'))
            raise MissionFailed(f'{color} is on the floor in front of the robot: the grasp missed or it slipped')
        self.carrying = True
        self.say(f'HOLDING {color}')

    def place(self):
        x, y = self.p('grasp_x'), self.p('grasp_y')
        self.grip_cmd = self.p('grip_closed') if self.carrying else None
        self.say(f'PLACE at ({x:.3f}, {y:.3f}) in front')
        yield from self.arm_to(self.ik(x, y, self.p('hover_height')))
        yield from self.arm_to(self.ik(x, y, self.p('place_height')), self.p('slow_speed'))
        if self.held:
            self.grasp_pub.publish(String(data=f'detach {self.held}')); self.held = None
        yield from self.grip_to(self.p('grip_open'), 0.2)
        self.carrying = False
        yield from self.arm_to(self.ik(x, y, self.p('hover_height')), self.p('slow_speed'))
        yield from self.arm_to(POSES['stow'])

    def approach_pose(self, x, y, yaw, dist):
        return x - dist * math.cos(yaw), y - dist * math.sin(yaw), yaw

    def fetch(self, x, y, yaw, color):
        yield from self.goto(*self.approach_pose(x, y, yaw, self.p('standoff')))
        yield from self.pick(color)

    def deliver(self, x, y, yaw):
        yield from self.goto(*self.approach_pose(x, y, yaw, self.p('grasp_x')))
        yield from self.place()

    def transfer(self, n, color):
        yield from self.fetch(n[0], n[1], n[2], color)
        yield from self.deliver(n[3], n[4], n[5])


def main():
    rclpy.init()
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
