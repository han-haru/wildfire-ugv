#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class CmdVelRampToTracks(Node):
    """
    새 cmd_vel이 들어오면 '현재 출력값'에서 '목표값'까지 정확히 rise_time초(T) 동안
    S-커브로 보간해 속도를 만들고, 그걸 6개 바퀴 각속도로 변환해 publish.
    """
    def __init__(self):
        super().__init__('cmdvel_ramp_to_tracks')

        # 차량/주행 파라미터
        self.declare_parameter('forward_sign', -1.0)   # +1(기본) 또는 -1 (앞/뒤 반전)
        self.declare_parameter('left_sign',    1.0)   # 좌측 바퀴만 뒤집고 싶으면 -1
        self.declare_parameter('right_sign',   1.0)   # 우측 바퀴만 뒤집고 싶으면 -1
        self.declare_parameter('wheel_radius', 0.10)       # r [m]
        self.declare_parameter('track_separation', 0.32)   # b [m]
        self.declare_parameter('left_wheels',  [
            'left_wheel_1_joint','left_wheel_2_joint','left_wheel_3_joint'
        ])
        self.declare_parameter('right_wheels', [
            'right_wheel_1_joint','right_wheel_2_joint','right_wheel_3_joint'
        ])

        # 램프/보호 파라미터
        self.declare_parameter('rise_time', 0.5)           # T [s] → 고정 상승시간
        self.declare_parameter('timeout', 0.5)             # 데드맨 타임아웃 [s]
        self.declare_parameter('use_feedback', True)       # joint_states로 현재 속도 추정
        self.declare_parameter('w_limit', 0.0)             # 0이면 무제한, >0이면 각속도 제한(rad/s)

        # 토픽 이름
        self.declare_parameter('topic_in', '/revised/cmd_vel') 
        self.declare_parameter('topic_out', '/wheel_test_controller/commands')

        # 파라미터 로드
        self.r = float(self.get_parameter('wheel_radius').value)
        self.b = float(self.get_parameter('track_separation').value)
        self.L = [str(s) for s in self.get_parameter('left_wheels').value]
        self.R = [str(s) for s in self.get_parameter('right_wheels').value]
        self.T = max(1e-3, float(self.get_parameter('rise_time').value))
        self.timeout = float(self.get_parameter('timeout').value)
        self.use_fb = bool(self.get_parameter('use_feedback').value)
        self.w_lim = float(self.get_parameter('w_limit').value)
        self.forward_sign = float(self.get_parameter('forward_sign').value)
        self.left_sign    = float(self.get_parameter('left_sign').value)
        self.right_sign   = float(self.get_parameter('right_sign').value)

        # I/O
        self.pub = self.create_publisher(Float64MultiArray,
                                         self.get_parameter('topic_out').value, 10)
        self.sub_cmd = self.create_subscription(Twist,
                                                self.get_parameter('topic_in').value,
                                                self.on_cmd, 10)
        if self.use_fb:
            self.sub_js = self.create_subscription(JointState, '/joint_states',
                                                   self.on_js, 20)
        self.motor_pub = self.create_publisher(Twist, '/motor/cmd_vel', 10)

        # 내부 상태
        self.last_rx = self.get_clock().now()
        self.timer = self.create_timer(0.02, self.tick)  # 50 Hz

        # 현재 출력(보간 값)
        self.v_out = 0.0
        self.w_out = 0.0

        # 피드백(현재 실제 속도 추정)
        self.v_cur = 0.0
        self.w_cur = 0.0

        # 램프 상태
        self._ramp_active = False
        self._t0 = self.get_clock().now()
        self._v0 = 0.0
        self._w0 = 0.0
        self._v1 = 0.0
        self._w1 = 0.0

    # S-커브(매끈한 시작/끝). 선형 원하면 return s
    def _shape(self, s: float) -> float:
        s = max(0.0, min(1.0, s))
        return 6*s**5 - 15*s**4 + 10*s**3

    def on_cmd(self, msg: Twist):
        # 새 목표 도착 → 현 출력에서 목표까지 T초 보간 시작
        self._v0, self._w0 = self.v_out, self.w_out
        self._v1, self._w1 = msg.linear.x, msg.angular.z
        self._t0 = self.get_clock().now()
        self._ramp_active = True
        self.last_rx = self._t0

    def on_js(self, msg: JointState):
        # 좌/우 평균 각속도 → v, w 추정
        idx = {n:i for i,n in enumerate(msg.name)}
        def avg(names):
            vals = [msg.velocity[idx[n]] for n in names if n in idx]
            return sum(vals)/len(vals) if vals else 0.0
        wL = avg(self.L); wR = avg(self.R)
        self.v_cur = self.r * (wR + wL) / 2.0
        self.w_cur = self.r * (wR - wL) / self.b

    def _limit(self, x, lim):
        return max(-lim, min(lim, x))

    def tick(self):
        now = self.get_clock().now()

        # 데드맨: 입력이 끊기면 0으로 램프
        if (now - self.last_rx) > Duration(seconds=self.timeout) and not self._ramp_active:
            self._v0, self._w0 = self.v_out, self.w_out
            self._v1, self._w1 = 0.0, 0.0
            self._t0 = now
            self._ramp_active = True

        # 램프 진행
        if self._ramp_active:
            s = (now - self._t0).nanoseconds * 1e-9 / self.T
            if s >= 1.0:
                self.v_out, self.w_out = self._v1, self._w1
                self._ramp_active = False
            else:
                a = self._shape(s)   # 선형 원하면: a = s
                self.v_out = self._v0 + (self._v1 - self._v0) * a
                self.w_out = self._w0 + (self._w1 - self._w0) * a

        # 차동 기하 → 각바퀴 속도
        wL = self.forward_sign * self.left_sign  * (self.v_out - 0.5*self.b*self.w_out) / self.r
        wR = self.forward_sign * self.right_sign * (self.v_out + 0.5*self.b*self.w_out) / self.r

        # 선택: 각속도 제한
        if self.w_lim > 0.0:
            wL = self._limit(wL, self.w_lim)
            wR = self._limit(wR, self.w_lim)

        out = Float64MultiArray()
        out.data = [wL, wL, wL, wR, wR, wR]
        self.pub.publish(out)
        # --- 현재 모터의 선속도/각속도 퍼블리시 ---
        twist_msg = Twist()
        twist_msg.linear.x  = self.v_out   # [m/s]
        twist_msg.angular.z = self.w_out   # [rad/s]
        self.motor_pub.publish(twist_msg)

def main():
    rclpy.init()
    node = CmdVelRampToTracks()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
