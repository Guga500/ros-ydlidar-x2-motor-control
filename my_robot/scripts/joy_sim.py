#!/usr/bin/env python3
import rospy, time, random
from sensor_msgs.msg import Joy

"""
joy_sim.py — จำลองจอยให้ /joy
โหมด:
  - hold:     ค่าคงที่ (axes คงที่) สำหรับเทสตรงไป/เลี้ยว
  - sweep:    ปัดแกนไปกลับแบบไซน์ นุ่มๆ
  - square:   เดินเป็นรูปสี่เหลี่ยม (เดินตรง-หยุด-เลี้ยวคงที่ วน)
  - random:   ค่ากึ่งสุ่ม เปลี่ยนทีละนิดแบบ smooth
ปุ่มจำลอง:
  - STOP toggle (Start)
  - Precision hold (A)
  - Speed up/down (LB/RB)
ใช้ร่วมกับ joy_to_cmdvel_advanced.py ที่เราเซ็ตก่อนหน้า
"""

def clamp(v, a, b): return max(a, min(b, v))

class JoySim:
    def __init__(self):
        self.pub = rospy.Publisher("joy", Joy, queue_size=10)

        # Params (ให้ตรงกับ launch ที่ใช้อยู่)
        self.axis_linear   = rospy.get_param("~axis_linear", 1)   # LY
        self.axis_angular  = rospy.get_param("~axis_angular", 0)  # LX

        # ปุ่มตาม XInput: A=0 B=1 X=2 Y=3 LB=4 RB=5 Back=6 Start=7
        self.btn_precision = rospy.get_param("~btn_precision", 0)  # A
        self.btn_up        = rospy.get_param("~btn_speed_up", 4)   # LB
        self.btn_down      = rospy.get_param("~btn_speed_down", 5) # RB
        self.btn_stop      = rospy.get_param("~btn_stop", 7)       # Start

        self.rate_hz       = rospy.get_param("~rate", 30)
        self.mode          = rospy.get_param("~mode", "sweep")  # hold/sweep/square/random
        self.duration      = rospy.get_param("~duration", 0.0)  # 0=ไม่จำกัดเวลา

        # สำหรับ hold mode
        self.hold_linear   = rospy.get_param("~hold_linear",  0.8)  # -1..1 (ดันหน้า)
        self.hold_angular  = rospy.get_param("~hold_angular", 0.0)

        # random mode smoothing
        self.random_step   = rospy.get_param("~random_step", 0.05)   # ก้าวเปลี่ยนเป้าหมาย/เฟรม
        self.random_bias_f = rospy.get_param("~random_bias_forward", 0.2) # bias ให้เดินหน้า
        self.ax_lin_curr   = 0.0
        self.ax_ang_curr   = 0.0
        self.ax_lin_target = 0.0
        self.ax_ang_target = 0.0

        # square mode settings
        self.straight_secs = rospy.get_param("~square_straight_secs", 2.0)
        self.turn_secs     = rospy.get_param("~square_turn_secs", 1.2)
        self.stop_secs     = rospy.get_param("~square_stop_secs", 0.3)

        self.precision_hold = rospy.get_param("~precision_hold", False)
        self.auto_stop_every = rospy.get_param("~auto_stop_every", 0)  # จำนวนวินาทีเว้น toggle STOP อัตโนมัติ (0=ไม่ใช้)
        self.last_auto_stop = rospy.Time.now()

        self.buttons = [0]*8
        self.axes    = [0.0]*4   # ใช้ 4 แกนพอ (LX,LY,RX,RY)

        self.t0 = rospy.Time.now()

    def publish(self):
        msg = Joy()
        msg.header.stamp = rospy.Time.now()
        msg.axes = list(self.axes)
        msg.buttons = list(self.buttons)
        self.pub.publish(msg)

    # ============ รูปแบบแกนต่างๆ ============
    def mode_hold(self, t):
        self.axes[self.axis_linear]  = clamp(self.hold_linear, -1.0, 1.0)
        self.axes[self.axis_angular] = clamp(self.hold_angular, -1.0, 1.0)

    def mode_sweep(self, t):
        # เดินหน้า (LY ~ +0.7) และเลี้ยวปัดไปกลับช้าๆ
        import math
        self.axes[self.axis_linear]  = 0.7
        self.axes[self.axis_angular] = 0.6*math.sin(0.5*t)

    def mode_square(self, t):
        # วน: เดินตรง -> หยุด -> เลี้ยว -> หยุด -> ซ้ำ
        cyc = self.straight_secs + self.stop_secs + self.turn_secs + self.stop_secs
        tau = (t % cyc)
        if tau < self.straight_secs:
            self.axes[self.axis_linear]  = 0.8
            self.axes[self.axis_angular] = 0.0
        elif tau < self.straight_secs + self.stop_secs:
            self.axes[self.axis_linear]  = 0.0
            self.axes[self.axis_angular] = 0.0
        elif tau < self.straight_secs + self.stop_secs + self.turn_secs:
            self.axes[self.axis_linear]  = 0.0
            self.axes[self.axis_angular] = 0.8  # หมุนคงที่
        else:
            self.axes[self.axis_linear]  = 0.0
            self.axes[self.axis_angular] = 0.0

    def mode_random(self, t):
        # เล็งเป้าหมายใหม่เป็นพักๆ
        if random.random() < 0.02:
            # bias ให้เดินหน้า
            base_lin = random.uniform(-1.0, 1.0) + self.random_bias_f
            self.ax_lin_target = clamp(base_lin, -1.0, 1.0)
            self.ax_ang_target = clamp(random.uniform(-0.8, 0.8), -1.0, 1.0)
        # ค่อยๆ ขยับไปหาเป้าหมาย
        def approach(curr, target, step):
            if curr < target:  curr = min(curr + step, target)
            elif curr > target: curr = max(curr - step, target)
            return curr
        self.ax_lin_curr = approach(self.ax_lin_curr, self.ax_lin_target, self.random_step)
        self.ax_ang_curr = approach(self.ax_ang_curr, self.ax_ang_target, self.random_step)
        self.axes[self.axis_linear]  = self.ax_lin_curr
        self.axes[self.axis_angular] = self.ax_ang_curr

    # ============ ปุ่มจำลอง ============
    def maybe_toggle_stop(self, t):
        if self.auto_stop_every <= 0: return
        if (rospy.Time.now() - self.last_auto_stop).to_sec() >= self.auto_stop_every:
            # edge press ที่ปุ่ม Start
            self.buttons[self.btn_stop] = 1
            self.publish()
            rospy.sleep(0.05)
            self.buttons[self.btn_stop] = 0
            self.last_auto_stop = rospy.Time.now()

    def run(self):
        r = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            t = (rospy.Time.now() - self.t0).to_sec()

            # reset axes/buttons ทุกเฟรมก่อนเขียน
            for i in range(len(self.axes)): self.axes[i] = 0.0
            for i in range(len(self.buttons)): self.buttons[i] = 0

            # precision hold (A) ถ้าต้องการ
            if self.precision_hold:
                self.buttons[self.btn_precision] = 1

            # โหมดหลัก
            if   self.mode == "hold":   self.mode_hold(t)
            elif self.mode == "square": self.mode_square(t)
            elif self.mode == "random": self.mode_random(t)
            else:                       self.mode_sweep(t)

            # ตัวอย่าง: กด Speed up ทุกๆ 5 วินาทีใน 0.1s
            if int(t) % 5 == 0 and (t - int(t)) < 0.1:
                self.buttons[self.btn_up] = 1

            # toggle STOP อัตโนมัติถ้าตั้งไว้
            self.maybe_toggle_stop(t)

            self.publish()
            r.sleep()

if __name__ == "__main__":
    rospy.init_node("joy_sim")
    JoySim().run()
