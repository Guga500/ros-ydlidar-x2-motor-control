#!/usr/bin/env python3
import rospy, time
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from math import copysign

# ===================== Helpers =====================
def clamp(v, lo, hi): return max(lo, min(hi, v))

def apply_deadzone(x, dz):
    if abs(x) <= dz: return 0.0
    # รีแมพช่วงที่เหลือให้เต็มระยะ -1..1
    return copysign((abs(x)-dz)/(1.0-dz), x)

def expo_curve(x, expo):  # expo=0..1 (0=เส้นตรง, 1=โค้งจัด)
    return copysign(abs(x)**(1.0+2.0*expo), x)

def ema(prev, x, alpha):  # alpha: 0..1 (มาก=ตามไว)
    return alpha*x + (1.0-alpha)*prev

def slew_limit(prev, target, max_step):
    dv = target - prev
    if dv >  max_step: return prev + max_step
    if dv < -max_step: return prev - max_step
    return target

# ===================== State / Defaults =====================
linear_scale   = 0.5
angular_scale  = 1.0
linear_min     = 0.1
linear_max     = 1.0
angular_min    = 0.2
angular_max    = 2.0

deadzone       = 0.08     # ปัดสัญญาณจอยหลวม ๆ
expo_linear    = 0.25     # 0..1
expo_angular   = 0.20

alpha_filter   = 0.35     # EMA ของแกนจอย
slew_lin_max   = 0.20     # m/s ต่อเฟรม
slew_ang_max   = 0.60     # rad/s ต่อเฟรม

precision_scale = 0.3
stop_mode       = False
precision_mode  = False

# ปุ่ม/แกน (ปรับผ่าน ~params ได้)
BTN_SPEED_UP   = 4
BTN_SPEED_DOWN = 5
BTN_PRECISION  = 0
BTN_STOP       = 7
AXIS_LINEAR    = 1
AXIS_ANGULAR   = 3
INVERT_LINEAR  = False
INVERT_ANGULAR = False

# Watchdog
joy_timeout_s  = 0.5
last_joy_time  = 0.0

# Filtering states
last_buttons   = []
ax_lin_f       = 0.0
ax_ang_f       = 0.0
cmd_lin_prev   = 0.0
cmd_ang_prev   = 0.0

pub = None

def load_params():
    global BTN_SPEED_UP, BTN_SPEED_DOWN, BTN_PRECISION, BTN_STOP
    global AXIS_LINEAR, AXIS_ANGULAR, INVERT_LINEAR, INVERT_ANGULAR
    global linear_scale, angular_scale, linear_min, linear_max, angular_min, angular_max
    global deadzone, expo_linear, expo_angular, alpha_filter
    global slew_lin_max, slew_ang_max, precision_scale, joy_timeout_s

    BTN_SPEED_UP   = rospy.get_param("~btn_speed_up",   BTN_SPEED_UP)
    BTN_SPEED_DOWN = rospy.get_param("~btn_speed_down", BTN_SPEED_DOWN)
    BTN_PRECISION  = rospy.get_param("~btn_precision",  BTN_PRECISION)
    BTN_STOP       = rospy.get_param("~btn_stop",       BTN_STOP)

    AXIS_LINEAR    = rospy.get_param("~axis_linear",    AXIS_LINEAR)
    AXIS_ANGULAR   = rospy.get_param("~axis_angular",   AXIS_ANGULAR)
    INVERT_LINEAR  = rospy.get_param("~invert_linear",  INVERT_LINEAR)
    INVERT_ANGULAR = rospy.get_param("~invert_angular", INVERT_ANGULAR)

    linear_scale   = rospy.get_param("~linear_scale",   linear_scale)
    angular_scale  = rospy.get_param("~angular_scale",  angular_scale)
    linear_min     = rospy.get_param("~linear_min",     linear_min)
    linear_max     = rospy.get_param("~linear_max",     linear_max)
    angular_min    = rospy.get_param("~angular_min",    angular_min)
    angular_max    = rospy.get_param("~angular_max",    angular_max)

    deadzone       = rospy.get_param("~deadzone",       deadzone)
    expo_linear    = rospy.get_param("~expo_linear",    expo_linear)
    expo_angular   = rospy.get_param("~expo_angular",   expo_angular)
    alpha_filter   = rospy.get_param("~alpha_filter",   alpha_filter)

    slew_lin_max   = rospy.get_param("~slew_lin_max",   slew_lin_max)
    slew_ang_max   = rospy.get_param("~slew_ang_max",   slew_ang_max)
    precision_scale= rospy.get_param("~precision_scale",precision_scale)

    joy_timeout_s  = rospy.get_param("~joy_timeout_s",  joy_timeout_s)

def send_zero():
    cmd = Twist()
    cmd.linear.x = 0.0
    cmd.angular.z = 0.0
    pub.publish(cmd)

# ===================== Callback =====================
def callback(data):
    global last_buttons, stop_mode, precision_mode, last_joy_time
    global linear_scale, angular_scale, ax_lin_f, ax_ang_f
    global cmd_lin_prev, cmd_ang_prev

    # guard sizes
    if not data.axes or not data.buttons:
        rospy.logwarn_throttle(1.0, "Empty Joy message")
        return
    if AXIS_LINEAR   >= len(data.axes) or AXIS_ANGULAR >= len(data.axes):
        rospy.logwarn_throttle(1.0, "Joystick axes index out of range")
        return
    if BTN_SPEED_UP  >= len(data.buttons) or BTN_SPEED_DOWN >= len(data.buttons) \
       or BTN_PRECISION >= len(data.buttons) or BTN_STOP >= len(data.buttons):
        rospy.logwarn_throttle(1.0, "Joystick buttons index out of range")
        return

    if not last_buttons:
        last_buttons[:] = [0]*len(data.buttons)

    last_joy_time = time.time()

    # --- Toggle STOP ---
    if data.buttons[BTN_STOP] and not last_buttons[BTN_STOP]:
        stop_mode = not stop_mode
        rospy.loginfo("[Joystick] STOP %s" % ("ON" if stop_mode else "OFF"))
        if stop_mode:
            send_zero()

    # ใน STOP โหมด ให้ค้าง 0 ตลอด จนกว่าปิด
    if stop_mode:
        last_buttons[:] = data.buttons
        send_zero()
        return

    # --- Speed scale UP/DOWN (edge trigger) ---
    if data.buttons[BTN_SPEED_UP] and not last_buttons[BTN_SPEED_UP]:
        linear_scale  += 0.05
        angular_scale += 0.10
        rospy.loginfo("Speed UP -> linear:%.2f angular:%.2f" % (linear_scale, angular_scale))
    if data.buttons[BTN_SPEED_DOWN] and not last_buttons[BTN_SPEED_DOWN]:
        linear_scale  -= 0.05
        angular_scale -= 0.10
        rospy.loginfo("Speed DOWN -> linear:%.2f angular:%.2f" % (linear_scale, angular_scale))

    # clamp scales
    linear_scale  = clamp(linear_scale,  linear_min,  linear_max)
    angular_scale = clamp(angular_scale, angular_min, angular_max)

    # --- Precision (hold) ---
    precision_mode = (data.buttons[BTN_PRECISION] == 1)
    hold_scale = precision_scale if precision_mode else 1.0

    # --- Read axes ---
    ax_lin = data.axes[AXIS_LINEAR]
    ax_ang = data.axes[AXIS_ANGULAR]
    if INVERT_LINEAR:  ax_lin = -ax_lin
    if INVERT_ANGULAR: ax_ang = -ax_ang

    # --- Deadzone + expo + EMA filter ---
    ax_lin = apply_deadzone(ax_lin, deadzone)
    ax_ang = apply_deadzone(ax_ang, deadzone)

    ax_lin = expo_curve(ax_lin, expo_linear)
    ax_ang = expo_curve(ax_ang, expo_angular)

    ax_lin_f = ema(ax_lin_f, ax_lin, alpha_filter)
    ax_ang_f = ema(ax_ang_f, ax_ang, alpha_filter)

    # --- Scale & clamp ---
    target_lin = clamp(ax_lin_f * linear_scale  * hold_scale, -linear_max,  linear_max)
    target_ang = clamp(ax_ang_f * angular_scale * hold_scale, -angular_max, angular_max)

    # --- Slew rate limiting (จำกัดอัตราเปลี่ยนแปลงต่อ callback) ---
    cmd_lin = slew_limit(cmd_lin_prev, target_lin, slew_lin_max)
    cmd_ang = slew_limit(cmd_ang_prev, target_ang, slew_ang_max)

    # publish
    cmd = Twist()
    cmd.linear.x  = cmd_lin
    cmd.angular.z = cmd_ang
    pub.publish(cmd)

    cmd_lin_prev, cmd_ang_prev = cmd_lin, cmd_ang
    last_buttons[:] = data.buttons

# ===================== Watchdog Timer =====================
def watchdog_timer(event):
    global cmd_lin_prev, cmd_ang_prev
    if last_joy_time == 0.0:  # ยังไม่เคยได้ joy
        return
    if (time.time() - last_joy_time) > joy_timeout_s:
        # fail-safe: ปล่อย 0 แล้วรีเซ็ตสถานะฟิลเตอร์เพื่อไม่ให้ดีดตอนกลับมา
        cmd_lin_prev = 0.0
        cmd_ang_prev = 0.0
        send_zero()
        rospy.logwarn_throttle(2.0, "[Joystick] timeout -> cmd_vel = 0")

# ===================== Main =====================
if __name__ == "__main__":
    rospy.init_node("joy_to_cmdvel_safe")

    load_params()
    rospy.loginfo("[Joystick Node] Ready. Toggle STOP with Start button.")

    pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    sub = rospy.Subscriber("joy", Joy, callback)

    # watchdog 20 Hz
    #rospy.Timer(rospy.Duration(0.05), watchdog_timer)

    rospy.spin()

