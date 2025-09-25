import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

def apply_deadzone(x, dz):
    if abs(x) <= dz:
        return 0.0
    # re-scale after deadzone
    s = (abs(x) - dz) / (1.0 - dz)
    return math.copysign(s, x)

def expo_curve(x, expo):
    # expo in [0,1]: soft near center, stronger near edge
    return math.copysign(abs(x) ** (1.0 + 2.0 * expo), x)

class JoyToTwist(Node):
    def __init__(self):
        super().__init__('gp650_turtlesim_teleop')

        # Parameters (ปรับได้จาก YAML)
        self.declare_parameter('axis_linear', 1)          # แกนเดินหน้า/ถอยหลัง (ทั่วไป=Left Y)
        self.declare_parameter('axis_angular', 0)         # แกนหมุนซ้ายขวา (ทั่วไป=Left X)
        self.declare_parameter('scale_linear', 2.0)       # ความเร็วหน้า/หลังสูงสุด
        self.declare_parameter('scale_angular', 2.0)      # อัตราหมุนสูงสุด
        self.declare_parameter('deadzone', 0.10)
        self.declare_parameter('expo', 0.25)              # โค้งเอ็กซ์โป
        self.declare_parameter('enable_button', -1)       # ปุ่มกดค้างเพื่อขับ (เช่น A=0) ; -1 = ไม่ต้องกด
        self.declare_parameter('invert_linear', False)
        self.declare_parameter('invert_angular', False)

        self.axis_lin = self.get_parameter('axis_linear').get_parameter_value().integer_value
        self.axis_ang = self.get_parameter('axis_angular').get_parameter_value().integer_value
        self.k_lin    = float(self.get_parameter('scale_linear').value)
        self.k_ang    = float(self.get_parameter('scale_angular').value)
        self.deadzone = float(self.get_parameter('deadzone').value)
        self.expo     = float(self.get_parameter('expo').value)
        self.en_btn   = int(self.get_parameter('enable_button').value)
        self.inv_lin  = bool(self.get_parameter('invert_linear').value)
        self.inv_ang  = bool(self.get_parameter('invert_angular').value)

        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.sub = self.create_subscription(Joy, '/joy', self.on_joy, 10)

        self.enabled = (self.en_btn < 0)  # ถ้าไม่ตั้งปุ่ม ให้ขับได้เลย

        self.get_logger().info('GP-650 → turtlesim ready.')

    def on_joy(self, msg: Joy):
        # enable logic
        if self.en_btn >= 0 and self.en_btn < len(msg.buttons):
            self.enabled = (msg.buttons[self.en_btn] == 1)

        # อ่านแกน
        try:
            lx = msg.axes[self.axis_ang]   # left X
            ly = msg.axes[self.axis_lin]   # left Y
        except IndexError:
            return

        # invert ตามต้องการ
        if self.inv_lin:  ly = -ly
        if self.inv_ang:  lx = -lx

        # deadzone + expo
        lx = expo_curve(apply_deadzone(lx, self.deadzone), self.expo)
        ly = expo_curve(apply_deadzone(ly, self.deadzone), self.expo)

        twist = Twist()
        if self.enabled:
            twist.linear.x  = self.k_lin * ly
            twist.angular.z = self.k_ang * lx
        else:
            twist.linear.x  = 0.0
            twist.angular.z = 0.0

        self.pub.publish(twist)

def main():
    rclpy.init()
    node = JoyToTwist()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

