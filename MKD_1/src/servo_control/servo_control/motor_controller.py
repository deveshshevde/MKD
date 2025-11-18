import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

import board
import busio
from adafruit_pca9685 import PCA9685


class MotorController(Node):
    def __init__(self):
        super().__init__("servo_controller")

        self.subscription = self.create_subscription(
            Joy,
            "/joy",
            self.joy_callback,
            10
        )

        # Init I2C + PCA9685
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50

        # Servo channels
        self.servo1 = 0
        self.servo2 = 1
        self.servo3 = 2

        self.get_logger().info("Servo Controller Node Started (3 servos).")

    def joy_callback(self, msg):
        # Joystick axes
        s1 = msg.axes[0]    # left stick horizontal
        s2 = msg.axes[1]    # left stick vertical
        s3 = msg.axes[3]    # right stick vertical

        pwm1 = self.map_to_pwm(s1)
        pwm2 = self.map_to_pwm(s2)
        pwm3 = self.map_to_pwm(s3)

        self.pca.channels[self.servo1].duty_cycle = pwm1
        self.pca.channels[self.servo2].duty_cycle = pwm2
        self.pca.channels[self.servo3].duty_cycle = pwm3

    def map_to_pwm(self, x):
        # Convert joystick -1..1 → 3000–9000 duty cycle
        min_pwm = 3000
        max_pwm = 9000
        return int(min_pwm + (x + 1) * (max_pwm - min_pwm) / 2)


def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
