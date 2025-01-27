#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from ros_robot_controller_msgs.msg import MotorsState, MotorState
import numpy as np
import math

class CircleTrajectoryController(Node):
    def __init__(self):
        super().__init__('circle_trajectory_controller')

        # Parameter der Kreisbahn
        self.radius = 0.5  # Radius des Kreises in Metern
        self.angular_velocity = 0.5  # Winkelgeschwindigkeit in rad/s
        self.linear_velocity = self.radius * self.angular_velocity # v = r*w

        # Regelparameter
        self.kp_linear = 1.0  # Proportionalfaktor für lineare Geschwindigkeit
        self.kp_angular = 2.0  # Proportionalfaktor für Winkelgeschwindigkeit

        # Startposition und aktuelle Position
        self.start_position = None
        self.current_position = None
        self.current_orientation = 0.0

        # Mecanum-Chassis-Objekt
        self.mecanum_chassis = MecanumChassis()

        # ROS2 Publisher und Subscriber
        self.motor_pub = self.create_publisher(MotorsState, 'ros_robot_controller/set_motor', 10)
        self.odom_subscriber = self.create_subscription(Odometry, 'odom_raw', self.odom_callback, 10)

        # Kontroll-Timer
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Circle Trajectory Controller gestartet.')

    def odom_callback(self, msg):
        """Callback, um die aktuelle Position und Orientierung aus der Odometrie zu aktualisieren."""
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_orientation = self.quaternion_to_euler(
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        )

        # Startposition speichern
        if self.start_position is None:
            self.start_position = self.current_position

    def control_loop(self):
        """Kontrollschleife zur Trajektorienregelung."""
        if self.current_position is None or self.start_position is None:
            self.get_logger().warning("Warte auf Odometrie-Daten...")
            return

        # Zielposition auf der Kreisbahn berechnen
        elapsed_time = self.get_clock().now().nanoseconds * 1e-9
        target_x = self.start_position[0] + self.radius * np.cos(self.angular_velocity * elapsed_time)
        target_y = self.start_position[1] + self.radius * np.sin(self.angular_velocity * elapsed_time)

        # Fehlerberechnung
        error_x = target_x - self.current_position[0]
        error_y = target_y - self.current_position[1]
        distance_error = math.sqrt(error_x**2 + error_y**2)
        
        # Berechnung des Zielwinkels und Winkelfehlers
        desired_theta = np.arctan2(error_y, error_x)
        angular_error = desired_theta - self.current_orientation
        angular_error = np.arctan2(np.sin(angular_error), np.cos(angular_error))  # Normierung auf [-pi, pi]

        # Berechnung der Steuerung
        linear_speed = self.kp_linear * distance_error
        angular_speed = self.kp_angular * angular_error

        # Geschwindigkeiten begrenzen
        linear_speed = min(linear_speed, self.linear_velocity)
        angular_speed = max(min(angular_speed, 1.0), -1.0)

        # Mecanum-Chassis-Velocity setzen
        motor_command = self.mecanum_chassis.set_velocity(linear_speed, 0.0, angular_speed)
        self.motor_pub.publish(motor_command)

        self.get_logger().info(f"Target: ({target_x:.2f}, {target_y:.2f}), "
                               f"Current: {self.current_position}, "
                               f"Distance Error: {distance_error:.2f}, "
                               f"Angular Error: {angular_error:.2f}")

    @staticmethod
    def quaternion_to_euler(x, y, z, w):
        """Konvertiere Quaternionen in Euler-Winkel."""
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw


class MecanumChassis:
    """Implementierung der Mecanum-Chassis-Kinematik."""
    def __init__(self, wheelbase=0.1368, track_width=0.1410, wheel_diameter=0.065):
        self.wheelbase = wheelbase
        self.track_width = track_width
        self.wheel_diameter = wheel_diameter

    def speed_covert(self, speed):
        return speed / (np.pi * self.wheel_diameter)

    def set_velocity(self, linear_x, linear_y, angular_z):
        motor1 = (linear_x - linear_y - angular_z * (self.wheelbase + self.track_width) / 2)
        motor2 = (linear_x + linear_y - angular_z * (self.wheelbase + self.track_width) / 2)
        motor3 = (linear_x + linear_y + angular_z * (self.wheelbase + self.track_width) / 2)
        motor4 = (linear_x - linear_y + angular_z * (self.wheelbase + self.track_width) / 2)
        v_s = [self.speed_covert(v) for v in [-motor1, -motor2, motor3, motor4]]
        data = []
        for i in range(len(v_s)):
            msg = MotorState()
            msg.id = i + 1
            msg.rps = float(v_s[i])
            data.append(msg)
        msg = MotorsState()
        msg.data = data
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = CircleTrajectoryController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node wird beendet... Roboter wird gestoppt.")
    finally:
        # Stop-Botschaft an die Motoren senden
        stop_command = node.mecanum_chassis.set_velocity(0.0, 0.0, 0.0)
        node.motor_pub.publish(stop_command)

        node.get_logger().info("Roboter gestoppt. Node wird zerstört.")
        node.destroy_node()
        rclpy.shutdown()

    #rclpy.spin(node)
    #node.destroy_node()
    #rclpy.shutdown()


if __name__ == '__main__':
    main()