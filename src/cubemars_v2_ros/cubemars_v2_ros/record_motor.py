import rclpy
from rclpy.node import Node
import csv
import datetime
import os
from pathlib import Path

from std_msgs.msg import Float64MultiArray, String
from motor_interfaces.msg import MotorState


class RecordMotor(Node):
    def __init__(self):
        super().__init__("record_motor")

        (self.declare_parameter("experiment_name", ""),)
        self.declare_parameter("motor_name")
        self.declare_parameter("save_to", "./")

        self.motor_name = (
            self.get_parameter("motor_name").value
        )
        self.save_to = self.get_parameter("save_to").get_parameter_value().value
        self.experiment_name = (
            self.get_parameter("experiment_name").value
        )

        parent_folder = Path(self.save_to)
        # change to date_experiment_name format?
        save_folder = parent_folder / self.experiment_name / self.motor_name
        save_folder.mkdir(parents=True, exist_ok=True)

        mit_csv_path = (
            save_folder / f"{self.experiment_name}_{self.motor_name}_mit_cmd.csv"
        )
        error_csv_path = (
            save_folder / f"{self.experiment_name}_{self.motor_name}_error_code.csv"
        )
        motor_state_csv_path = (
            save_folder / f"{self.experiment_name}_{self.motor_name}_motor_state.csv"
        )

        self.mit_file = open(mit_csv_path, "w", newline="")
        self.error_file = open(error_csv_path, "w", newline="")
        self.state_file = open(motor_state_csv_path, "w", newline="")

        self.mit_writer = csv.writer(self.mit_file)
        self.error_writer = csv.writer(self.error_file)
        self.state_writer = csv.writer(self.state_file)

        self.mit_writer.writerow(
            ["timestamp", "position_cmd", "velocity_cmd", "kp", "kd", "torque_cmd"]
        )
        self.error_writer.writerow(["timestamp", "error_code"])
        self.state_writer.writerow(
            [
                "timestamp",
                "name",
                "position",
                "abs_position",
                "velocity",
                "torque",
                "current",
                "temperature",
            ]
        )

        self.state_sub = self.create_subscription(
            MotorState, f"/{self.motor_name}/motor_state", self.state_callback, 10
        )

        self.error_sub = self.create_subscription(
            String, f"/{self.motor_name}/error_code", self.error_callback, 10
        )

        self.mit_sub = self.create_subscription(
            Float64MultiArray, f"/{self.motor_name}/mit_cmd", self.mit_callback, 10
        )

    def mit_callback(self, msg):
        timestamp = self.get_clock().now().nanoseconds / 1e-9
        self.mit_writer.writerow([timestamp] + list(msg.data))
        self.mit_file.flush()

    def error_callback(self, msg):
        timestamp = self.get_clock().now().nanoseconds / 1e-9
        self.error_writer.writerow([timestamp, msg.data])
        self.error_file.flush()

    def state_callback(self, msg):
        timestamp = self.get_clock().now().nanoseconds / 1e-9
        self.state_writer.writerow(
            [
                timestamp,
                msg.name,
                msg.position,
                msg.abs_position,
                msg.velocity,
                msg.torque,
                msg.current,
                msg.temperature,
            ]
        )
        self.state_file.flush()

    def __del__(self):
        self.mit_file.close()
        self.error_file.close()
        self.state_file.close()


def main():
    rclpy.init()
    node = RecordMotor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        self.mit_file.close()
        self.error_file.close()
        self.state_file.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
