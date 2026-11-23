"""Save the optional Gazebo overhead camera as a short simulation video."""

import json
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class DemoRecorder(Node):
    def __init__(self):
        super().__init__("demo_recorder")
        self.path = Path(self.declare_parameter("video_path", "inspection.mp4").value)
        self.scenario = self.declare_parameter("scenario", "static").value
        self.staging = self.path.with_name(f"{self.path.stem}.partial.mp4")
        self.writer = None
        self.frames = 0
        self.state = "waiting_for_stack"
        self.first_stamp = None
        self.last_frame = None
        self.create_subscription(Image, "/sim/demo_image", self._record, qos_profile_sensor_data)
        self.create_subscription(String, "/mission/state", self._state, 10)

    def _state(self, message):
        self.state = message.data

    def _record(self, message):
        if self.state == "waiting_for_stack":
            return
        if message.encoding not in ("rgb8", "bgr8"):
            self.get_logger().error(f"Unsupported demo camera encoding: {message.encoding}")
            return
        data = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
        frame = data[:, :message.width * 3].reshape(message.height, message.width, 3)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) if message.encoding == "rgb8" else frame.copy()
        if self.writer is None:
            for codec in ("avc1", "mp4v"):
                writer = cv2.VideoWriter(str(self.staging), cv2.VideoWriter_fourcc(*codec),
                                         10.0, (message.width, message.height))
                if writer.isOpened():
                    self.writer = writer
                    break
                writer.release()
            if self.writer is None:
                raise RuntimeError("OpenCV could not open an MP4 encoder")
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        if self.first_stamp is None:
            self.first_stamp = stamp
        label_left = max(0, message.width - 450)
        cv2.rectangle(frame, (label_left, 0), (message.width, 62), (37, 31, 25), -1)
        cv2.putText(frame, f"X30 FACTORY INSPECTION  /  {self.scenario.upper()}", (label_left + 20, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (238, 239, 242), 1, cv2.LINE_AA)
        state = self.state.replace("_", " ").upper()
        cv2.putText(frame, f"{stamp - self.first_stamp:5.1f} s     {state}", (label_left + 20, 49),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 218, 236), 1, cv2.LINE_AA)
        self.writer.write(frame)
        self.last_frame = frame
        self.frames += 1

    def finish(self):
        if self.writer is None:
            return
        if self.last_frame is not None:
            frame = self.last_frame.copy()
            metrics_path = self.path.with_name("metrics.json")
            if metrics_path.is_file():
                result = json.loads(metrics_path.read_text(encoding="utf-8"))
                clearance = result["final_target_clearance_m"]
                if clearance is not None and result["passed"] and self.scenario != "scan_loss":
                    height, width = frame.shape[:2]
                    cv2.rectangle(frame, (0, height - 48), (width, height), (37, 31, 25), -1)
                    cv2.putText(frame, f"Final clearance: {clearance:.2f} m    |    Collisions: {result['collision_events']}",
                                (20, height - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.64,
                                (230, 239, 242), 1, cv2.LINE_AA)
            for _ in range(16):
                self.writer.write(frame)
        self.writer.release()
        if self.frames:
            self.staging.replace(self.path)
            self.get_logger().info(f"Saved {self.frames} frames to {self.path}")


def main():
    rclpy.init()
    recorder = DemoRecorder()
    try:
        rclpy.spin(recorder)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        recorder.finish()
        recorder.destroy_node()
        rclpy.try_shutdown()
