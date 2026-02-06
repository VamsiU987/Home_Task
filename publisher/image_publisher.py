#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray

import cv2
import numpy as np
import time
from datetime import datetime


def human_timestamp() -> str:
    # Example: 2026-02-06 14:32:18.417
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


class ImagePublisher(Node):
    def __init__(self):
        super().__init__("image_publisher")

        self.publisher_ = self.create_publisher(ByteMultiArray, "image_topic", 10)

        # Target publish rate (FPS). Actual FPS depends on camera/CPU.
        self.target_fps = 30
        self.timer = self.create_timer(1.0 / self.target_fps, self.publish_image)

        self.frame_id = 0

        # FPS measurement (kept for debugging; logging is commented out below)
        self.last_fps_time = time.time()
        self.frame_counter_for_fps = 0

        # Try webcam; if not available, fall back to synthetic images
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().warn("Webcam not found. Using synthetic images.")
            self.cap = None

        # Required console log: when publishing starts
        self.get_logger().info("Publishing started")

    def publish_image(self):
        ts = human_timestamp()

        try:
            # Get a frame
            if self.cap:
                ret, frame = self.cap.read()
                if not ret or frame is None:
                    raise RuntimeError("camera read failed")
            else:
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(
                    frame,
                    "Synthetic Image",
                    (50, 240),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (255, 255, 255),
                    2,
                )

            # Encode to JPEG bytes
            ok, buffer = cv2.imencode(".jpg", frame)
            if not ok:
                raise RuntimeError("JPEG encode failed")

            msg = ByteMultiArray()
            msg.data = buffer.tobytes()

            self.publisher_.publish(msg)

            # -------- FPS logging (COMMENTED OUT BY DEFAULT) --------
            # self.frame_counter_for_fps += 1
            # now = time.time()
            # if now - self.last_fps_time >= 1.0:
            #     fps = self.frame_counter_for_fps / (now - self.last_fps_time)
            #     self.get_logger().info(f"FPS={fps:.2f}")
            #     self.last_fps_time = now
            #     self.frame_counter_for_fps = 0
            # --------------------------------------------------------

            # Required console log per frame
            self.get_logger().info(f"Frame={self.frame_id} | Time={ts} | Status=Sent")

        except Exception as e:
            # Required console log per frame (errors)
            self.get_logger().error(f"Frame={self.frame_id} | Time={ts} | Status=Error ({e})")

        finally:
            self.frame_id += 1

    def destroy_node(self):
        try:
            if self.cap is not None:
                self.cap.release()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = ImagePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
