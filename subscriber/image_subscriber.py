import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray
import cv2
import numpy as np
import os
import json
import time

OUTPUT_DIR = 'output/images'
METADATA_FILE = 'output/metadata.json'


def to_bytes_ros(msg_data) -> bytes:
    if isinstance(msg_data, (bytes, bytearray, memoryview)):
        return bytes(msg_data)
    try:
        if len(msg_data) > 0 and isinstance(msg_data[0], (bytes, bytearray, memoryview)):
            return b"".join(bytes(x) for x in msg_data)
        return bytes(msg_data)
    except TypeError:
        return bytes(bytearray(msg_data))


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            ByteMultiArray,
            'image_topic',
            self.callback,
            10
        )

        os.makedirs(OUTPUT_DIR, exist_ok=True)
        os.makedirs(os.path.dirname(METADATA_FILE), exist_ok=True)

        if not os.path.exists(METADATA_FILE):
            with open(METADATA_FILE, 'w', encoding='utf-8') as f:
                json.dump([], f)

        self.frame_id = 0
        self.get_logger().info('Subscriber started')

    def callback(self, msg):
        timestamp = time.time()
        human_ts = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(timestamp)) + f".{int((timestamp % 1)*1000):03d}"

        img_bytes = to_bytes_ros(msg.data)
        img_array = np.frombuffer(img_bytes, dtype=np.uint8)

        image = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
        if image is None:
            self.get_logger().error('Failed to decode image')
            return

        cv2.putText(
            image,
            f"Timestamp: {human_ts}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )

        filename = f'image_{self.frame_id}.jpg'
        path = os.path.join(OUTPUT_DIR, filename)

        if not cv2.imwrite(path, image):
            self.get_logger().error(f'Failed to save image to {path}')
            return

        with open(METADATA_FILE, 'r+', encoding='utf-8') as f:
            data = json.load(f)
            data.append({
                'saved_file_name': filename,
                'timestamp_used': human_ts,
                'timestamp_epoch': timestamp
            })
            f.seek(0)
            json.dump(data, f, indent=2)
            f.truncate()

        self.get_logger().info(
            f"Received frame {self.frame_id} | {human_ts} | saved: {path}"
        )
        self.frame_id += 1


def main():
    rclpy.init()
    node = ImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()