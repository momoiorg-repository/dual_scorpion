"""Optional ROS2 process: calibrated C270 -> stella_vslam; pose/TF transport.

Import rclpy only inside main. Core Python tooling works without ROS installed.
The wrapper publishes T_S_C in ROS map/camera axes; it has already converted
OpenCV axes on both sides. Never invert or convert it a second time.
"""

import argparse
import json
import socket
import time
from collections import OrderedDict

import numpy as np
from scipy.spatial.transform import Rotation

from .camera import UvcCamera, intrinsics_id, load_intrinsics
from .config import camera_device, load_config
from .geometry import pose


def build_parser():
    parser = argparse.ArgumentParser(description="Head camera + external stella_vslam ROS2 adapter")
    parser.add_argument("--config", default="config/mocopi/tracking.yaml")
    parser.add_argument("--camera", type=camera_device, help="Camera index or device path; overrides YAML")
    parser.add_argument("--map-id", help="Unique map identity; same value only for the same saved map")
    return parser


def main():
    args = build_parser().parse_args()
    cfg = load_config(args.config, camera=args.camera)

    import cv2
    import rclpy
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_ros import TransformBroadcaster

    intrinsics, k, d = load_intrinsics(
        cfg["slam"]["intrinsics"], [cfg["camera"]["width"], cfg["camera"]["height"]]
    )
    session = args.map_id or cfg["slam"]["map_id"]
    identity = intrinsics_id(cfg["slam"]["intrinsics"])
    rclpy.init()
    node = Node("mocopi_head_camera")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    debug = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    camera = None
    try:
        debug.bind(("127.0.0.1", cfg["tracking"]["debug_port"]))
        debug.setblocking(False)
        camera = UvcCamera(cfg["camera"])
        node.get_logger().info(f"Head camera: {cfg['camera']['device']}")
        image_pub = node.create_publisher(Image, "/camera/image_raw", qos_profile_sensor_data)
        info_pub = node.create_publisher(CameraInfo, "/camera/camera_info", qos_profile_sensor_data)
        tf = TransformBroadcaster(node)
        pose_publishers = {}
        acquisition = OrderedDict()
        last_pose = [0.0]
        state = {"frames": 0, "start": time.monotonic()}

        def send(camera_pose, timestamp, valid):
            row = {
                "schema": 1,
                "convention": "T_S_C_ros",
                "timestamp": timestamp,
                "position": camera_pose[:3, 3].tolist(),
                "quaternion": Rotation.from_matrix(camera_pose[:3, :3]).as_quat().tolist(),
                "valid": valid,
                "session": session,
                "intrinsics_id": identity,
            }
            sock.sendto(json.dumps(row).encode(), ("127.0.0.1", cfg["slam"]["pose_port"]))

        def odometry(msg):
            key = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            stamp = acquisition.get(key)
            # Do not stamp delayed SLAM processing as if it were a fresh image.
            if stamp is None or time.monotonic() - stamp > cfg["tracking"]["timeout_s"]:
                return
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            try:
                camera_pose = pose([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
                send(camera_pose, stamp, True)
                last_pose[0] = time.monotonic()
            except ValueError as exc:
                node.get_logger().warning(str(exc))

        node.create_subscription(Odometry, cfg["slam"]["topic"], odometry, 10)

        def publish_debug():
            for _ in range(30):
                try:
                    raw, _ = debug.recvfrom(65535)
                except BlockingIOError:
                    return
                row = json.loads(raw)
                frames = row.get("frames", {})
                sample_age = max(0.0, time.monotonic() - float(row["timestamp"]))
                stamp = (node.get_clock().now() - Duration(seconds=sample_age)).to_msg()
                for name, item in frames.items():
                    if name not in pose_publishers:
                        pose_publishers[name] = node.create_publisher(PoseStamped, "/mocopi/" + name, 10)
                    matrix = np.asarray(item["pose"])
                    xyz, quat = matrix[:3, 3], Rotation.from_matrix(matrix[:3, :3]).as_quat()
                    msg = PoseStamped()
                    msg.header.stamp, msg.header.frame_id = stamp, item["parent"]
                    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, xyz)
                    (
                        msg.pose.orientation.x,
                        msg.pose.orientation.y,
                        msg.pose.orientation.z,
                        msg.pose.orientation.w,
                    ) = map(float, quat)
                    pose_publishers[name].publish(msg)
                    transform = TransformStamped()
                    transform.header = msg.header
                    transform.child_frame_id = name
                    (
                        transform.transform.translation.x,
                        transform.transform.translation.y,
                        transform.transform.translation.z,
                    ) = map(float, xyz)
                    transform.transform.rotation = msg.pose.orientation
                    tf.sendTransform(transform)

        def capture():
            try:
                stamp, image = camera.read()
                if [image.shape[1], image.shape[0]] != intrinsics["resolution"]:
                    raise ValueError("Camera resolution differs from intrinsic calibration")
                image = cv2.undistort(image, k, d)
                ros_stamp = node.get_clock().now().to_msg()
                key = ros_stamp.sec * 10**9 + ros_stamp.nanosec
                acquisition[key] = stamp
                while len(acquisition) > 400:
                    acquisition.popitem(last=False)
                msg = Image()
                msg.header.stamp, msg.header.frame_id = ros_stamp, "head_camera_optical"
                msg.height, msg.width, msg.encoding = image.shape[0], image.shape[1], "bgr8"
                msg.step, msg.data = image.shape[1] * 3, image.tobytes()
                image_pub.publish(msg)
                info = CameraInfo()
                info.header, info.height, info.width = msg.header, msg.height, msg.width
                info.distortion_model, info.d, info.k = "plumb_bob", [0.0] * 5, k.ravel().tolist()
                info.r = np.eye(3).ravel().tolist()
                info.p = np.c_[k, np.zeros(3)].ravel().tolist()
                info_pub.publish(info)
                state["frames"] += 1
                publish_debug()
            except Exception as exc:
                send(np.eye(4), time.monotonic(), False)
                node.get_logger().error(f"Camera/adapter failure: {exc}")
                # Exit; core holds on invalid packet/timeout.
                raise

        def status():
            age = time.monotonic() - last_pose[0]
            node.get_logger().info(
                f"camera FPS={state['frames'] / max(time.monotonic() - state['start'], 0.01):.1f} "
                f"SLAM={'OK' if age < cfg['tracking']['timeout_s'] else 'LOST/initializing'}"
            )

        node.create_timer(1 / cfg["camera"]["fps"], capture)
        node.create_timer(1.0, status)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if camera:
            camera.close()
        sock.close()
        debug.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
