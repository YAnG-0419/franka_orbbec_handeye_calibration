#!/usr/bin/env python3
"""Estimate a gravity-aligned world frame from Orbbec IMU and hand-eye poses.

This script is intentionally standalone and read-only with respect to TF.  It
collects accelerometer samples, computes an orthonormal world frame, and prints
the resulting transforms as JSON.  It does not broadcast any transform.
"""

import argparse
import json
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformException, TransformListener


# Existing eye-to-hand calibration results: base -> camera_color_optical_frame.
LEFT_TRANSLATION = np.array([-0.0434372, -0.39801, 0.292056])
LEFT_QUATERNION = np.array([-0.38606, 0.705046, -0.16033, 0.572854])
RIGHT_TRANSLATION = np.array([-0.024517, 0.421091, 0.256964])
RIGHT_QUATERNION = np.array([0.682676, -0.383116, 0.604902, -0.145841])


def quaternion_matrix(quaternion):
    """Return a 3x3 rotation matrix for an xyzw quaternion."""
    x, y, z, w = np.asarray(quaternion, dtype=float)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError("zero-length quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def matrix_quaternion(matrix):
    """Return a normalized xyzw quaternion for a 3x3 rotation matrix."""
    m = np.asarray(matrix, dtype=float)
    trace = np.trace(m)
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        q = np.array(
            [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s,
             (m[1, 0] - m[0, 1]) / s, 0.25 * s]
        )
    else:
        i = int(np.argmax(np.diag(m)))
        if i == 0:
            s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
            q = np.array(
                [0.25 * s, (m[0, 1] + m[1, 0]) / s,
                 (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s]
            )
        elif i == 1:
            s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
            q = np.array(
                [(m[0, 1] + m[1, 0]) / s, 0.25 * s,
                 (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s]
            )
        else:
            s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
            q = np.array(
                [(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s,
                 0.25 * s, (m[1, 0] - m[0, 1]) / s]
            )
    q /= np.linalg.norm(q)
    return q if q[3] >= 0 else -q


def normalize(vector):
    norm = np.linalg.norm(vector)
    if norm < 1e-9:
        raise ValueError("cannot normalize a near-zero vector")
    return vector / norm


def transform_dict(rotation, translation):
    return {
        "translation_xyz_m": np.asarray(translation).round(9).tolist(),
        "quaternion_xyzw": matrix_quaternion(rotation).round(9).tolist(),
    }


class ImuCollector(Node):
    def __init__(self, topic):
        super().__init__("estimate_world_frame")
        self.samples = []
        self.frame_id = None
        self.create_subscription(Imu, topic, self._callback, qos_profile_sensor_data)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _callback(self, message):
        frame_id = message.header.frame_id
        if not frame_id:
            return
        if self.frame_id is None:
            self.frame_id = frame_id
        if frame_id != self.frame_id:
            self.get_logger().warning(
                f"Ignoring sample whose frame changed from {self.frame_id} to {frame_id}"
            )
            return
        a = message.linear_acceleration
        sample = np.array([a.x, a.y, a.z], dtype=float)
        if np.all(np.isfinite(sample)):
            self.samples.append(sample)


def collect_samples(node, duration):
    deadline = time.monotonic() + duration
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if len(node.samples) < 20:
        raise RuntimeError(
            f"only received {len(node.samples)} valid samples; check the IMU topic"
        )

    samples = np.asarray(node.samples)
    magnitudes = np.linalg.norm(samples, axis=1)
    median = np.median(magnitudes)
    mad = np.median(np.abs(magnitudes - median))
    tolerance = max(0.15, 4.0 * 1.4826 * mad)
    keep = np.abs(magnitudes - median) <= tolerance
    filtered = samples[keep]
    if len(filtered) < 20:
        raise RuntimeError("too many acceleration samples were rejected")
    return filtered, median, tolerance


def lookup_rotation(node, target_frame, source_frame):
    deadline = time.monotonic() + 3.0
    last_error = None
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        try:
            transform = node.tf_buffer.lookup_transform(
                target_frame, source_frame, Time(), timeout=Duration(seconds=0.2)
            )
            q = transform.transform.rotation
            return quaternion_matrix([q.x, q.y, q.z, q.w])
        except TransformException as error:
            last_error = error
    raise RuntimeError(
        f"TF unavailable: {source_frame} -> {target_frame}: {last_error}"
    )


def estimate(node, filtered, camera_frame):
    mean_accel_imu = np.mean(filtered, axis=0)
    rotation_camera_imu = lookup_rotation(node, camera_frame, node.frame_id)
    # REP-145: a stationary accelerometer measures +g in the upward direction.
    z_camera = normalize(rotation_camera_imu @ mean_accel_imu)

    rotation_left_camera = quaternion_matrix(LEFT_QUATERNION)
    rotation_right_camera = quaternion_matrix(RIGHT_QUATERNION)
    rotation_camera_left = rotation_left_camera.T
    rotation_camera_right = rotation_right_camera.T

    left_x_camera = rotation_camera_left @ np.array([1.0, 0.0, 0.0])
    right_x_camera = rotation_camera_right @ np.array([1.0, 0.0, 0.0])
    if np.dot(left_x_camera, right_x_camera) < 0:
        right_x_camera = -right_x_camera
    mean_x_camera = normalize(left_x_camera + right_x_camera)

    projected_x = mean_x_camera - np.dot(mean_x_camera, z_camera) * z_camera
    if np.linalg.norm(projected_x) < 0.1:
        raise RuntimeError("arm X direction is too close to vertical to define world X")
    x_camera = normalize(projected_x)
    y_camera = normalize(np.cross(z_camera, x_camera))
    x_camera = normalize(np.cross(y_camera, z_camera))

    # Columns are world axes expressed in the camera frame.
    rotation_camera_world = np.column_stack((x_camera, y_camera, z_camera))
    rotation_world_camera = rotation_camera_world.T

    left_origin_camera = -rotation_camera_left @ LEFT_TRANSLATION
    right_origin_camera = -rotation_camera_right @ RIGHT_TRANSLATION
    world_origin_camera = 0.5 * (left_origin_camera + right_origin_camera)

    translation_world_camera = -rotation_world_camera @ world_origin_camera
    rotation_world_left = rotation_world_camera @ rotation_camera_left
    rotation_world_right = rotation_world_camera @ rotation_camera_right
    translation_world_left = rotation_world_camera @ (
        left_origin_camera - world_origin_camera
    )
    translation_world_right = rotation_world_camera @ (
        right_origin_camera - world_origin_camera
    )

    x_angle = math.degrees(
        math.acos(np.clip(np.dot(left_x_camera, right_x_camera), -1.0, 1.0))
    )
    return {
        "frames": {
            "world_origin": "midpoint of left and right base origins",
            "world_z": "opposite physical gravity, estimated from stationary accelerometer",
            "world_x": "mean arm X direction projected perpendicular to world Z",
            "camera_frame": camera_frame,
            "imu_frame": node.frame_id,
        },
        "sampling": {
            "received": len(node.samples),
            "accepted": len(filtered),
            "mean_acceleration_imu_m_s2": mean_accel_imu.round(9).tolist(),
            "mean_acceleration_norm_m_s2": round(float(np.linalg.norm(mean_accel_imu)), 9),
        },
        "diagnostics": {
            "arm_x_axis_angle_deg": round(x_angle, 6),
            "world_axes_expressed_in_camera": {
                "x": x_camera.round(9).tolist(),
                "y": y_camera.round(9).tolist(),
                "z": z_camera.round(9).tolist(),
            },
        },
        "transforms": {
            f"world_to_{camera_frame}": transform_dict(
                rotation_world_camera, translation_world_camera
            ),
            "world_to_left_base": transform_dict(
                rotation_world_left, translation_world_left
            ),
            "world_to_right_base": transform_dict(
                rotation_world_right, translation_world_right
            ),
        },
    }


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--topic", default="/camera/accel/sample")
    parser.add_argument("--camera-frame", default="camera_color_optical_frame")
    parser.add_argument("--output", help="also save the JSON result to this path")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.duration < 1.0:
        print("--duration must be at least 1 second", file=sys.stderr)
        return 2

    rclpy.init()
    node = ImuCollector(args.topic)
    try:
        print(
            f"Keep the camera still; collecting {args.duration:.1f} s from "
            f"{args.topic} ...",
            file=sys.stderr,
        )
        filtered, median, tolerance = collect_samples(node, args.duration)
        result = estimate(node, filtered, args.camera_frame)
        result["sampling"]["norm_filter_median_m_s2"] = round(float(median), 9)
        result["sampling"]["norm_filter_tolerance_m_s2"] = round(
            float(tolerance), 9
        )
        output = json.dumps(result, indent=2, ensure_ascii=False)
        print(output)
        if args.output:
            with open(args.output, "w", encoding="utf-8") as output_file:
                output_file.write(output + "\n")
    except (RuntimeError, ValueError) as error:
        node.get_logger().error(str(error))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
