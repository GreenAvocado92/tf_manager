#!/usr/bin/env python3
"""Python implementation of pointcloud_tf_transformer.cpp."""

import copy
from math import isfinite
from pathlib import Path
import struct

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


class PointCloudConsumer(Node):
    """Transform PointCloud2 XYZ fields using the configured TF transform."""

    def __init__(self) -> None:
        super().__init__('pointcloud_consumer')
        config_path = self.declare_parameter('pointcloud_config', '').value
        if not config_path:
            raise ValueError("Parameter 'pointcloud_config' must point to a YAML file")
        self._load_configuration(Path(config_path))

        self._buffer = Buffer(cache_time=Duration(seconds=10.0), node=self)
        self._listener = TransformListener(self._buffer, self)
        self._publisher = self.create_publisher(
            PointCloud2, self._output_topic, qos_profile_sensor_data)
        self._subscription = self.create_subscription(
            PointCloud2, self._cloud_topic, self._cloud_callback, qos_profile_sensor_data)

        self.get_logger().info(
            f'Transforming {self._cloud_topic} ({self._input_xyz_unit}) from '
            f'{self._source_frame_id} to {self._target_frame_id} and publishing '
            f'{self._output_topic}')

    def _load_configuration(self, config_path: Path) -> None:
        try:
            with config_path.open(encoding='utf-8') as config_file:
                config = yaml.safe_load(config_file)
        except (OSError, yaml.YAMLError) as error:
            raise ValueError(
                f"Could not read point-cloud configuration '{config_path}': {error}") from error

        if not isinstance(config, dict):
            raise ValueError(f"Point-cloud configuration '{config_path}' must be a YAML mapping")

        required_keys = (
            'input_xyz_unit', 'cloud_topic', 'output_topic', 'source_frame_id',
            'target_frame_id')
        missing_keys = [key for key in required_keys if not isinstance(config.get(key), str)
                        or not config[key]]
        if missing_keys:
            raise ValueError(
                f"Point-cloud configuration '{config_path}' has missing or empty keys: "
                f"{', '.join(missing_keys)}")

        self._input_xyz_unit = config['input_xyz_unit']
        self._cloud_topic = config['cloud_topic']
        self._output_topic = config['output_topic']
        self._source_frame_id = config['source_frame_id']
        self._target_frame_id = config['target_frame_id']
        if self._input_xyz_unit not in ('m', 'mm'):
            raise ValueError("'input_xyz_unit' must be either 'm' or 'mm'")

    @staticmethod
    def _xyz_offsets(cloud: PointCloud2) -> tuple[int, int, int]:
        fields = {field.name: field for field in cloud.fields}
        try:
            x_field, y_field, z_field = (fields['x'], fields['y'], fields['z'])
        except KeyError as error:
            raise ValueError('PointCloud2 must contain float32 x, y and z fields') from error

        xyz_fields = (x_field, y_field, z_field)
        if any(field.datatype != PointField.FLOAT32 or field.count != 1 for field in xyz_fields):
            raise ValueError('PointCloud2 must contain float32 x, y and z fields')
        return x_field.offset, y_field.offset, z_field.offset

    @staticmethod
    def _rotate_point(x: float, y: float, z: float, qx: float, qy: float, qz: float,
                      qw: float) -> tuple[float, float, float]:
        """Apply the same quaternion rotation represented by tf2::Transform's basis."""
        xx, yy, zz = qx * qx, qy * qy, qz * qz
        xy, xz, yz = qx * qy, qx * qz, qy * qz
        wx, wy, wz = qw * qx, qw * qy, qw * qz
        return (
            (1.0 - 2.0 * (yy + zz)) * x + 2.0 * (xy - wz) * y + 2.0 * (xz + wy) * z,
            2.0 * (xy + wz) * x + (1.0 - 2.0 * (xx + zz)) * y + 2.0 * (yz - wx) * z,
            2.0 * (xz - wy) * x + 2.0 * (yz + wx) * y + (1.0 - 2.0 * (xx + yy)) * z,
        )

    def _cloud_callback(self, input_cloud: PointCloud2) -> None:
        try:
            transform = self._buffer.lookup_transform(
                self._target_frame_id, self._source_frame_id,
                Time.from_msg(input_cloud.header.stamp), timeout=Duration(seconds=0.1))
        except TransformException as error:
            self.get_logger().warning(
                f'TF unavailable; publishing the input cloud unchanged: {error}',
                throttle_duration_sec=5.0)
            self._publisher.publish(input_cloud)
            return

        output_cloud = copy.deepcopy(input_cloud)
        output_cloud.header.frame_id = self._target_frame_id
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        self.get_logger().info(
            f'T_{self._target_frame_id}_{self._source_frame_id}: '
            f'translation=({translation.x:.6f}, {translation.y:.6f}, {translation.z:.6f}), '
            f'quaternion=({rotation.w:.6f}, {rotation.x:.6f}, {rotation.y:.6f}, '
            f'{rotation.z:.6f})', throttle_duration_sec=5.0)

        try:
            x_offset, y_offset, z_offset = self._xyz_offsets(output_cloud)
            if output_cloud.row_step < output_cloud.width * output_cloud.point_step:
                raise ValueError('PointCloud2 row_step is smaller than width * point_step')
            output_data = bytearray(output_cloud.data)
            if len(output_data) < output_cloud.row_step * output_cloud.height:
                raise ValueError('PointCloud2 data is smaller than row_step * height')

            byte_order = '>' if output_cloud.is_bigendian else '<'
            input_scale_to_mm = 1000.0 if self._input_xyz_unit == 'm' else 1.0
            translation_mm = (
                translation.x * 1000.0, translation.y * 1000.0, translation.z * 1000.0)

            for row in range(output_cloud.height):
                row_start = row * output_cloud.row_step
                for column in range(output_cloud.width):
                    point_start = row_start + column * output_cloud.point_step
                    x = struct.unpack_from(byte_order + 'f', output_data, point_start + x_offset)[0]
                    y = struct.unpack_from(byte_order + 'f', output_data, point_start + y_offset)[0]
                    z = struct.unpack_from(byte_order + 'f', output_data, point_start + z_offset)[0]
                    if not (isfinite(x) and isfinite(y) and isfinite(z)):
                        continue

                    x, y, z = self._rotate_point(
                        x * input_scale_to_mm, y * input_scale_to_mm, z * input_scale_to_mm,
                        rotation.x, rotation.y, rotation.z, rotation.w)
                    struct.pack_into(byte_order + 'f', output_data, point_start + x_offset,
                                     x + translation_mm[0])
                    struct.pack_into(byte_order + 'f', output_data, point_start + y_offset,
                                     y + translation_mm[1])
                    struct.pack_into(byte_order + 'f', output_data, point_start + z_offset,
                                     z + translation_mm[2])
            output_cloud.data = output_data
        except (IndexError, struct.error, ValueError) as error:
            self.get_logger().error(
                f'PointCloud2 must contain float32 x, y and z fields: {error}',
                throttle_duration_sec=5.0)
            return

        self._publisher.publish(output_cloud)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = PointCloudConsumer()
        rclpy.spin(node)
    except (RuntimeError, ValueError) as error:
        rclpy.logging.get_logger('pointcloud_consumer').fatal(str(error))
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
