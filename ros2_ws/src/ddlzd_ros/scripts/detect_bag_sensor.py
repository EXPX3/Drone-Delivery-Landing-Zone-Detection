#!/usr/bin/env python3
"""Resolve the LiDAR profile and PointCloud2 topic recorded in a ROS 2 bag."""

import argparse
import json
import re
import sys
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String


SUPPORTED = ("vlp16", "os1_rev7", "os1_rev8")


def _recursive_values(value, key):
    if isinstance(value, dict):
        if key in value:
            yield value[key]
        for child in value.values():
            yield from _recursive_values(child, key)
    elif isinstance(value, list):
        for child in value:
            yield from _recursive_values(child, key)


def _ouster_revision(metadata_documents):
    for document in metadata_documents:
        try:
            parsed = json.loads(document)
        except json.JSONDecodeError:
            continue
        part_numbers = [str(value).upper() for value in _recursive_values(parsed, "prod_pn")]
        if any(value.startswith("OS1-080-") for value in part_numbers):
            return "os1_rev8"
        # Revision is a hardware property. Firmware build_rev and prod_line do
        # not distinguish Rev8 from an older OS1 and must not be guessed.
        if any(
            re.match(r"OS1-07[0-9]-", value) or
            re.fullmatch(r"\d{3}-\d+-0[1-7]", value)
            for value in part_numbers
        ):
            return "os1_rev7"
    return None


def _field_map(message):
    return {field.name: field.datatype for field in message.fields}


def _point_family(topic, fields):
    topic_lower = topic.lower()
    if fields.get("t") == PointField.UINT32:
        return "ouster"
    if fields.get("time") == PointField.FLOAT32 and "ring" in fields:
        return "vlp16"
    if "velodyne" in topic_lower and "ring" in fields:
        return "vlp16"
    if "ouster" in topic_lower and "t" in fields:
        return "ouster"
    return None


def inspect_bag(bag_path, requested_sensor="auto", requested_topic=""):
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, rosbag2_py.ConverterOptions("", ""))
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}

    point_topics = [
        name for name, type_name in topic_types.items()
        if type_name == "sensor_msgs/msg/PointCloud2"
    ]
    if requested_topic:
        if topic_types.get(requested_topic) != "sensor_msgs/msg/PointCloud2":
            raise RuntimeError(
                f"requested point topic {requested_topic!r} is not a PointCloud2 topic in the bag")
        point_topics = [requested_topic]
    if not point_topics:
        raise RuntimeError("bag contains no sensor_msgs/msg/PointCloud2 topic")

    metadata_topics = {
        name for name, type_name in topic_types.items()
        if type_name == "std_msgs/msg/String" and name.lower().endswith("/metadata")
    }
    point_messages = {}
    metadata_documents = []
    while reader.has_next() and (
        len(point_messages) < len(point_topics) or
        (metadata_topics and not metadata_documents)
    ):
        topic, serialized, _ = reader.read_next()
        if topic in point_topics and topic not in point_messages:
            point_messages[topic] = deserialize_message(serialized, PointCloud2)
        elif topic in metadata_topics and not metadata_documents:
            metadata_documents.append(deserialize_message(serialized, String).data)

    families = {}
    for topic in point_topics:
        message = point_messages.get(topic)
        if message is not None:
            families[topic] = _point_family(topic, _field_map(message))

    if requested_sensor == "auto":
        recognized = [(topic, family) for topic, family in families.items() if family]
        if len(recognized) != 1:
            detail = ", ".join(f"{topic}:{family or 'unknown'}" for topic, family in families.items())
            raise RuntimeError(
                "bag sensor is absent or ambiguous; pass --sensor and --point-topic explicitly "
                f"(observed {detail or 'no readable cloud'})")
        point_topic, family = recognized[0]
        if family == "vlp16":
            sensor = "vlp16"
        else:
            sensor = _ouster_revision(metadata_documents)
            if sensor is None:
                raise RuntimeError(
                    "bag contains an Ouster cloud but its hardware revision is not provable from "
                    "recorded metadata; pass --sensor os1_rev7 or --sensor os1_rev8")
    else:
        sensor = requested_sensor
        expected_family = "vlp16" if sensor == "vlp16" else "ouster"
        matching = [topic for topic, family in families.items() if family == expected_family]
        if len(matching) != 1:
            detail = ", ".join(f"{topic}:{family or 'unknown'}" for topic, family in families.items())
            raise RuntimeError(
                f"sensor override {sensor!r} does not identify exactly one compatible cloud "
                f"(observed {detail})")
        point_topic = matching[0]

    return {"sensor_model": sensor, "point_cloud_topic": point_topic}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--sensor", choices=("auto",) + SUPPORTED, default="auto")
    parser.add_argument("--point-topic", default="")
    args = parser.parse_args()
    try:
        result = inspect_bag(args.bag, args.sensor, args.point_topic)
    except Exception as error:  # launch wrapper needs one concise error line
        print(f"bag sensor detection failed: {error}", file=sys.stderr)
        return 2
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
