#!/usr/bin/env python3
"""Inspect PointCloud2 messages in ROS 2 sqlite bags."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def open_reader(bag_path: Path) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    return reader


def stamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def inspect_topic(bag_path: Path, topic: str, limit: int) -> dict[str, Any]:
    reader = open_reader(bag_path)
    type_map = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if topic not in type_map:
        return {"bag": str(bag_path), "topic": topic, "error": "topic not found"}

    msg_type = get_message(type_map[topic])
    samples = []
    point_counts = []
    header_deltas = []
    first_stamp = None
    last_stamp = None
    count = 0

    while reader.has_next():
        topic_name, data, bag_stamp_ns = reader.read_next()
        if topic_name != topic:
            continue
        msg = deserialize_message(data, msg_type)
        count += 1
        h_stamp = stamp_ns(msg.header.stamp)
        if first_stamp is None:
            first_stamp = h_stamp
        last_stamp = h_stamp
        header_deltas.append((bag_stamp_ns - h_stamp) / 1e9)
        points = int(msg.width) * int(msg.height)
        point_counts.append(points)
        if len(samples) < limit:
            samples.append(
                {
                    "bag_stamp_ns": bag_stamp_ns,
                    "header_stamp_ns": h_stamp,
                    "frame_id": msg.header.frame_id,
                    "height": int(msg.height),
                    "width": int(msg.width),
                    "point_count": points,
                    "point_step": int(msg.point_step),
                    "row_step": int(msg.row_step),
                    "is_dense": bool(msg.is_dense),
                    "fields": [
                        {
                            "name": field.name,
                            "offset": int(field.offset),
                            "datatype": int(field.datatype),
                            "count": int(field.count),
                        }
                        for field in msg.fields
                    ],
                }
            )

    point_counts_sorted = sorted(point_counts)
    header_deltas_sorted = sorted(header_deltas)

    def percentile(values: list[float] | list[int], pct: float) -> float | None:
        if not values:
            return None
        idx = round((len(values) - 1) * pct / 100.0)
        return float(values[idx])

    return {
        "bag": str(bag_path),
        "topic": topic,
        "type": type_map[topic],
        "count": count,
        "duration_header_sec": None
        if first_stamp is None or last_stamp is None
        else (last_stamp - first_stamp) / 1e9,
        "rate_header_hz": None
        if first_stamp is None or last_stamp is None or last_stamp == first_stamp
        else (count - 1) / ((last_stamp - first_stamp) / 1e9),
        "point_count": {
            "min": None if not point_counts_sorted else int(point_counts_sorted[0]),
            "p50": percentile(point_counts_sorted, 50),
            "max": None if not point_counts_sorted else int(point_counts_sorted[-1]),
        },
        "bag_minus_header_stamp_sec": {
            "min": percentile(header_deltas_sorted, 0),
            "p50": percentile(header_deltas_sorted, 50),
            "max": percentile(header_deltas_sorted, 100),
        },
        "samples": samples,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", action="append", required=True)
    parser.add_argument("--topic", action="append", required=True)
    parser.add_argument("--limit", type=int, default=2)
    args = parser.parse_args()

    results = []
    for bag in args.bag:
        bag_path = Path(bag).expanduser()
        for topic in args.topic:
            results.append(inspect_topic(bag_path, topic, args.limit))
    print(json.dumps(results, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
