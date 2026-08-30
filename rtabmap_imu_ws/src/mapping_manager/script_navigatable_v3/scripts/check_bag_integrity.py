#!/usr/bin/env python3

"""Check whether camera images survived rosbag transport and storage."""

import argparse
import os
from collections import defaultdict
from pathlib import Path
import sys

import yaml


DEFAULT_STREAMS = (
    (
        "color",
        (
            "/camera/camera/color/image_raw/compressed",
            "/camera/camera/color/image_raw",
        ),
        "/camera/camera/color/camera_info",
    ),
    (
        "aligned_depth_to_color",
        (
            "/camera/camera/aligned_depth_to_color/image_raw/compressedDepth",
            "/camera/camera/aligned_depth_to_color/image_raw",
        ),
        "/camera/camera/color/camera_info",
    ),
)


def coverage_value(raw_value):
    try:
        value = float(raw_value)
    except (TypeError, ValueError) as error:
        raise argparse.ArgumentTypeError(
            f"expected a number between 0 and 1, got {raw_value!r}"
        ) from error
    if not 0.0 <= value <= 1.0:
        raise argparse.ArgumentTypeError(
            f"expected a number between 0 and 1, got {raw_value!r}"
        )
    return value


def positive_rate_value(raw_value):
    try:
        value = float(raw_value)
    except (TypeError, ValueError) as error:
        raise argparse.ArgumentTypeError(
            f"expected a positive rate in Hz, got {raw_value!r}"
        ) from error
    if value <= 0.0:
        raise argparse.ArgumentTypeError(
            f"expected a positive rate in Hz, got {raw_value!r}"
        )
    return value


def load_bag_information(bag_path):
    path = Path(bag_path).expanduser()
    metadata_path = path / "metadata.yaml" if path.is_dir() else path
    if not metadata_path.is_file():
        raise ValueError(f"metadata.yaml not found: {metadata_path}")

    try:
        document = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
        information = document["rosbag2_bagfile_information"]
        duration_ns = int(information["duration"]["nanoseconds"])
        topic_entries = information["topics_with_message_count"]
    except (KeyError, TypeError, ValueError, yaml.YAMLError) as error:
        raise ValueError(f"invalid rosbag metadata: {metadata_path}: {error}") from error

    if duration_ns <= 0:
        raise ValueError(
            f"bag duration must be positive to calculate effective rates: {duration_ns} ns"
        )

    topic_counts = defaultdict(int)
    try:
        for entry in topic_entries:
            topic_name = entry["topic_metadata"]["name"]
            topic_counts[topic_name] += int(entry["message_count"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(
            f"invalid topic count in rosbag metadata: {metadata_path}: {error}"
        ) from error

    relative_paths = information.get("relative_file_paths") or []
    split_count = len(relative_paths)
    if split_count == 0:
        split_count = len(information.get("files") or [])

    return metadata_path, duration_ns / 1_000_000_000.0, split_count, topic_counts


def check_streams(
    duration_sec,
    topic_counts,
    streams,
    minimum_coverage,
    minimum_rate_hz,
):
    results = []
    for name, image_topics, camera_info_topic in streams:
        image_topic = next(
            (topic for topic in image_topics if topic_counts.get(topic, 0) > 0),
            image_topics[0],
        )
        image_count = topic_counts.get(image_topic, 0)
        reference_count = topic_counts.get(camera_info_topic, 0)
        coverage = image_count / reference_count if reference_count > 0 else 0.0
        results.append(
            {
                "name": name,
                "image_topic": image_topic,
                "camera_info_topic": camera_info_topic,
                "image_count": image_count,
                "reference_count": reference_count,
                "coverage": coverage,
                "missing": max(reference_count - image_count, 0),
                "image_hz": image_count / duration_sec,
                "camera_info_hz": reference_count / duration_sec,
                "passed": (
                    image_count > 0
                    and reference_count > 0
                    and coverage >= minimum_coverage
                    and image_count / duration_sec >= minimum_rate_hz
                    and reference_count / duration_sec >= minimum_rate_hz
                ),
            }
        )
    return results


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            "Compare each recorded Image count with its CameraInfo reference count. "
            "Counts come from the aggregate metadata and therefore include every "
            "split bag file."
        )
    )
    parser.add_argument("bag", help="Rosbag directory or metadata.yaml path.")
    parser.add_argument(
        "--min-coverage",
        type=coverage_value,
        default=os.environ.get("BAG_INTEGRITY_MIN_COVERAGE", "0.99"),
        help=(
            "Minimum image_count/camera_info_count ratio (default: 0.99, "
            "environment: BAG_INTEGRITY_MIN_COVERAGE)."
        ),
    )
    parser.add_argument(
        "--min-rate-hz",
        type=positive_rate_value,
        default=os.environ.get("BAG_INTEGRITY_MIN_RATE_HZ", "27.0"),
        help=(
            "Minimum effective rate for both Image and CameraInfo "
            "(default: 27.0 Hz for a 30 Hz stream, environment: "
            "BAG_INTEGRITY_MIN_RATE_HZ)."
        ),
    )
    parser.add_argument(
        "--stream",
        nargs=3,
        action="append",
        metavar=("NAME", "IMAGE_TOPIC", "CAMERA_INFO_TOPIC"),
        help="Stream comparison. Repeat for multiple streams.",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_arguments(argv)
    streams = (
        tuple(
            (name, (image_topic,), camera_info_topic)
            for name, image_topic, camera_info_topic in args.stream
        )
        if args.stream
        else DEFAULT_STREAMS
    )

    try:
        metadata_path, duration_sec, split_count, topic_counts = load_bag_information(
            args.bag
        )
    except ValueError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2

    print("==========================================")
    print(" Rosbag camera integrity")
    print("==========================================")
    print(f"Metadata: {metadata_path}")
    print(f"Duration: {duration_sec:.3f} s")
    print(f"Split files: {split_count}")
    print(f"Minimum coverage: {args.min_coverage:.4f}")
    print(f"Minimum effective rate: {args.min_rate_hz:.3f} Hz")

    results = check_streams(
        duration_sec,
        topic_counts,
        streams,
        args.min_coverage,
        args.min_rate_hz,
    )
    for result in results:
        status = "PASS" if result["passed"] else "FAIL"
        print(
            f"{status} {result['name']}: "
            f"image={result['image_count']} "
            f"camera_info={result['reference_count']} "
            f"missing={result['missing']} "
            f"coverage={result['coverage']:.4f} "
            f"image_hz={result['image_hz']:.3f} "
            f"camera_info_hz={result['camera_info_hz']:.3f}"
        )
        print(f"  image topic: {result['image_topic']}")
        print(f"  reference:   {result['camera_info_topic']}")

    if all(result["passed"] for result in results):
        print("RESULT: PASS")
        return 0

    print("RESULT: FAIL")
    return 1


if __name__ == "__main__":
    sys.exit(main())
