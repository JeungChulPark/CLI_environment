#!/usr/bin/env python3

import argparse
import math
import os
import time
from pathlib import Path

import rclpy
from nav_msgs.srv import GetMap
from rclpy.node import Node
from std_srvs.srv import Empty


SERVICE_DISCOVERY_TIMEOUT = 30.0
SERVICE_CALL_TIMEOUT = 30.0
GET_MAP_TIMEOUT = 90.0


def service_type_name(service_type) -> str:
    package = service_type.__module__.split(".", 1)[0]
    return f"{package}/srv/{service_type.__name__}"


def resolve_service(
    node: Node,
    requested_name: str,
    service_type,
    timeout_sec: float = SERVICE_DISCOVERY_TIMEOUT,
) -> str:
    expected_type = service_type_name(service_type)
    basename = requested_name.rsplit("/", 1)[-1]
    deadline = time.monotonic() + timeout_sec

    while rclpy.ok():
        services = dict(node.get_service_names_and_types())
        if expected_type in services.get(requested_name, []):
            return requested_name

        matches = sorted(
            name
            for name, types in services.items()
            if name.rsplit("/", 1)[-1] == basename and expected_type in types
        )
        if len(matches) == 1:
            print(
                f"Using discovered service {matches[0]} "
                f"instead of {requested_name}"
            )
            return matches[0]
        if len(matches) > 1:
            raise RuntimeError(
                f"ambiguous {basename} services: {', '.join(matches)}"
            )

        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            break
        rclpy.spin_once(node, timeout_sec=min(0.2, remaining))

    raise RuntimeError(
        f"service unavailable: {requested_name} ({expected_type})"
    )


def find_service(node: Node, requested_name: str, service_type) -> str | None:
    expected_type = service_type_name(service_type)
    basename = requested_name.rsplit("/", 1)[-1]
    services = dict(node.get_service_names_and_types())

    if expected_type in services.get(requested_name, []):
        return requested_name

    matches = sorted(
        name
        for name, types in services.items()
        if name.rsplit("/", 1)[-1] == basename and expected_type in types
    )
    if len(matches) == 1:
        print(
            f"Using discovered service {matches[0]} "
            f"instead of {requested_name}"
        )
        return matches[0]
    if len(matches) > 1:
        print(
            f"WARNING: ambiguous optional {basename} services: "
            f"{', '.join(matches)}"
        )
    return None


def call_service(node: Node, service_type, service_name: str, timeout_sec: float):
    client = node.create_client(service_type, service_name)
    if not client.wait_for_service(timeout_sec=timeout_sec):
        raise RuntimeError(f"service unavailable: {service_name}")

    future = client.call_async(service_type.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if not future.done():
        raise RuntimeError(f"timed out waiting for {service_name}")
    response = future.result()
    if response is None:
        raise RuntimeError(f"{service_name} returned no response")
    return response


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def occupancy_to_pgm(data, width: int, height: int) -> bytes:
    pixels = bytearray(width * height)
    output_index = 0

    # OccupancyGrid starts at the lower-left; PGM rows start at the top-left.
    for y in range(height - 1, -1, -1):
        row_start = y * width
        for x in range(width):
            value = data[row_start + x]
            if value < 0:
                pixel = 205
            elif value >= 65:
                pixel = 0
            elif value <= 25:
                pixel = 254
            else:
                pixel = 205
            pixels[output_index] = pixel
            output_index += 1

    return bytes(pixels)


def save_map(message, output_prefix: Path) -> None:
    info = message.info
    width = int(info.width)
    height = int(info.height)

    if width <= 0 or height <= 0:
        raise RuntimeError(f"empty occupancy grid: {width}x{height}")
    if len(message.data) != width * height:
        raise RuntimeError(
            f"invalid occupancy data: {len(message.data)} cells for {width}x{height}"
        )
    if not math.isfinite(info.resolution) or info.resolution <= 0.0:
        raise RuntimeError(f"invalid map resolution: {info.resolution}")

    output_prefix.parent.mkdir(parents=True, exist_ok=True)
    pgm_path = output_prefix.with_suffix(".pgm")
    yaml_path = output_prefix.with_suffix(".yaml")
    temporary_pgm = output_prefix.parent / f".{output_prefix.name}.pending.pgm"
    temporary_yaml = output_prefix.parent / f".{output_prefix.name}.pending.yaml"

    pixels = occupancy_to_pgm(message.data, width, height)
    with temporary_pgm.open("wb") as stream:
        stream.write(f"P5\n# CREATOR: RTAB-Map GetMap\n{width} {height}\n255\n".encode())
        stream.write(pixels)

    origin = info.origin
    yaw = quaternion_yaw(
        origin.orientation.x,
        origin.orientation.y,
        origin.orientation.z,
        origin.orientation.w,
    )
    yaml_text = (
        f"image: {pgm_path.name}\n"
        "mode: trinary\n"
        f"resolution: {info.resolution:.9g}\n"
        f"origin: [{origin.position.x:.9g}, {origin.position.y:.9g}, {yaw:.9g}]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n"
    )
    temporary_yaml.write_text(yaml_text, encoding="utf-8")

    os.replace(temporary_pgm, pgm_path)
    os.replace(temporary_yaml, yaml_path)
    print(f"Occupancy grid: {width}x{height} at {info.resolution:.3f} m/cell")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--service", required=True)
    parser.add_argument("--pause-service", required=True)
    parser.add_argument("--resume-service", required=True)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    rclpy.init()
    node = Node("save_rtabmap_occupancy_grid")
    paused = False
    try:
        get_map_service = resolve_service(node, args.service, GetMap)
        pause_service = find_service(node, args.pause_service, Empty)
        resume_service = find_service(node, args.resume_service, Empty)

        if pause_service and resume_service:
            print(f"Pausing RTAB-Map through {pause_service}...")
            call_service(node, Empty, pause_service, SERVICE_CALL_TIMEOUT)
            paused = True
        else:
            print(
                "RTAB-Map pause/resume services are not advertised; "
                "saving directly through its serialized get_map callback."
            )

        print(f"Requesting occupancy grid from {get_map_service}...")
        response = call_service(node, GetMap, get_map_service, GET_MAP_TIMEOUT)
        save_map(response.map, args.output)
    except Exception as error:
        raise SystemExit(f"ERROR: {error}") from error
    finally:
        if paused and rclpy.ok():
            try:
                print(f"Resuming RTAB-Map through {resume_service}...")
                call_service(node, Empty, resume_service, SERVICE_CALL_TIMEOUT)
            except Exception as error:
                print(f"WARNING: failed to resume RTAB-Map: {error}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
