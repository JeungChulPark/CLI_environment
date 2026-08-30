#!/usr/bin/env python3

from __future__ import annotations

import bisect
import json
import math
import statistics
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import rclpy
import yaml
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, LaserScan, PointCloud2, TimeReference

try:
    from realsense2_camera_msgs.msg import Metadata
except Exception:  # pragma: no cover - dependency preflight reports this
    Metadata = None

try:
    from velodyne_msgs.msg import VelodyneScan
except Exception:  # pragma: no cover
    VelodyneScan = None


ROOT = Path(__file__).resolve().parents[1]
POLICY_PATH = ROOT / "config" / "timestamp_sync.yaml"
NSEC_PER_SEC = 1_000_000_000


@dataclass(frozen=True)
class Sample:
    stamp_ns: int
    arrival_ros_ns: int
    arrival_steady_ns: int
    frame_id: str = ""


@dataclass(frozen=True)
class MetadataSample:
    header: Sample
    clock_domain: str
    frame_timestamp_ms: Optional[float]
    hardware_timestamp: Optional[float]
    sensor_timestamp: Optional[float]
    frame_number: Optional[int]


class StreamBuffer:
    def __init__(self) -> None:
        self.samples: List[Sample] = []

    def append(self, sample: Sample) -> None:
        self.samples.append(sample)

    def between(self, start_ns: int, end_ns: int) -> List[Sample]:
        return [
            sample
            for sample in self.samples
            if start_ns <= sample.arrival_steady_ns <= end_ns
        ]


def time_to_ns(stamp: Any) -> int:
    return int(stamp.sec) * NSEC_PER_SEC + int(stamp.nanosec)


def percentile(values: Sequence[float], fraction: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return float(ordered[0])
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return float(ordered[lower])
    weight = position - lower
    return float(
        ordered[lower] * (1.0 - weight) + ordered[upper] * weight
    )


def finite_round(
    value: Optional[float], digits: int = 3
) -> Optional[float]:
    if value is None or not math.isfinite(value):
        return None
    return round(float(value), digits)


def linear_drift_ppm(samples: Sequence[Sample]) -> Optional[float]:
    if len(samples) < 3:
        return None
    first = samples[0]
    x = [
        (sample.arrival_steady_ns - first.arrival_steady_ns) / NSEC_PER_SEC
        for sample in samples
    ]
    y = [
        (sample.stamp_ns - first.stamp_ns) / NSEC_PER_SEC
        for sample in samples
    ]
    x_mean = statistics.fmean(x)
    y_mean = statistics.fmean(y)
    denominator = sum((value - x_mean) ** 2 for value in x)
    if denominator <= 0.0:
        return None
    slope = sum(
        (x_value - x_mean) * (y_value - y_mean)
        for x_value, y_value in zip(x, y)
    ) / denominator
    return (slope - 1.0) * 1_000_000.0


def nearest_sample(
    ordered_samples: Sequence[Sample], stamp_ns: int
) -> Optional[Sample]:
    if not ordered_samples:
        return None
    stamps = [sample.stamp_ns for sample in ordered_samples]
    index = bisect.bisect_left(stamps, stamp_ns)
    candidates: List[Sample] = []
    if index < len(ordered_samples):
        candidates.append(ordered_samples[index])
    if index > 0:
        candidates.append(ordered_samples[index - 1])
    return min(
        candidates, key=lambda sample: abs(sample.stamp_ns - stamp_ns)
    )


def residuals_ms(
    anchors: Sequence[Sample], targets: Sequence[Sample]
) -> List[float]:
    ordered_targets = sorted(targets, key=lambda sample: sample.stamp_ns)
    residuals: List[float] = []
    for anchor in anchors:
        nearest = nearest_sample(ordered_targets, anchor.stamp_ns)
        if nearest is not None:
            residuals.append(
                (nearest.stamp_ns - anchor.stamp_ns) / 1_000_000.0
            )
    return residuals


def residual_summary(values: Sequence[float]) -> Dict[str, Any]:
    absolute = [abs(value) for value in values]
    return {
        "count": len(values),
        "signed_median_ms": finite_round(
            statistics.median(values) if values else None
        ),
        "abs_p95_ms": finite_round(percentile(absolute, 0.95)),
        "abs_p99_ms": finite_round(percentile(absolute, 0.99)),
        "abs_max_ms": finite_round(max(absolute) if absolute else None),
    }


def parse_number(value: Any) -> Optional[float]:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def parse_integer(value: Any) -> Optional[int]:
    parsed = parse_number(value)
    return int(parsed) if parsed is not None else None


def active_timing_expectations(
    policy: Dict[str, Any], profile_name: Optional[str] = None
) -> Dict[str, Any]:
    selected = profile_name or str(policy["active_profile"])
    profiles = policy.get("profiles", {})
    if selected not in profiles:
        available = ", ".join(sorted(profiles))
        raise ValueError(
            f"unknown timing profile {selected!r}; available: {available}"
        )
    profile = profiles[selected]
    realsense = profile["realsense"]
    xsens = profile["xsens"]
    velodyne = profile["velodyne_vlp16"]
    return {
        "profile": selected,
        "role": str(profile["role"]),
        "camera_fps": int(profile["camera_fps"]),
        "require_chrony_sync": bool(profile["require_chrony_sync"]),
        "realsense_enable_sync": bool(realsense["enable_sync"]),
        "realsense_inter_cam_sync_mode": int(
            realsense["inter_cam_sync_mode"]
        ),
        "realsense_allowed_inter_cam_sync_modes": [
            int(value)
            for value in realsense.get(
                "allowed_inter_cam_sync_modes",
                [realsense["inter_cam_sync_mode"]],
            )
        ],
        "realsense_global_time_enabled": bool(
            realsense["global_time_enabled"]
        ),
        "xsens_time_option": int(xsens["time_option"]),
        "velodyne_gps_time": bool(velodyne["gps_time"]),
        "velodyne_timestamp_first_packet": bool(
            velodyne["timestamp_first_packet"]
        ),
        "velodyne_time_offset": float(velodyne["time_offset"]),
    }


def evaluate_timing_parameters(
    camera_parameters: Dict[str, Any],
    xsens_parameters: Dict[str, Any],
    velodyne_parameters: Dict[str, Any],
    expectations: Dict[str, Any],
) -> Tuple[Dict[str, bool], List[str]]:
    expected_global_time = expectations[
        "realsense_global_time_enabled"
    ]
    actual_camera_mode = parse_integer(
        camera_parameters.get("depth_module.inter_cam_sync_mode")
    )
    camera_configured = (
        camera_parameters.get("enable_sync")
        is expectations["realsense_enable_sync"]
        and actual_camera_mode
        in expectations["realsense_allowed_inter_cam_sync_modes"]
        and camera_parameters.get("depth_module.global_time_enabled")
        is expected_global_time
        and camera_parameters.get("rgb_camera.global_time_enabled")
        is expected_global_time
    )
    xsens_configured = (
        parse_integer(xsens_parameters.get("time_option"))
        == expectations["xsens_time_option"]
    )
    velodyne_time_offset = parse_number(
        velodyne_parameters.get("time_offset")
    )
    velodyne_configured = (
        velodyne_parameters.get("gps_time")
        is expectations["velodyne_gps_time"]
        and velodyne_parameters.get("timestamp_first_packet")
        is expectations["velodyne_timestamp_first_packet"]
        and velodyne_time_offset is not None
        and math.isclose(
            velodyne_time_offset,
            expectations["velodyne_time_offset"],
            abs_tol=1e-9,
        )
    )

    profile = expectations["profile"]
    failures: List[str] = []
    if not camera_configured:
        allowed_modes = ", ".join(
            str(value)
            for value in expectations[
                "realsense_allowed_inter_cam_sync_modes"
            ]
        )
        failures.append(
            f"D455f parameters do not match {profile}: expected "
            f"mode in [{allowed_modes}] and "
            f"global_time={str(expected_global_time).lower()}"
        )
    if not xsens_configured:
        failures.append(
            f"Xsens parameters do not match {profile}: expected "
            f"time_option={expectations['xsens_time_option']}"
        )
    if not velodyne_configured:
        failures.append(
            f"VLP-16 parameters do not match {profile}: expected "
            "host time, timestamp_first_packet="
            f"{str(expectations['velodyne_timestamp_first_packet']).lower()}, "
            f"time_offset={expectations['velodyne_time_offset']:g}"
        )
    return {
        "camera": camera_configured,
        "xsens": xsens_configured,
        "velodyne": velodyne_configured,
    }, failures


def metadata_value(data: Dict[str, Any], *names: str) -> Any:
    for name in names:
        if name in data:
            return data[name]
    return None


def chrony_tracking_report() -> Dict[str, Any]:
    report: Dict[str, Any] = {
        "available": False,
        "synchronized": False,
        "reference_id": None,
        "reference_name": None,
        "stratum": None,
        "system_time_offset_ms": None,
        "last_offset_ms": None,
        "rms_offset_ms": None,
        "leap_status": None,
        "selected_source": None,
    }
    try:
        result = subprocess.run(
            ["chronyc", "-c", "tracking"],
            check=False,
            capture_output=True,
            text=True,
            timeout=2.0,
        )
    except (FileNotFoundError, OSError, subprocess.TimeoutExpired):
        return report
    if result.returncode != 0:
        report["error"] = result.stderr.strip() or result.stdout.strip()
        return report

    fields = result.stdout.strip().split(",")
    if len(fields) < 13:
        report["error"] = f"unexpected chronyc output ({len(fields)} fields)"
        return report
    has_reference_name = len(fields) >= 14
    stratum_index = 2 if has_reference_name else 1
    system_time_index = 4 if has_reference_name else 3
    last_offset_index = 5 if has_reference_name else 4
    rms_offset_index = 6 if has_reference_name else 5
    leap_status_index = 13 if has_reference_name else 12
    try:
        report.update(
            {
                "available": True,
                "reference_id": fields[0],
                "reference_name": (
                    fields[1] if has_reference_name else None
                ),
                "stratum": int(fields[stratum_index]),
                "system_time_offset_ms": finite_round(
                    float(fields[system_time_index]) * 1000.0, 6
                ),
                "last_offset_ms": finite_round(
                    float(fields[last_offset_index]) * 1000.0, 6
                ),
                "rms_offset_ms": finite_round(
                    float(fields[rms_offset_index]) * 1000.0, 6
                ),
                "leap_status": fields[leap_status_index],
            }
        )
    except (TypeError, ValueError):
        report["error"] = "could not parse chronyc numeric fields"
        return report

    report["synchronized"] = (
        0 < report["stratum"] <= 15
        and report["leap_status"].lower() == "normal"
        and abs(report["system_time_offset_ms"]) <= 5.0
        and abs(report["last_offset_ms"]) <= 5.0
    )
    try:
        sources = subprocess.run(
            ["chronyc", "-n", "sources"],
            check=False,
            capture_output=True,
            text=True,
            timeout=2.0,
        )
    except (FileNotFoundError, OSError, subprocess.TimeoutExpired):
        return report
    if sources.returncode == 0:
        selected = next(
            (
                line.strip()
                for line in sources.stdout.splitlines()
                if len(line.strip()) >= 2 and line.strip()[1] == "*"
            ),
            "",
        )
        report["selected_source"] = selected or None
    return report


class TripleSyncVerifier(Node):
    def __init__(self) -> None:
        super().__init__("triple_sync_verifier")
        with POLICY_PATH.open() as stream:
            self.policy = yaml.safe_load(stream)
        verification = self.policy["verification"]

        self.declare_parameter("duration_sec", verification["duration_sec"])
        self.declare_parameter("warmup_sec", verification["warmup_sec"])
        self.declare_parameter(
            "timing_profile", str(self.policy["active_profile"])
        )
        self.declare_parameter("output_json", "/tmp/triple_sync_report.json")
        for name in (
            "minimum_coverage",
            "camera_imu_p95_ms",
            "camera_imu_p99_ms",
            "lidar_camera_p95_ms",
            "lidar_camera_p99_ms",
            "triple_span_p95_ms",
            "triple_span_p99_ms",
            "maximum_relative_drift_ppm",
        ):
            self.declare_parameter(name, verification[name])

        self.duration_sec = float(self.get_parameter("duration_sec").value)
        self.warmup_sec = float(self.get_parameter("warmup_sec").value)
        selected = str(self.get_parameter("timing_profile").value)
        self.timing_expectations = active_timing_expectations(
            self.policy, selected
        )
        self.active_profile = selected
        profile = self.policy["profiles"][selected]
        self.expected_camera_hz = float(profile["camera_fps"])
        self.expected_imu_hz = float(
            profile["xsens"].get("output_rate_hz", 150.0)
        )
        self.output_json = str(self.get_parameter("output_json").value)
        self.started_steady_ns = time.monotonic_ns()
        self.ended_steady_ns: Optional[int] = None
        self.cutoff_steady_ns = self.started_steady_ns + int(
            self.warmup_sec * NSEC_PER_SEC
        )

        self.streams = {
            name: StreamBuffer()
            for name in (
                "color",
                "depth",
                "imu",
                "velodyne_packets",
                "velodyne_points",
                "scan",
            )
        }
        self.metadata: Dict[str, List[MetadataSample]] = {
            "color": [],
            "depth": [],
        }
        self.velodyne_scan_durations_ms: List[float] = []
        self.xsens_time_refs: List[Tuple[int, int, int]] = []
        self.xsens_utc_refs: List[Tuple[int, int, int]] = []
        self.metadata_parse_errors = 0

        if Metadata is None:
            self.create_subscription(
                Image,
                "/camera/camera/color/image_raw",
                lambda message: self.record_header("color", message),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Image,
                "/camera/camera/aligned_depth_to_color/image_raw",
                lambda message: self.record_header("depth", message),
                qos_profile_sensor_data,
            )
        else:
            self.create_subscription(
                Metadata,
                "/camera/camera/color/metadata",
                lambda message: self.record_metadata("color", message),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Metadata,
                "/camera/camera/depth/metadata",
                lambda message: self.record_metadata("depth", message),
                qos_profile_sensor_data,
            )
        self.create_subscription(
            Imu,
            "/imu/data",
            lambda message: self.record_header("imu", message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            "/velodyne_points",
            lambda message: self.record_header("velodyne_points", message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            "/scan",
            lambda message: self.record_header("scan", message),
            qos_profile_sensor_data,
        )
        if VelodyneScan is not None:
            self.create_subscription(
                VelodyneScan,
                "/velodyne_packets",
                self.record_velodyne_scan,
                qos_profile_sensor_data,
            )
        self.create_subscription(
            TimeReference,
            "/imu/time_ref",
            lambda message: self.record_time_reference(
                self.xsens_time_refs, message
            ),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TimeReference,
            "/imu/utctime",
            lambda message: self.record_time_reference(
                self.xsens_utc_refs, message
            ),
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"Collecting {selected} timing evidence for "
            f"{self.duration_sec:.1f}s (warm-up {self.warmup_sec:.1f}s)."
        )

    def now_pair(self) -> Tuple[int, int]:
        return self.get_clock().now().nanoseconds, time.monotonic_ns()

    def record_header(self, name: str, message: Any) -> None:
        arrival_ros_ns, arrival_steady_ns = self.now_pair()
        self.streams[name].append(
            Sample(
                stamp_ns=time_to_ns(message.header.stamp),
                arrival_ros_ns=arrival_ros_ns,
                arrival_steady_ns=arrival_steady_ns,
                frame_id=str(message.header.frame_id),
            )
        )

    def record_velodyne_scan(self, message: Any) -> None:
        self.record_header("velodyne_packets", message)
        packet_stamps = [
            time_to_ns(packet.stamp)
            for packet in message.packets
            if time_to_ns(packet.stamp) > 0
        ]
        if len(packet_stamps) >= 2:
            self.velodyne_scan_durations_ms.append(
                (max(packet_stamps) - min(packet_stamps)) / 1_000_000.0
            )

    def record_metadata(self, name: str, message: Any) -> None:
        self.record_header(name, message)
        arrival_ros_ns, arrival_steady_ns = self.now_pair()
        try:
            data = json.loads(message.json_data)
        except (TypeError, ValueError, json.JSONDecodeError):
            self.metadata_parse_errors += 1
            return
        normalized = {
            str(key).strip().lower().replace(" ", "_"): value
            for key, value in data.items()
        }
        header = Sample(
            stamp_ns=time_to_ns(message.header.stamp),
            arrival_ros_ns=arrival_ros_ns,
            arrival_steady_ns=arrival_steady_ns,
            frame_id=str(message.header.frame_id),
        )
        self.metadata[name].append(
            MetadataSample(
                header=header,
                clock_domain=str(
                    metadata_value(normalized, "clock_domain") or ""
                ).lower(),
                frame_timestamp_ms=parse_number(
                    metadata_value(normalized, "frame_timestamp")
                ),
                hardware_timestamp=parse_number(
                    metadata_value(
                        normalized, "hw_timestamp", "hardware_timestamp"
                    )
                ),
                sensor_timestamp=parse_number(
                    metadata_value(normalized, "sensor_timestamp")
                ),
                frame_number=parse_integer(
                    metadata_value(
                        normalized, "frame_number", "frame_counter"
                    )
                ),
            )
        )

    def record_time_reference(
        self, target: List[Tuple[int, int, int]], message: TimeReference
    ) -> None:
        target.append(
            (
                time_to_ns(message.header.stamp),
                time_to_ns(message.time_ref),
                time.monotonic_ns(),
            )
        )

    def collection_done(self) -> bool:
        elapsed = (time.monotonic_ns() - self.started_steady_ns) / NSEC_PER_SEC
        return elapsed >= self.duration_sec

    def filtered_stream(self, name: str) -> List[Sample]:
        end_ns = self.ended_steady_ns or time.monotonic_ns()
        return self.streams[name].between(self.cutoff_steady_ns, end_ns)

    def fetch_remote_parameters(
        self, remote_node: str, names: Sequence[str]
    ) -> Dict[str, Any]:
        client = AsyncParameterClient(self, remote_node)
        if not client.wait_for_services(timeout_sec=1.0):
            return {}
        future = client.get_parameters(list(names))
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done() or future.result() is None:
            return {}
        return {
            name: parameter_value_to_python(value)
            for name, value in zip(names, future.result().values)
        }

    def stream_report(
        self,
        name: str,
        expected_rate: Tuple[float, float],
        maximum_age_ms: float,
        maximum_gap_ms: float,
    ) -> Tuple[Dict[str, Any], List[str]]:
        samples = self.filtered_stream(name)
        failures: List[str] = []
        if len(samples) < 2:
            return {"count": len(samples), "rate_hz": None}, [
                f"{name}: missing or insufficient samples"
            ]
        ordered = sorted(samples, key=lambda sample: sample.arrival_steady_ns)
        elapsed = (
            ordered[-1].arrival_steady_ns - ordered[0].arrival_steady_ns
        ) / NSEC_PER_SEC
        rate = (len(ordered) - 1) / elapsed if elapsed > 0.0 else 0.0
        pairs = list(zip(ordered, ordered[1:]))
        regressions = sum(
            second.stamp_ns < first.stamp_ns for first, second in pairs
        )
        duplicates = sum(
            second.stamp_ns == first.stamp_ns for first, second in pairs
        )
        zero_stamps = sum(sample.stamp_ns <= 0 for sample in ordered)
        periods_ms = [
            (second.stamp_ns - first.stamp_ns) / 1_000_000.0
            for first, second in pairs
            if second.stamp_ns > first.stamp_ns
        ]
        ages_ms = [
            (sample.arrival_ros_ns - sample.stamp_ns) / 1_000_000.0
            for sample in ordered
        ]
        drift_ppm = linear_drift_ppm(ordered)
        allowed_duplicates = (
            max(1, int(len(samples) * 0.001))
            if name in ("color", "depth")
            else 0
        )
        if not expected_rate[0] <= rate <= expected_rate[1]:
            failures.append(
                f"{name}: rate {rate:.2f} Hz outside {expected_rate}"
            )
        if regressions:
            failures.append(f"{name}: {regressions} stamp regressions")
        if duplicates > allowed_duplicates:
            failures.append(
                f"{name}: {duplicates} duplicate stamps > "
                f"allowed {allowed_duplicates}"
            )
        if zero_stamps:
            failures.append(f"{name}: {zero_stamps} zero stamps")
        maximum_gap = max(periods_ms) if periods_ms else None
        if maximum_gap is not None and maximum_gap > maximum_gap_ms:
            failures.append(
                f"{name}: maximum gap {maximum_gap:.1f} ms > "
                f"{maximum_gap_ms:.1f} ms"
            )
        age_p99 = percentile(ages_ms, 0.99)
        if age_p99 is not None and age_p99 > maximum_age_ms:
            failures.append(
                f"{name}: p99 callback age {age_p99:.1f} ms > "
                f"{maximum_age_ms:.1f} ms"
            )
        maximum_drift = float(
            self.get_parameter("maximum_relative_drift_ppm").value
        )
        if (
            elapsed >= 60.0
            and drift_ppm is not None
            and abs(drift_ppm) > maximum_drift
        ):
            failures.append(
                f"{name}: relative drift {drift_ppm:.1f} ppm > "
                f"{maximum_drift:.1f} ppm"
            )
        return {
            "count": len(samples),
            "rate_hz": finite_round(rate),
            "observation_span_sec": finite_round(elapsed),
            "period_median_ms": finite_round(
                statistics.median(periods_ms) if periods_ms else None
            ),
            "period_p99_ms": finite_round(percentile(periods_ms, 0.99)),
            "maximum_gap_ms": finite_round(maximum_gap),
            "age_median_ms": finite_round(statistics.median(ages_ms)),
            "age_p99_ms": finite_round(age_p99),
            "relative_drift_ppm": finite_round(drift_ppm),
            "regressions": regressions,
            "duplicates": duplicates,
            "allowed_duplicates": allowed_duplicates,
            "zero_stamps": zero_stamps,
            "frames": sorted({sample.frame_id for sample in samples}),
        }, failures

    def choose_lidar_stream(self) -> Tuple[str, List[Sample]]:
        for name in ("velodyne_packets", "velodyne_points", "scan"):
            samples = self.filtered_stream(name)
            if len(samples) >= 2:
                return name, samples
        return "velodyne_packets", []

    def metadata_report(
        self, name: str
    ) -> Tuple[Dict[str, Any], bool]:
        end_ns = self.ended_steady_ns or time.monotonic_ns()
        records = [
            record
            for record in self.metadata[name]
            if self.cutoff_steady_ns
            <= record.header.arrival_steady_ns
            <= end_ns
        ]
        domains = sorted(
            {record.clock_domain for record in records if record.clock_domain}
        )
        device_stamps = [
            next(
                (
                    value
                    for value in (
                        record.sensor_timestamp,
                        record.hardware_timestamp,
                        record.frame_timestamp_ms,
                    )
                    if value is not None
                ),
                None,
            )
            for record in records
        ]
        device_stamps = [
            value for value in device_stamps if value is not None
        ]
        frame_numbers = [
            record.frame_number
            for record in records
            if record.frame_number is not None
        ]
        timestamp_regressions = sum(
            second < first
            for first, second in zip(device_stamps, device_stamps[1:])
        )
        timestamp_duplicates = sum(
            second == first
            for first, second in zip(device_stamps, device_stamps[1:])
        )
        counter_regressions = sum(
            second < first
            for first, second in zip(frame_numbers, frame_numbers[1:])
        )
        counter_duplicates = sum(
            second == first
            for first, second in zip(frame_numbers, frame_numbers[1:])
        )
        effective_duration = max(0.0, self.duration_sec - self.warmup_sec)
        minimum_records = max(10, int(effective_duration * 27.0 * 0.95))
        allowed_duplicates = max(1, int(len(records) * 0.001))
        global_time_domain = any(
            "global_time" in domain for domain in domains
        )
        observed = (
            len(records) >= minimum_records
            and len(device_stamps) >= minimum_records
            and len(frame_numbers) >= minimum_records
            and timestamp_regressions == 0
            and counter_regressions == 0
            and timestamp_duplicates <= allowed_duplicates
            and counter_duplicates <= allowed_duplicates
            and global_time_domain
        )
        return {
            "count": len(records),
            "minimum_required": minimum_records,
            "clock_domains": domains,
            "global_time_domain": global_time_domain,
            "timestamp_regressions": timestamp_regressions,
            "timestamp_duplicates": timestamp_duplicates,
            "frame_counter_regressions": counter_regressions,
            "frame_counter_duplicates": counter_duplicates,
            "allowed_duplicates": allowed_duplicates,
        }, observed

    def alignment_report(
        self, lidar_samples: Sequence[Sample]
    ) -> Tuple[Dict[str, Any], List[str]]:
        failures: List[str] = []
        color = self.filtered_stream("color")
        depth = self.filtered_stream("depth")
        imu = self.filtered_stream("imu")
        color_depth = residuals_ms(color, depth)
        camera_imu = residuals_ms(color, imu)
        lidar_camera = residuals_ms(lidar_samples, color)
        lidar_imu = residuals_ms(lidar_samples, imu)

        ordered_color = sorted(color, key=lambda sample: sample.stamp_ns)
        ordered_imu = sorted(imu, key=lambda sample: sample.stamp_ns)
        camera_limit = float(
            self.get_parameter("lidar_camera_p99_ms").value
        )
        imu_limit = float(self.get_parameter("camera_imu_p99_ms").value)
        triple_spans: List[float] = []
        covered = 0
        for lidar in lidar_samples:
            camera = nearest_sample(ordered_color, lidar.stamp_ns)
            imu_sample = nearest_sample(ordered_imu, lidar.stamp_ns)
            if camera is None or imu_sample is None:
                continue
            camera_delta = abs(camera.stamp_ns - lidar.stamp_ns) / 1e6
            imu_delta = abs(imu_sample.stamp_ns - lidar.stamp_ns) / 1e6
            if camera_delta <= camera_limit and imu_delta <= imu_limit:
                covered += 1
            stamps = [lidar.stamp_ns, camera.stamp_ns, imu_sample.stamp_ns]
            triple_spans.append((max(stamps) - min(stamps)) / 1e6)
        coverage = covered / len(lidar_samples) if lidar_samples else 0.0
        minimum_coverage = float(
            self.get_parameter("minimum_coverage").value
        )
        if coverage < minimum_coverage:
            failures.append(
                f"triple coverage {coverage:.3f} < {minimum_coverage:.3f}"
            )

        checks = (
            (
                "camera-IMU p95",
                camera_imu,
                0.95,
                "camera_imu_p95_ms",
            ),
            (
                "camera-IMU p99",
                camera_imu,
                0.99,
                "camera_imu_p99_ms",
            ),
            (
                "LiDAR-camera p95",
                lidar_camera,
                0.95,
                "lidar_camera_p95_ms",
            ),
            (
                "LiDAR-camera p99",
                lidar_camera,
                0.99,
                "lidar_camera_p99_ms",
            ),
            (
                "triple span p95",
                triple_spans,
                0.95,
                "triple_span_p95_ms",
            ),
            (
                "triple span p99",
                triple_spans,
                0.99,
                "triple_span_p99_ms",
            ),
        )
        for label, values, fraction, parameter in checks:
            measured = percentile([abs(value) for value in values], fraction)
            threshold = float(self.get_parameter(parameter).value)
            if measured is None:
                failures.append(f"{label}: no matched samples")
            elif measured > threshold:
                failures.append(
                    f"{label}: {measured:.2f} ms > {threshold:.2f} ms"
                )
        rgb_depth_p99 = percentile(
            [abs(value) for value in color_depth], 0.99
        )
        if rgb_depth_p99 is None or rgb_depth_p99 > 2.0:
            failures.append(
                "RGB-depth p99 missing or greater than 2.00 ms"
            )
        return {
            "coverage": finite_round(coverage, 4),
            "rgb_depth": residual_summary(color_depth),
            "camera_imu": residual_summary(camera_imu),
            "lidar_camera": residual_summary(lidar_camera),
            "lidar_imu": residual_summary(lidar_imu),
            "triple_span": {
                "count": len(triple_spans),
                "p95_ms": finite_round(percentile(triple_spans, 0.95)),
                "p99_ms": finite_round(percentile(triple_spans, 0.99)),
                "max_ms": finite_round(
                    max(triple_spans) if triple_spans else None
                ),
            },
        }, failures

    def timing_report(self) -> Tuple[Dict[str, Any], List[str]]:
        camera_parameters = self.fetch_remote_parameters(
            "/camera/camera",
            [
                "enable_sync",
                "depth_module.inter_cam_sync_mode",
                "depth_module.global_time_enabled",
                "rgb_camera.global_time_enabled",
            ],
        )
        xsens_parameters = self.fetch_remote_parameters(
            "/xsens_mti_node", ["time_option"]
        )
        velodyne_parameters = self.fetch_remote_parameters(
            "/velodyne_driver_node",
            ["gps_time", "timestamp_first_packet", "time_offset"],
        )
        component_status, failures = evaluate_timing_parameters(
            camera_parameters,
            xsens_parameters,
            velodyne_parameters,
            self.timing_expectations,
        )
        color_metadata, color_valid = self.metadata_report("color")
        depth_metadata, depth_valid = self.metadata_report("depth")
        if Metadata is not None and not color_valid:
            failures.append(
                "D455f color metadata is sparse, non-monotonic, or not global_time"
            )
        if Metadata is not None and not depth_valid:
            failures.append(
                "D455f depth metadata is sparse, non-monotonic, or not global_time"
            )

        end_ns = self.ended_steady_ns or time.monotonic_ns()
        time_refs = [
            reference
            for reference in self.xsens_time_refs
            if self.cutoff_steady_ns <= reference[2] <= end_ns
        ]
        time_ref_monotonic = all(
            second[1] > first[1]
            for first, second in zip(time_refs, time_refs[1:])
        )
        if time_refs and not time_ref_monotonic:
            failures.append("Xsens sample-time reference regressed")

        chrony = chrony_tracking_report()
        require_chrony = bool(
            self.timing_expectations["require_chrony_sync"]
        )
        if require_chrony and not chrony["synchronized"]:
            failures.append(
                "Chrony is absent, unsynchronized, or exceeds 5 ms offset"
            )
        configured = all(component_status.values())
        return {
            "level": "VERIFIED" if not failures else "FAILED",
            "active_profile": self.active_profile,
            "role": self.timing_expectations["role"],
            "active_profile_configured": configured,
            "expectations": self.timing_expectations,
            "camera": {
                "configured": component_status["camera"],
                "parameters": camera_parameters,
                "color_metadata": color_metadata,
                "depth_metadata": depth_metadata,
            },
            "xsens": {
                "configured": component_status["xsens"],
                "parameters": xsens_parameters,
                "time_reference_count": len(time_refs),
                "time_reference_monotonic": time_ref_monotonic,
                "utc_reference_count": len(self.xsens_utc_refs),
            },
            "velodyne": {
                "configured": component_status["velodyne"],
                "parameters": velodyne_parameters,
                "scan_duration_median_ms": finite_round(
                    statistics.median(self.velodyne_scan_durations_ms)
                    if self.velodyne_scan_durations_ms
                    else None
                ),
            },
            "host_clock": {
                "required": require_chrony,
                "verified": bool(chrony["synchronized"]),
                "chrony": chrony,
            },
            "metadata_parse_errors": self.metadata_parse_errors,
        }, failures

    def finalize(self) -> Tuple[Dict[str, Any], int]:
        self.ended_steady_ns = time.monotonic_ns()
        lidar_name, lidar_samples = self.choose_lidar_stream()
        camera_min = self.expected_camera_hz * 0.90
        camera_max = self.expected_camera_hz * 1.12
        stream_specs = {
            "color": ((camera_min, camera_max), 250.0, 100.0),
            "depth": ((camera_min, camera_max), 250.0, 100.0),
            "imu": (
                (
                    self.expected_imu_hz * 0.90,
                    self.expected_imu_hz * 1.10,
                ),
                100.0,
                50.0,
            ),
            lidar_name: ((8.5, 11.5), 250.0, 300.0),
        }
        stream_reports: Dict[str, Any] = {}
        data_failures: List[str] = []
        for name, (rate, age, gap) in stream_specs.items():
            report, failures = self.stream_report(name, rate, age, gap)
            stream_reports[name] = report
            data_failures.extend(failures)
        alignment, alignment_failures = self.alignment_report(lidar_samples)
        timing, timing_failures = self.timing_report()
        all_failures = data_failures + alignment_failures + timing_failures
        verdict = "FAIL" if all_failures else "PASS"
        exit_code = 1 if all_failures else 0
        return {
            "verdict": verdict,
            "exit_code": exit_code,
            "policy": str(POLICY_PATH),
            "profile": self.active_profile,
            "role": self.timing_expectations["role"],
            "duration_sec": self.duration_sec,
            "warmup_sec": self.warmup_sec,
            "lidar_source": lidar_name,
            "streams": stream_reports,
            "alignment": alignment,
            "timing_evidence": timing,
            "failures": {
                "data_validity": data_failures,
                "operational_alignment": alignment_failures,
                "timing_configuration": timing_failures,
            },
        }, exit_code


def print_human_report(report: Dict[str, Any]) -> None:
    print("\n=== Triple sensor timestamp verification ===")
    print(f"Verdict: {report['verdict']}")
    print(f"Profile: {report['profile']} ({report['role']})")
    print(f"LiDAR source: {report['lidar_source']}")
    for name, stream in report["streams"].items():
        print(
            f"{name:18} count={stream.get('count', 0):5} "
            f"rate={stream.get('rate_hz')} Hz "
            f"age_p99={stream.get('age_p99_ms')} ms "
            f"drift={stream.get('relative_drift_ppm')} ppm"
        )
    alignment = report["alignment"]
    print(
        "alignment: "
        f"coverage={alignment.get('coverage')} "
        f"camera-imu p99={alignment['camera_imu'].get('abs_p99_ms')} ms "
        f"lidar-camera p99={alignment['lidar_camera'].get('abs_p99_ms')} ms "
        f"triple-span p99={alignment['triple_span'].get('p99_ms')} ms"
    )
    print(
        "timing profile: "
        f"{report['timing_evidence']['level']}"
    )
    for failure in (
        failure
        for group in report["failures"].values()
        for failure in group
    ):
        print(f"FAIL: {failure}")


def main() -> int:
    rclpy.init()
    node = TripleSyncVerifier()
    try:
        while rclpy.ok() and not node.collection_done():
            rclpy.spin_once(node, timeout_sec=0.1)
        report, exit_code = node.finalize()
        output_path = Path(node.output_json).expanduser()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print_human_report(report)
        print(f"JSON report: {output_path}")
        return exit_code
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
