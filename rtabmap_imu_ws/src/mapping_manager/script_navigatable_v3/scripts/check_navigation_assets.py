#!/usr/bin/env python3

"""Verify that a Nav2 static map and RTAB-Map database are one usable pair."""

from __future__ import annotations

import argparse
import math
import sqlite3
import sys
import zlib
from pathlib import Path

import yaml


LOOP_TYPES = {
    1: "global",
    2: "local-space",
    3: "local-time",
    4: "user",
    7: "pose-prior",
    8: "landmark",
}


def fail(message: str) -> None:
    raise RuntimeError(message)


def read_pgm(path: Path) -> tuple[int, int, bytes]:
    raw = path.read_bytes()
    index = 0

    def token() -> bytes:
        nonlocal index
        while index < len(raw):
            if raw[index] == ord("#"):
                newline = raw.find(b"\n", index)
                if newline < 0:
                    fail(f"invalid PGM comment in {path}")
                index = newline + 1
            elif chr(raw[index]).isspace():
                index += 1
            else:
                break
        start = index
        while index < len(raw) and not chr(raw[index]).isspace():
            index += 1
        if start == index:
            fail(f"truncated PGM header in {path}")
        return raw[start:index]

    if token() != b"P5":
        fail(f"{path} is not a binary P5 PGM")
    width = int(token())
    height = int(token())
    maximum = int(token())
    if width <= 0 or height <= 0 or maximum != 255:
        fail(f"unsupported PGM geometry {width}x{height}, max={maximum}")

    if index >= len(raw) or not chr(raw[index]).isspace():
        fail(f"missing PGM header terminator in {path}")
    if raw[index:index + 2] == b"\r\n":
        index += 2
    else:
        index += 1
    pixels = raw[index:]
    if len(pixels) != width * height:
        fail(
            f"PGM has {len(pixels)} pixels, expected {width * height} "
            f"for {width}x{height}"
        )
    return width, height, pixels


def db_grid_to_pgm(grid: bytes, width: int, height: int) -> bytes:
    if len(grid) != width * height:
        fail(
            "database occupancy-grid size does not match the PGM: "
            f"{len(grid)} != {width}x{height}"
        )

    pixels = bytearray(width * height)
    output = 0
    for y in range(height - 1, -1, -1):
        row = y * width
        for x in range(width):
            unsigned = grid[row + x]
            occupancy = unsigned if unsigned < 128 else unsigned - 256
            if occupancy < 0:
                pixel = 205
            elif occupancy >= 65:
                pixel = 0
            elif occupancy <= 25:
                pixel = 254
            else:
                pixel = 205
            pixels[output] = pixel
            output += 1
    return bytes(pixels)


def unique_link_counts(connection: sqlite3.Connection) -> dict[int, int]:
    pairs: dict[int, set[tuple[int, int]]] = {}
    for from_id, to_id, link_type in connection.execute(
        "SELECT from_id, to_id, type FROM Link"
    ):
        pair = (min(int(from_id), int(to_id)), max(int(from_id), int(to_id)))
        pairs.setdefault(int(link_type), set()).add(pair)
    return {link_type: len(values) for link_type, values in pairs.items()}


def check(db_path: Path, map_path: Path, allow_unoptimized: bool) -> None:
    map_config = yaml.safe_load(map_path.read_text(encoding="utf-8"))
    if not isinstance(map_config, dict):
        fail(f"invalid map YAML: {map_path}")

    image_value = map_config.get("image")
    if not isinstance(image_value, str) or not image_value:
        fail(f"map YAML has no image path: {map_path}")
    image_path = Path(image_value)
    if not image_path.is_absolute():
        image_path = map_path.parent / image_path
    if not image_path.is_file():
        fail(f"map image does not exist: {image_path}")

    try:
        resolution = float(map_config["resolution"])
        origin = [float(value) for value in map_config["origin"]]
    except (KeyError, TypeError, ValueError) as error:
        fail(f"invalid map resolution/origin: {error}")
    if len(origin) != 3 or resolution <= 0.0:
        fail(f"invalid map resolution/origin: {resolution}, {origin}")

    width, height, pgm_pixels = read_pgm(image_path)

    with sqlite3.connect(f"file:{db_path}?mode=ro", uri=True) as connection:
        admin_rows = connection.execute(
            "SELECT opt_map, opt_map_x_min, opt_map_y_min, "
            "opt_map_resolution FROM Admin"
        ).fetchall()
        if len(admin_rows) != 1:
            fail(f"database has {len(admin_rows)} Admin rows, expected one")
        compressed_grid, db_x, db_y, db_resolution = admin_rows[0]
        if not compressed_grid:
            fail("database has no saved optimized occupancy grid")

        try:
            db_grid = zlib.decompress(compressed_grid)
        except zlib.error as error:
            fail(f"cannot decompress database occupancy grid: {error}")

        if not math.isclose(resolution, float(db_resolution), abs_tol=1.0e-6):
            fail(
                "map/database resolution mismatch: "
                f"{resolution} != {float(db_resolution)}"
            )
        if not math.isclose(origin[0], float(db_x), abs_tol=1.0e-5) or not math.isclose(
            origin[1], float(db_y), abs_tol=1.0e-5
        ):
            fail(
                "map/database origin mismatch: "
                f"{origin[:2]} != [{float(db_x)}, {float(db_y)}]"
            )
        if abs(origin[2]) > 1.0e-6:
            fail(f"rotated map origin is unsupported by this workflow: {origin[2]}")

        expected_pixels = db_grid_to_pgm(db_grid, width, height)
        if expected_pixels != pgm_pixels:
            mismatch_count = sum(
                left != right for left, right in zip(expected_pixels, pgm_pixels)
            )
            fail(
                "map image is not the optimized grid stored in the database "
                f"({mismatch_count} differing pixels)"
            )

        node_count = int(connection.execute("SELECT COUNT(*) FROM Node").fetchone()[0])
        scan_node_count = int(
            connection.execute(
                "SELECT COUNT(*) FROM Data "
                "WHERE scan IS NOT NULL AND length(scan) > 0"
            ).fetchone()[0]
        )
        feature_node_count = int(
            connection.execute(
                "SELECT COUNT(DISTINCT node_id) FROM Feature"
            ).fetchone()[0]
        )
        combined_node_count = int(
            connection.execute(
                "SELECT COUNT(*) FROM ("
                "SELECT DISTINCT Feature.node_id FROM Feature "
                "JOIN Data ON Data.id = Feature.node_id "
                "WHERE Data.scan IS NOT NULL AND length(Data.scan) > 0"
                ")"
            ).fetchone()[0]
        )
        counts = unique_link_counts(connection)
        node_stamps = {
            int(node_id): float(stamp)
            for node_id, stamp in connection.execute(
                "SELECT id, stamp FROM Node"
            )
        }
        long_loop_pairs: set[tuple[int, int, int]] = set()
        for from_id, to_id, link_type in connection.execute(
            "SELECT from_id, to_id, type FROM Link "
            "WHERE type IN (1, 2, 3, 4)"
        ):
            first = int(from_id)
            second = int(to_id)
            link_type = int(link_type)
            pair = (min(first, second), max(first, second), link_type)
            if (
                first in node_stamps
                and second in node_stamps
                and abs(node_stamps[first] - node_stamps[second]) >= 10.0
            ):
                long_loop_pairs.add(pair)

    loop_counts = {name: counts.get(link_type, 0) for link_type, name in LOOP_TYPES.items()}
    global_constraints = sum(loop_counts.values())
    absolute_constraints = loop_counts["pose-prior"] + loop_counts["landmark"]
    print(
        "OK: map/database are an exact pair "
        f"({width}x{height}, {resolution:.3f} m/cell, {node_count} nodes).",
        flush=True,
    )
    print(
        "Graph constraints: "
        + ", ".join(f"{name}={count}" for name, count in loop_counts.items())
        + f", long-baseline={len(long_loop_pairs)}",
        flush=True,
    )
    print(
        "Localization payload coverage: "
        f"scan={scan_node_count}/{node_count}, "
        f"features={feature_node_count}/{node_count}, "
        f"both={combined_node_count}/{node_count}.",
        flush=True,
    )

    if node_count < 2:
        fail("database does not contain a usable trajectory")
    if global_constraints == 0:
        message = (
            "database is only an odometry chain: no loop closure, pose prior, "
            "or landmark can correct accumulated map distortion"
        )
        if allow_unoptimized:
            print(f"WARNING: {message}", file=sys.stderr)
        else:
            fail(message)
    elif not long_loop_pairs and absolute_constraints == 0:
        message = (
            "database has no loop spanning at least 10 seconds and no "
            "pose-prior/landmark anchor; only local redundant links exist"
        )
        if allow_unoptimized:
            print(f"WARNING: {message}", file=sys.stderr)
        else:
            fail(message)
    if combined_node_count / node_count < 0.70:
        print(
            "WARNING: fewer than 70% of nodes contain both visual features "
            "and a scan; VisICP localization coverage may be intermittent",
            file=sys.stderr,
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--database", required=True, type=Path)
    parser.add_argument("--map", required=True, dest="map_path", type=Path)
    parser.add_argument("--allow-unoptimized-map", action="store_true")
    args = parser.parse_args()

    try:
        check(
            args.database.resolve(),
            args.map_path.resolve(),
            args.allow_unoptimized_map,
        )
    except (OSError, sqlite3.Error, RuntimeError) as error:
        print(f"ERROR: navigation asset check failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
