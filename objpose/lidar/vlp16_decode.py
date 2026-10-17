"""vlp16_decode.py — Velodyne VLP-16 / Puck Hi-Res UDP packets -> points, without the ROS velodyne driver (2026-09-29).

The Samsung laptop's ROS 2 env has no `velodyne` package and no internet, so the live LiDAR lane decodes the 1206-byte
data packets itself. The output reproduces the ROS 2 velodyne_pointcloud organized cloud of our recordings:
  one row per firing (packets x 12 blocks x 2 firing sequences), 16 columns = rings (0 = lowest beam),
  fields x y z intensity (float32) ring (uint16) time (float32, s relative to the last packet of the scan),
  NaN for no return. Frame "velodyne": x forward, y left, z up.
Validated element-wise against the driver's /velodyne_points of 260901_cbnu_longcircle (objpose/lidar/vlp16_check.py).
"""
from __future__ import annotations

import numpy as np

BLOCKS, CHANNELS, SEQS = 12, 16, 2
BLOCK_BYTES = 100
FIRING_US, CHANNEL_US = 55.296, 2.304          # per firing sequence / per channel (VLP-16 manual)
DIST_RES = 0.002                               # m per unit
# per channel in packet order: vertical angle (deg) and the vertical offset of the optical centre (m, VLP-16 manual).
# Our sensor reports product byte 0x24 but the recordings' driver used the VLP-16 table and the fit on
# 260901_cbnu_longcircle reproduces its points exactly with it (vlp16_check.py): always use VLP-16.
VERT = [-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15]
VERT_OFFSET = [0.0112, -0.0007, 0.0097, -0.0022, 0.0081, -0.0037, 0.0066, -0.0051,
               0.0051, -0.0066, 0.0037, -0.0081, 0.0022, -0.0097, 0.0007, -0.0112]
PACKET_US = BLOCKS * SEQS * FIRING_US              # 1327.104 us


def product_id(packet: bytes) -> int:
    return packet[1205]


def ring_order(vert):
    """ring index of each packet channel: 0 = lowest beam, like the ROS driver"""
    order = np.argsort(vert)
    ring = np.empty(len(vert), np.int64)
    ring[order] = np.arange(len(vert))
    return ring


def decode_scan(packets: list[bytes], min_range=0.4, max_range=130.0, vert=None, vert_offset=None):
    """packets of one revolution -> dict of organized arrays [firings, 16] (x, y, z, intensity, ring, time)"""
    n = len(packets)
    raw = np.frombuffer(b"".join(p[:1200] for p in packets), np.uint8).reshape(n, BLOCKS, BLOCK_BYTES)
    az = raw[:, :, 2].astype(np.int32) | (raw[:, :, 3].astype(np.int32) << 8)            # [n, 12] 0.01 deg
    data = raw[:, :, 4:].reshape(n, BLOCKS, SEQS, CHANNELS, 3)
    dist = (data[..., 0].astype(np.int32) | (data[..., 1].astype(np.int32) << 8)) * DIST_RES
    inten = data[..., 2].astype(np.float32)
    vert = np.radians(np.asarray(vert if vert is not None else VERT, float))
    voff = np.asarray(vert_offset if vert_offset is not None else VERT_OFFSET, float)
    # azimuth of every firing: block azimuth, the second sequence half-way to the next block, then per-channel timing
    azf = az.reshape(-1).astype(np.float64)
    gap = np.diff(azf, append=azf[-1] + (azf[-1] - azf[-2]))
    gap = np.where(gap < 0, gap + 36000, gap)
    gap = gap.reshape(n, BLOCKS)
    seq_t = np.arange(SEQS)[:, None] * FIRING_US + np.arange(CHANNELS)[None, :] * CHANNEL_US    # [2, 16] us
    azc = az[:, :, None, None] + gap[:, :, None, None] * seq_t[None, None] / (2 * FIRING_US)
    azr = np.radians((azc % 36000) / 100.0)
    cv, sv = np.cos(vert), np.sin(vert)
    xy = dist * cv - voff * sv                       # velodyne_pointcloud: optical-centre offset along the beam normal
    x = xy * np.cos(azr)
    y = -xy * np.sin(azr)
    z = dist * sv + voff * cv
    ok = (dist >= min_range) & (dist <= max_range)
    # time relative to the LAST packet of the scan (the driver stamps the scan with it): 1 packet = 1327.104 us
    tt = ((np.arange(n)[:, None, None, None] - (n - 1)) * PACKET_US + np.arange(BLOCKS)[None, :, None, None] * SEQS * FIRING_US
          + seq_t[None, None]) * 1e-6
    ring = ring_order(vert)
    shape = (n * BLOCKS * SEQS, CHANNELS)
    out = {k: np.where(ok, v, np.nan).reshape(shape).astype(np.float32) for k, v in (("x", x), ("y", y), ("z", z), ("intensity", inten))}
    out["time"] = np.broadcast_to(tt, x.shape).reshape(shape).astype(np.float32)
    # columns ordered by ring (the driver's organized cloud is [firing, ring])
    inv = np.argsort(ring)
    for k in out:
        out[k] = out[k][:, inv]
    out["ring"] = np.broadcast_to(np.arange(CHANNELS, dtype=np.uint16), shape).copy()
    out["valid"] = ok.reshape(shape)[:, inv]
    return out
