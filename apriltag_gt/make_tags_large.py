#!/usr/bin/env python3
"""Large AprilTag tag36h11 markers tiled across 2x2 A4 sheets.

A single A4 caps the tag at 160 mm, which on the measured D455F intrinsics
(fx=388.6 @ 640x480) only gives a trustworthy pose inside ~1 m. Corridor-scale
anchors need a bigger square, so each tag here is printed as four A4 pages that
are butt-joined into one 420x594 mm sheet.

  print_A3x/tag36h11_id###_tile{TL,TR,BL,BR}.png

Assembly: print all four at 100%, trim each page along the printed trim line,
butt the trimmed edges together (do not overlap), tape on the BACK only.
Tape on the front changes the reflectance and can break thresholding.

Run with an interpreter that has OpenCV, e.g.
  ~/miniconda3/envs/rtabmap_imu/bin/python make_tags_large.py
"""

from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent
OUT_DIR = ROOT / "print_A3x"

N_TAGS = 8
DPI = 300
CELLS_MARKER = 8
BLACK_EDGE_MM = 300.0

A4_W_MM, A4_H_MM = 210.0, 297.0
SHEET_W_MM, SHEET_H_MM = 2 * A4_W_MM, 2 * A4_H_MM
FONT_PATH = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
FONT_BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"

TILE_NAMES = (("TL", "TR"), ("BL", "BR"))


def mm2px(mm: float) -> int:
    return int(round(mm / 25.4 * DPI))


def build_full_sheet(dictionary, tag_id: int) -> Image.Image:
    cell_px = mm2px(BLACK_EDGE_MM / CELLS_MARKER)
    marker = cv2.aruco.generateImageMarker(
        dictionary, tag_id, CELLS_MARKER * cell_px
    )
    sheet = Image.new("L", (mm2px(SHEET_W_MM), mm2px(SHEET_H_MM)), 255)
    tag_img = Image.fromarray(marker)
    x = (sheet.width - tag_img.width) // 2
    y = mm2px(70.0)
    sheet.paste(tag_img, (x, y))

    draw = ImageDraw.Draw(sheet)
    title = ImageFont.truetype(FONT_BOLD, 64)
    body = ImageFont.truetype(FONT_PATH, 42)

    draw.text((mm2px(25.0), mm2px(22.0)),
              f"AprilTag  tag36h11  ID = {tag_id}", font=title, fill=0)
    draw.text((mm2px(25.0), mm2px(40.0)),
              f"black square edge = {BLACK_EDGE_MM:.0f} mm  "
              f"(4 x A4, butt-joined)", font=body, fill=0)

    y_txt = 70.0 + BLACK_EDGE_MM + 30.0
    sx0, sx1 = mm2px(25.0), mm2px(25.0 + 100.0)
    py = mm2px(y_txt)
    draw.line([(sx0, py), (sx1, py)], fill=0, width=5)
    for sx in (sx0, sx1):
        draw.line([(sx, py - mm2px(3.5)), (sx, py + mm2px(3.5))], fill=0, width=5)
    draw.text((sx0, py + mm2px(6.0)),
              "this bar must measure exactly 100.0 mm after printing",
              font=body, fill=0)

    y_txt += 22.0
    for line in (
        "Print all 4 tiles at 100% / actual size.",
        "Trim each tile on the dashed line, butt the edges, tape on the BACK.",
        "Then measure the real black-square edge and record it to 1 mm.",
        "Mount on a rigid flat board. A bent tag biases the pose estimate.",
    ):
        draw.text((mm2px(25.0), mm2px(y_txt)), line, font=body, fill=0)
        y_txt += 12.0
    return sheet


def add_trim_marks(tile: Image.Image, row: int, col: int) -> Image.Image:
    """Dashed line on the two edges that get trimmed and joined."""
    draw = ImageDraw.Draw(tile)
    label = ImageFont.truetype(FONT_BOLD, 40)
    dash, gap = mm2px(4.0), mm2px(3.0)

    def dashed(x0, y0, x1, y1):
        if x0 == x1:
            y = y0
            while y < y1:
                draw.line([(x0, y), (x0, min(y + dash, y1))], fill=128, width=2)
                y += dash + gap
        else:
            x = x0
            while x < x1:
                draw.line([(x, y0), (min(x + dash, x1), y0)], fill=128, width=2)
                x += dash + gap

    inner_x = 0 if col == 1 else tile.width - 1
    inner_y = 0 if row == 1 else tile.height - 1
    dashed(inner_x, 0, inner_x, tile.height)
    dashed(0, inner_y, tile.width, inner_y)

    tag = TILE_NAMES[row][col]
    tx = mm2px(6.0) if col == 0 else tile.width - mm2px(26.0)
    ty = mm2px(6.0) if row == 0 else tile.height - mm2px(20.0)
    draw.text((tx, ty), tag, font=label, fill=128)
    return tile


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    tw, th = mm2px(A4_W_MM), mm2px(A4_H_MM)

    for tag_id in range(N_TAGS):
        full = build_full_sheet(dictionary, tag_id)
        for row in range(2):
            for col in range(2):
                box = (col * tw, row * th, (col + 1) * tw, (row + 1) * th)
                tile = add_trim_marks(full.crop(box).copy(), row, col)
                name = f"tag36h11_id{tag_id:03d}_tile{TILE_NAMES[row][col]}.png"
                tile.save(OUT_DIR / name, dpi=(DPI, DPI))

    quiet = (SHEET_W_MM - BLACK_EDGE_MM) / 2
    print(f"{N_TAGS} large tags x 4 tiles -> {OUT_DIR}")
    print(f"black edge {BLACK_EDGE_MM:.0f} mm, cell {BLACK_EDGE_MM/CELLS_MARKER:.1f} mm, "
          f"side quiet zone {quiet:.1f} mm (need >= {BLACK_EDGE_MM/CELLS_MARKER:.1f})")


if __name__ == "__main__":
    main()
