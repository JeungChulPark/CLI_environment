#!/usr/bin/env python3
"""Generate printable AprilTag tag36h11 markers for SLAM ground-truth surveying.

Two outputs per tag id:
  tags/tag36h11_id###.png            raw marker with white quiet zone, no text
  print_A4/A4_tag36h11_id###.png     300 dpi A4 sheet ready to print

The family is tag36h11: the AprilTag authors' recommended default and what
apriltag_ros / TagSLAM assume unless told otherwise. Largest Hamming distance
of the usable families, so false detections are effectively zero.

The sheet prints the 8-cell marker at exactly BLACK_EDGE_MM and leaves the
surrounding page blank. The detector needs one clear cell of white around the
black square; the page margins provide 25 mm where one cell is 20 mm, so no
mark of any kind is drawn near the tag.

Run with an interpreter that has OpenCV, e.g.
  ~/miniconda3/envs/rtabmap_imu/bin/python make_tags.py
"""

from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent
TAG_DIR = ROOT / "tags"
PRINT_DIR = ROOT / "print_A4"

N_TAGS = 20
DPI = 300

# tag36h11 draws as 8x8 cells (6x6 payload + 1 black border ring). The detector
# also needs a white ring around it, so a standalone image is 10x10 cells.
CELLS_MARKER = 8
CELLS_QUIET = 1

# Black square outer edge = the length apriltag_ros wants in its `size` param.
BLACK_EDGE_MM = 160.0

A4_W_MM, A4_H_MM = 210.0, 297.0
FONT_PATH = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
FONT_BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"

TAG_TOP_MM = 48.0
MARGIN_MM = 18.0


def mm2px(mm: float) -> int:
    return int(round(mm / 25.4 * DPI))


def make_marker(dictionary, tag_id: int, cell_px: int, quiet: bool):
    marker = cv2.aruco.generateImageMarker(
        dictionary, tag_id, CELLS_MARKER * cell_px
    )
    if not quiet:
        return marker
    pad = CELLS_QUIET * cell_px
    return cv2.copyMakeBorder(
        marker, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255
    )


def draw_sheet(tag_id: int, marker: np.ndarray) -> Image.Image:
    sheet = Image.new("L", (mm2px(A4_W_MM), mm2px(A4_H_MM)), 255)
    draw = ImageDraw.Draw(sheet)
    title = ImageFont.truetype(FONT_BOLD, 46)
    body = ImageFont.truetype(FONT_PATH, 32)
    small = ImageFont.truetype(FONT_PATH, 27)

    tag_img = Image.fromarray(marker)
    sheet.paste(tag_img, ((sheet.width - tag_img.width) // 2, mm2px(TAG_TOP_MM)))

    draw.text(
        (mm2px(MARGIN_MM), mm2px(13.0)),
        f"AprilTag  tag36h11  ID = {tag_id}",
        font=title, fill=0,
    )
    draw.text(
        (mm2px(MARGIN_MM), mm2px(25.0)),
        f"black square edge = {BLACK_EDGE_MM:.0f} mm",
        font=body, fill=0,
    )

    # Nothing is drawn between 26 mm and the tag, or beside it: the blank page
    # is the detector's quiet zone.
    y = TAG_TOP_MM + BLACK_EDGE_MM + 24.0
    draw.text(
        (mm2px(MARGIN_MM), mm2px(y)),
        "PRINT AT 100% / ACTUAL SIZE. No 'fit to page', no 'shrink to margin'.",
        font=body, fill=0,
    )

    # Scale-check bar: printers silently rescale, this catches it in 5 seconds.
    y += 13.0
    sx0 = mm2px(MARGIN_MM)
    sx1 = sx0 + mm2px(100.0)
    py = mm2px(y)
    draw.line([(sx0, py), (sx1, py)], fill=0, width=4)
    for sx in (sx0, sx1):
        draw.line([(sx, py - mm2px(3.0)), (sx, py + mm2px(3.0))], fill=0, width=4)
    draw.text(
        (sx0, py + mm2px(4.5)),
        "measure this bar: must be exactly 100.0 mm",
        font=small, fill=0,
    )

    y += 16.0
    for line in (
        "1. Check the 100 mm bar with a ruler BEFORE mounting.",
        "2. Measure the real black-square edge, record it to 0.5 mm.",
        "3. Glue to a rigid flat board. A bent tag biases the pose.",
        "4. Never print two sheets with the same ID.",
        "5. Record this ID and its surveyed position in tag_survey.csv.",
    ):
        draw.text((mm2px(MARGIN_MM), mm2px(y)), line, font=small, fill=0)
        y += 7.5

    return sheet


def main() -> None:
    TAG_DIR.mkdir(parents=True, exist_ok=True)
    PRINT_DIR.mkdir(parents=True, exist_ok=True)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)

    cell_px = mm2px(BLACK_EDGE_MM / CELLS_MARKER)
    for tag_id in range(N_TAGS):
        cv2.imwrite(
            str(TAG_DIR / f"tag36h11_id{tag_id:03d}.png"),
            make_marker(dictionary, tag_id, cell_px, quiet=True),
        )
        sheet = draw_sheet(
            tag_id, make_marker(dictionary, tag_id, cell_px, quiet=False)
        )
        sheet.save(PRINT_DIR / f"A4_tag36h11_id{tag_id:03d}.png", dpi=(DPI, DPI))

    side = mm2px(BLACK_EDGE_MM)
    white = (mm2px(A4_W_MM) - side) // 2
    print(f"{N_TAGS} tags       -> {TAG_DIR}")
    print(f"{N_TAGS} A4 sheets  -> {PRINT_DIR}  ({DPI} dpi)")
    print(f"black edge {BLACK_EDGE_MM:.0f} mm, cell {BLACK_EDGE_MM/CELLS_MARKER:.1f} mm, "
          f"side quiet zone {white/DPI*25.4:.1f} mm (need >= {BLACK_EDGE_MM/CELLS_MARKER:.1f})")


if __name__ == "__main__":
    main()
