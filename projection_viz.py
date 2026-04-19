#!/usr/bin/env python3
"""Generate horizontal/vertical projection visualizations for an input image.

Outputs:
1) Horizontal projection block: 64 x W (W = input image width)
2) Vertical projection block: H x 64 (H = input image height)

Projection definitions used here:
- Horizontal projection (along x-axis): per-column intensity (sum over rows)
- Vertical projection (along y-axis): per-row intensity (sum over columns)
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from PIL import Image, ImageOps


H_STRIP = 64
V_STRIP = 64


def normalize_to_u8(values: np.ndarray) -> np.ndarray:
    values = values.astype(np.float64)
    vmin = float(values.min())
    vmax = float(values.max())
    if vmax <= vmin:
        return np.zeros_like(values, dtype=np.uint8)
    out = (values - vmin) / (vmax - vmin)
    return (out * 255.0).clip(0, 255).astype(np.uint8)


def projection_blocks(gray: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    # Per-column and per-row projections
    proj_x = gray.sum(axis=0)  # length W
    proj_y = gray.sum(axis=1)  # length H

    proj_x_u8 = normalize_to_u8(proj_x)
    proj_y_u8 = normalize_to_u8(proj_y)

    # Requested visualization shapes:
    # horizontal block -> 64 x W
    # vertical block   -> H x 64
    horiz_block = np.tile(proj_x_u8[np.newaxis, :], (H_STRIP, 1))
    vert_block = np.tile(proj_y_u8[:, np.newaxis], (1, V_STRIP))

    return horiz_block, vert_block


def add_label_strip(img: Image.Image, label: str, strip_height: int = 26) -> Image.Image:
    canvas = Image.new("L", (img.width, img.height + strip_height), color=255)
    canvas.paste(img, (0, strip_height))

    # Use default bitmap font from PIL (no extra dependencies)
    from PIL import ImageDraw

    draw = ImageDraw.Draw(canvas)
    draw.text((8, 6), label, fill=0)
    return canvas


def save_outputs(input_path: Path, out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)

    img = Image.open(input_path)
    gray_img = ImageOps.grayscale(img)
    gray = np.asarray(gray_img, dtype=np.uint8)

    horiz_block, vert_block = projection_blocks(gray)

    stem = input_path.stem

    gray_out = out_dir / f"{stem}_gray.png"
    horiz_out = out_dir / f"{stem}_horizontal_projection_64xW.png"
    vert_out = out_dir / f"{stem}_vertical_projection_Hx64.png"
    panel_out = out_dir / f"{stem}_projection_panel.png"

    gray_img.save(gray_out)
    Image.fromarray(horiz_block, mode="L").save(horiz_out)
    Image.fromarray(vert_block, mode="L").save(vert_out)

    # Build a simple panel preview
    gray_l = add_label_strip(gray_img, f"Input grayscale: {gray.shape[0]}x{gray.shape[1]}")
    horiz_l = add_label_strip(
        Image.fromarray(horiz_block, mode="L"),
        f"Horizontal projection block: {H_STRIP}x{gray.shape[1]}",
    )
    vert_l = add_label_strip(
        Image.fromarray(vert_block, mode="L"),
        f"Vertical projection block: {gray.shape[0]}x{V_STRIP}",
    )

    panel_w = max(gray_l.width, horiz_l.width, vert_l.width)
    panel_h = gray_l.height + horiz_l.height + vert_l.height + 8 * 2
    panel = Image.new("L", (panel_w, panel_h), color=245)

    y = 0
    panel.paste(gray_l, (0, y))
    y += gray_l.height + 8
    panel.paste(horiz_l, (0, y))
    y += horiz_l.height + 8
    panel.paste(vert_l, (0, y))

    panel.save(panel_out)

    print(f"Saved: {gray_out}")
    print(f"Saved: {horiz_out}")
    print(f"Saved: {vert_out}")
    print(f"Saved: {panel_out}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate horizontal and vertical projection visualizations.",
    )
    parser.add_argument("input", type=Path, help="Input image path")
    parser.add_argument(
        "-o",
        "--out-dir",
        type=Path,
        default=Path("data_out/projections"),
        help="Output directory (default: data_out/projections)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    save_outputs(args.input, args.out_dir)


if __name__ == "__main__":
    main()
