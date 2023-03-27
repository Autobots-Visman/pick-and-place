#!/usr/bin/env python3
"""Generates a texture for use in models/aruco_tile/description.sdf"""
from argparse import ArgumentParser
from pathlib import Path

import cv2


def parse_args():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        default=(
            Path(__file__).parent.parent
            / "models/aruco_tile/materials/textures/aruco.png"
        ).as_posix(),
    )
    parser.add_argument("--force", action="store_true", help="Overwrite existing files")
    # ignore any other args
    args, _ = parser.parse_known_args()
    return args


def main():
    args = parse_args()
    if Path(args.output).exists() and not args.force:
        raise FileExistsError(
            f"{args.output} already exists. Use --force to overwrite."
        )
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
    img = cv2.aruco.drawMarker(aruco_dict, 0, 200)
    cv2.imwrite(args.output, img)


if __name__ == "__main__":
    main()
