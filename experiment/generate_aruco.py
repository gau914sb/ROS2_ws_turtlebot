#!/usr/bin/env python3

import cv2
import argparse


def main():
    parser = argparse.ArgumentParser(description="Generate an ArUco marker")
    parser.add_argument(
        "--id",
        type=int,
        default=4,
        help="Marker ID"
    )
    parser.add_argument(
        "--dict",
        type=str,
        default="DICT_4X4_50",
        help="Dictionary name"
    )
    parser.add_argument(
        "--size",
        type=int,
        default=600,
        help="Marker image size in pixels"
    )
    parser.add_argument(
        "--output",
        type=str,
        default="aruco_marker.png",
        help="Output image filename"
    )

    args = parser.parse_args()

    # Available dictionaries
    dictionaries = {
        "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
        "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
        "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
        "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
        "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
        "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
        "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
        "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
        "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    }

    if args.dict not in dictionaries:
        print(f"Unknown dictionary: {args.dict}")
        return

    dictionary = cv2.aruco.getPredefinedDictionary(dictionaries[args.dict])

    marker = cv2.aruco.generateImageMarker(
        dictionary,
        args.id,
        args.size
    )

    cv2.imwrite(args.output, marker)

    print("Marker generated successfully!")
    print(f"Dictionary : {args.dict}")
    print(f"Marker ID  : {args.id}")
    print(f"Image Size : {args.size} x {args.size}")
    print(f"Saved as   : {args.output}")


if __name__ == "__main__":
    main()