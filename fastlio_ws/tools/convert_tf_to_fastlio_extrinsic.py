#!/usr/bin/env python3

import argparse
import math
from typing import List, Tuple


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> List[List[float]]:
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def mat_transpose(mat: List[List[float]]) -> List[List[float]]:
    return [list(row) for row in zip(*mat)]


def mat_vec_mul(mat: List[List[float]], vec: Tuple[float, float, float]) -> List[float]:
    return [
        mat[0][0] * vec[0] + mat[0][1] * vec[1] + mat[0][2] * vec[2],
        mat[1][0] * vec[0] + mat[1][1] * vec[1] + mat[1][2] * vec[2],
        mat[2][0] * vec[0] + mat[2][1] * vec[1] + mat[2][2] * vec[2],
    ]


def invert_transform(rotation: List[List[float]], translation: Tuple[float, float, float]) -> Tuple[List[List[float]], List[float]]:
    rotation_inv = mat_transpose(rotation)
    translation_inv = mat_vec_mul(rotation_inv, (-translation[0], -translation[1], -translation[2]))
    return rotation_inv, translation_inv


def format_matrix(rotation: List[List[float]]) -> str:
    values = [
        rotation[0][0], rotation[0][1], rotation[0][2],
        rotation[1][0], rotation[1][1], rotation[1][2],
        rotation[2][0], rotation[2][1], rotation[2][2],
    ]
    return "[ " + ", ".join(f"{value:.6f}" for value in values) + " ]"


def format_translation(translation: Tuple[float, float, float]) -> str:
    return "[ " + ", ".join(f"{value:.6f}" for value in translation) + " ]"


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert a TF translation + RPY into FAST-LIO extrinsic_R/T values."
    )
    parser.add_argument("--translation", nargs=3, type=float, metavar=("X", "Y", "Z"), required=True)
    parser.add_argument("--rpy", nargs=3, type=float, metavar=("ROLL", "PITCH", "YAW"), required=True)
    parser.add_argument(
        "--invert",
        action="store_true",
        help="Invert the transform before printing FAST-LIO extrinsics.",
    )
    parser.add_argument(
        "--label",
        default="mapping",
        help="YAML section label to print (default: mapping).",
    )
    args = parser.parse_args()

    translation = (args.translation[0], args.translation[1], args.translation[2])
    roll, pitch, yaw = args.rpy
    rotation = rpy_to_matrix(roll, pitch, yaw)

    if args.invert:
        rotation, translation_list = invert_transform(rotation, translation)
        translation = (translation_list[0], translation_list[1], translation_list[2])

    print(f"{args.label}:")
    print(f"  extrinsic_T: {format_translation(translation)}")
    print("  extrinsic_R:")
    print(f"    {format_matrix(rotation)}")


if __name__ == "__main__":
    main()
