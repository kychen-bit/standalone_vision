"""Dependency-free planar geometry used by the detector."""

import math


def apply_homography(matrix, x, y):
    denominator = matrix[2][0] * x + matrix[2][1] * y + matrix[2][2]
    if abs(denominator) < 1e-12:
        raise ValueError("homography maps point to infinity")
    mapped_x = (matrix[0][0] * x + matrix[0][1] * y + matrix[0][2]) / denominator
    mapped_y = (matrix[1][0] * x + matrix[1][1] * y + matrix[1][2]) / denominator
    return float(mapped_x), float(mapped_y)


def estimate_rigid_pose(reference_points, observed_points):
    if len(reference_points) != len(observed_points) or len(reference_points) < 2:
        raise ValueError("two matching point sets are required")
    count = len(reference_points)
    reference_center = (
        sum(point[0] for point in reference_points) / count,
        sum(point[1] for point in reference_points) / count,
    )
    observed_center = (
        sum(point[0] for point in observed_points) / count,
        sum(point[1] for point in observed_points) / count,
    )
    cross = 0.0
    dot = 0.0
    for reference, observed in zip(reference_points, observed_points):
        rx = reference[0] - reference_center[0]
        ry = reference[1] - reference_center[1]
        ox = observed[0] - observed_center[0]
        oy = observed[1] - observed_center[1]
        dot += rx * ox + ry * oy
        cross += rx * oy - ry * ox
    angle = math.atan2(cross, dot)
    cosine = math.cos(angle)
    sine = math.sin(angle)
    tx = observed_center[0] - (
        cosine * reference_center[0] - sine * reference_center[1]
    )
    ty = observed_center[1] - (
        sine * reference_center[0] + cosine * reference_center[1]
    )
    squared_error = 0.0
    for reference, observed in zip(reference_points, observed_points):
        predicted_x = cosine * reference[0] - sine * reference[1] + tx
        predicted_y = sine * reference[0] + cosine * reference[1] + ty
        squared_error += (predicted_x - observed[0]) ** 2 + (predicted_y - observed[1]) ** 2
    return float(tx), float(ty), math.degrees(angle), math.sqrt(squared_error / count)
