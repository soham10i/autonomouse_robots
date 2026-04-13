"""Camera-based perception: pillar detection, green ground, floating walls."""

import math
import struct
from constants import (
    BLUE_H_MIN, BLUE_H_MAX, BLUE_S_MIN, BLUE_V_MIN,
    YELLOW_H_MIN, YELLOW_H_MAX, YELLOW_S_MIN, YELLOW_V_MIN,
    GREEN_H_MIN, GREEN_H_MAX, GREEN_S_MIN, GREEN_V_MIN,
    GREEN_PIXEL_THRESHOLD,
    WOOD_H_MIN, WOOD_H_MAX, WOOD_S_MIN, WOOD_V_MIN, WOOD_V_MAX,
    MIN_PILLAR_AREA,
)


def _rgb_to_hsv(r, g, b):
    """Convert RGB (0-255) to HSV (H: 0-180, S: 0-255, V: 0-255) like OpenCV."""
    r, g, b = r / 255.0, g / 255.0, b / 255.0
    mx = max(r, g, b)
    mn = min(r, g, b)
    diff = mx - mn
    v = mx

    if mx == 0:
        s = 0
    else:
        s = diff / mx

    if diff == 0:
        h = 0
    elif mx == r:
        h = (60 * ((g - b) / diff) + 360) % 360
    elif mx == g:
        h = (60 * ((b - r) / diff) + 120) % 360
    else:
        h = (60 * ((r - g) / diff) + 240) % 360

    # Convert to OpenCV scale
    return int(h / 2), int(s * 255), int(v * 255)


def _parse_image(image_bytes, width, height):
    """Parse Webots camera image (BGRA format) into list of (r, g, b) tuples per pixel."""
    pixels = []
    for i in range(width * height):
        offset = i * 4
        b = image_bytes[offset]
        g = image_bytes[offset + 1]
        r = image_bytes[offset + 2]
        pixels.append((r, g, b))
    return pixels


def _in_hsv_range(h, s, v, h_min, h_max, s_min, v_min, v_max=255):
    return h_min <= h <= h_max and s >= s_min and v_min <= v <= v_max


def detect_pillar(image_bytes, width, height, color="blue"):
    """Detect a colored pillar in the camera image.

    Args:
        image_bytes: raw camera image (BGRA)
        width, height: image dimensions
        color: "blue" or "yellow"

    Returns:
        (found, centroid_x, centroid_y, area) or (False, 0, 0, 0)
    """
    if image_bytes is None:
        return False, 0, 0, 0

    if color == "blue":
        h_min, h_max, s_min, v_min = BLUE_H_MIN, BLUE_H_MAX, BLUE_S_MIN, BLUE_V_MIN
    else:
        h_min, h_max, s_min, v_min = YELLOW_H_MIN, YELLOW_H_MAX, YELLOW_S_MIN, YELLOW_V_MIN

    sum_x, sum_y, count = 0, 0, 0

    for py in range(height):
        for px in range(width):
            idx = (py * width + px) * 4
            b = image_bytes[idx]
            g = image_bytes[idx + 1]
            r = image_bytes[idx + 2]
            h, s, v = _rgb_to_hsv(r, g, b)
            if _in_hsv_range(h, s, v, h_min, h_max, s_min, v_min):
                sum_x += px
                sum_y += py
                count += 1

    if count >= MIN_PILLAR_AREA:
        cx = sum_x / count
        cy = sum_y / count
        return True, cx, cy, count
    return False, 0, 0, 0


def detect_green_ground(image_bytes, width, height):
    """Detect green ground (poison) in the bottom third of the camera image.

    Returns:
        (detected, direction) where direction is "left", "center", "right", or "wide"
    """
    if image_bytes is None:
        return False, ""

    # Only scan bottom third
    start_row = int(height * 2 / 3)
    total_pixels = width * (height - start_row)
    left_count, center_count, right_count = 0, 0, 0
    third_w = width // 3

    for py in range(start_row, height):
        for px in range(width):
            idx = (py * width + px) * 4
            b = image_bytes[idx]
            g = image_bytes[idx + 1]
            r = image_bytes[idx + 2]
            h, s, v = _rgb_to_hsv(r, g, b)
            if _in_hsv_range(h, s, v, GREEN_H_MIN, GREEN_H_MAX, GREEN_S_MIN, GREEN_V_MIN):
                if px < third_w:
                    left_count += 1
                elif px < 2 * third_w:
                    center_count += 1
                else:
                    right_count += 1

    total_green = left_count + center_count + right_count
    fraction = total_green / max(total_pixels, 1)

    if fraction < GREEN_PIXEL_THRESHOLD:
        return False, ""

    # Determine direction
    section_pixels = total_pixels // 3
    left_frac = left_count / max(section_pixels, 1)
    center_frac = center_count / max(section_pixels, 1)
    right_frac = right_count / max(section_pixels, 1)

    active = sum(1 for f in [left_frac, center_frac, right_frac]
                 if f > GREEN_PIXEL_THRESHOLD)
    if active >= 2:
        return True, "wide"
    if center_frac > GREEN_PIXEL_THRESHOLD:
        return True, "center"
    if left_frac > right_frac:
        return True, "left"
    return True, "right"


def detect_floating_wall(image_bytes, depth_data, width, height):
    """Detect floating walls via wood-colored regions in the upper half of the image.

    Args:
        image_bytes: raw camera BGRA
        depth_data: depth RangeFinder data (list of floats)
        width, height: image dimensions

    Returns:
        (detected, estimated_distance) or (False, 0)
    """
    if image_bytes is None or depth_data is None:
        return False, 0

    # Scan upper half of image for wood-colored pixels
    end_row = height // 2
    wood_pixels = []

    for py in range(end_row):
        for px in range(width):
            idx = (py * width + px) * 4
            b = image_bytes[idx]
            g = image_bytes[idx + 1]
            r = image_bytes[idx + 2]
            h, s, v = _rgb_to_hsv(r, g, b)
            if _in_hsv_range(h, s, v, WOOD_H_MIN, WOOD_H_MAX, WOOD_S_MIN, WOOD_V_MIN, WOOD_V_MAX):
                wood_pixels.append((px, py))

    if len(wood_pixels) < 100:  # need significant area
        return False, 0

    # Get depth at wood pixel locations
    depths = []
    for px, py in wood_pixels:
        d_idx = py * width + px
        if d_idx < len(depth_data):
            d = depth_data[d_idx]
            if not math.isinf(d) and not math.isnan(d) and d > 0.1:
                depths.append(d)

    if not depths:
        return False, 0

    avg_depth = sum(depths) / len(depths)
    if avg_depth < 0.6:  # close floating wall
        return True, avg_depth
    return False, 0


def pillar_bearing(centroid_x, image_width, camera_fov):
    """Convert pixel x-coordinate to angular offset from camera center.

    Returns angle in radians (positive = right, negative = left).
    """
    center = image_width / 2.0
    offset = (centroid_x - center) / image_width
    return offset * camera_fov


def estimate_pillar_world_pos(robot_x, robot_y, robot_theta, bearing, distance):
    """Estimate pillar world position from robot pose, bearing, and distance."""
    world_angle = robot_theta + bearing
    wx = robot_x + distance * math.sin(world_angle)
    wy = robot_y + distance * math.cos(world_angle)
    return wx, wy


def get_depth_at_pixel(depth_data, width, height, px, py):
    """Get depth value at a specific pixel, averaging a small window."""
    depths = []
    for dy in range(-2, 3):
        for dx in range(-2, 3):
            nx, ny = int(px) + dx, int(py) + dy
            if 0 <= nx < width and 0 <= ny < height:
                idx = ny * width + nx
                if idx < len(depth_data):
                    d = depth_data[idx]
                    if not math.isinf(d) and not math.isnan(d) and d > 0.1:
                        depths.append(d)
    if depths:
        return sum(depths) / len(depths)
    return None
