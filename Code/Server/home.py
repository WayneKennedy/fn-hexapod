# -*- coding: utf-8 -*-
"""
home.py - Move all legs to calibrated home position for verification.
"""
import math
import time
from servo import Servo


def read_calibration_offsets(filename='point'):
    """Read calibration offsets from point.txt"""
    with open(filename + ".txt", "r") as file:
        lines = file.readlines()
        data = [list(map(int, line.strip().split("\t"))) for line in lines]
    return data


def restrict_value(value, min_value, max_value):
    """Clamp value to range."""
    if value < min_value:
        return min_value
    elif value > max_value:
        return max_value
    return value


def coordinate_to_angle(x, y, z, l1=33, l2=90, l3=110):
    """Convert leg endpoint coordinates to servo angles."""
    a = math.pi / 2 - math.atan2(z, y)
    x_3 = 0
    x_4 = l1 * math.sin(a)
    x_5 = l1 * math.cos(a)
    l23 = math.sqrt((z - x_5) ** 2 + (y - x_4) ** 2 + (x - x_3) ** 2)

    w = max(-1, min(1, (x - x_3) / l23))
    v = max(-1, min(1, (l2 * l2 + l23 * l23 - l3 * l3) / (2 * l2 * l23)))
    u = max(-1, min(1, (l2 ** 2 + l3 ** 2 - l23 ** 2) / (2 * l3 * l2)))

    b = math.asin(round(w, 2)) - math.acos(round(v, 2))
    c = math.pi - math.acos(round(u, 2))
    return round(math.degrees(a)), round(math.degrees(b)), round(math.degrees(c))


def calculate_calibration_angles(calibration_positions):
    """Calculate calibration angle offsets from positions."""
    calibration_angles = [[0, 0, 0] for _ in range(6)]
    current_angles = [[0, 0, 0] for _ in range(6)]

    # Default leg position
    leg_position = [140, 0, 0]

    for i in range(6):
        # Angles from calibration positions
        calibration_angles[i][0], calibration_angles[i][1], calibration_angles[i][2] = coordinate_to_angle(
            -calibration_positions[i][2], calibration_positions[i][0], calibration_positions[i][1])
        # Angles from default position
        current_angles[i][0], current_angles[i][1], current_angles[i][2] = coordinate_to_angle(
            -leg_position[2], leg_position[0], leg_position[1])

    # Calculate offset differences
    for i in range(6):
        calibration_angles[i][0] = calibration_angles[i][0] - current_angles[i][0]
        calibration_angles[i][1] = calibration_angles[i][1] - current_angles[i][1]
        calibration_angles[i][2] = calibration_angles[i][2] - current_angles[i][2]

    return calibration_angles


def set_home_position(servo, calibration_angles):
    """Set all servos to calibrated home position."""
    # Default leg position
    leg_position = [140, 0, 0]
    current_angles = [[0, 0, 0] for _ in range(6)]

    # Calculate base angles for default position
    for i in range(6):
        current_angles[i][0], current_angles[i][1], current_angles[i][2] = coordinate_to_angle(
            -leg_position[2], leg_position[0], leg_position[1])

    # Apply calibration offsets (legs 0-2 vs 3-5 have mirrored servo orientations)
    for i in range(3):
        current_angles[i][0] = restrict_value(current_angles[i][0] + calibration_angles[i][0], 0, 180)
        current_angles[i][1] = restrict_value(90 - (current_angles[i][1] + calibration_angles[i][1]), 0, 180)
        current_angles[i][2] = restrict_value(current_angles[i][2] + calibration_angles[i][2], 0, 180)

        current_angles[i + 3][0] = restrict_value(current_angles[i + 3][0] + calibration_angles[i + 3][0], 0, 180)
        current_angles[i + 3][1] = restrict_value(90 + current_angles[i + 3][1] + calibration_angles[i + 3][1], 0, 180)
        current_angles[i + 3][2] = restrict_value(180 - (current_angles[i + 3][2] + calibration_angles[i + 3][2]), 0, 180)

    # Set servo angles per leg (matching control.py channel mapping)
    # Leg 1 (index 0)
    servo.set_servo_angle(15, current_angles[0][0])
    servo.set_servo_angle(14, current_angles[0][1])
    servo.set_servo_angle(13, current_angles[0][2])
    print(f"Leg 1: coxa={current_angles[0][0]}, femur={current_angles[0][1]}, tibia={current_angles[0][2]}")

    # Leg 2 (index 1)
    servo.set_servo_angle(12, current_angles[1][0])
    servo.set_servo_angle(11, current_angles[1][1])
    servo.set_servo_angle(10, current_angles[1][2])
    print(f"Leg 2: coxa={current_angles[1][0]}, femur={current_angles[1][1]}, tibia={current_angles[1][2]}")

    # Leg 3 (index 2)
    servo.set_servo_angle(9, current_angles[2][0])
    servo.set_servo_angle(8, current_angles[2][1])
    servo.set_servo_angle(31, current_angles[2][2])
    print(f"Leg 3: coxa={current_angles[2][0]}, femur={current_angles[2][1]}, tibia={current_angles[2][2]}")

    # Leg 4 (index 3)
    servo.set_servo_angle(22, current_angles[3][0])
    servo.set_servo_angle(23, current_angles[3][1])
    servo.set_servo_angle(27, current_angles[3][2])
    print(f"Leg 4: coxa={current_angles[3][0]}, femur={current_angles[3][1]}, tibia={current_angles[3][2]}")

    # Leg 5 (index 4)
    servo.set_servo_angle(19, current_angles[4][0])
    servo.set_servo_angle(20, current_angles[4][1])
    servo.set_servo_angle(21, current_angles[4][2])
    print(f"Leg 5: coxa={current_angles[4][0]}, femur={current_angles[4][1]}, tibia={current_angles[4][2]}")

    # Leg 6 (index 5)
    servo.set_servo_angle(16, current_angles[5][0])
    servo.set_servo_angle(17, current_angles[5][1])
    servo.set_servo_angle(18, current_angles[5][2])
    print(f"Leg 6: coxa={current_angles[5][0]}, femur={current_angles[5][1]}, tibia={current_angles[5][2]}")


if __name__ == '__main__':
    print("Moving all legs to calibrated home position...")
    print()

    # Read calibration data
    calibration_positions = read_calibration_offsets('point')
    print("Calibration offsets from point.txt:")
    for i, pos in enumerate(calibration_positions):
        print(f"  Leg {i+1}: x={pos[0]:4d}, y={pos[1]:4d}, z={pos[2]:4d}")
    print()

    # Calculate calibration angles
    calibration_angles = calculate_calibration_angles(calibration_positions)

    # Initialize servos and move to home
    servo = Servo()
    set_home_position(servo, calibration_angles)

    print()
    print("Home position set. Holding for 5 seconds...")
    time.sleep(5)
    print("Relaxing servos...")
    servo.relax()
    print("Done.")
