# -*- coding: utf-8 -*-
"""
stand.py - Smoothly move to home position, then raise body to standing height.
"""
import math
import time
from servo import Servo


# Servo channel mapping per leg
SERVO_CHANNELS = {
    0: (15, 14, 13),  # Leg 1: coxa, femur, tibia
    1: (12, 11, 10),  # Leg 2
    2: (9, 8, 31),    # Leg 3
    3: (22, 23, 27),  # Leg 4
    4: (19, 20, 21),  # Leg 5
    5: (16, 17, 18),  # Leg 6
}


def read_calibration_offsets(filename='point'):
    """Read calibration offsets from point.txt"""
    with open(filename + ".txt", "r") as file:
        lines = file.readlines()
        data = [list(map(int, line.strip().split("\t"))) for line in lines]
    return data


def restrict_value(value, min_value, max_value):
    """Clamp value to range."""
    return max(min_value, min(max_value, value))


def coordinate_to_angle(x, y, z, l1=33, l2=90, l3=110):
    """Convert leg endpoint coordinates to servo angles."""
    a = math.pi / 2 - math.atan2(z, y)
    x_4 = l1 * math.sin(a)
    x_5 = l1 * math.cos(a)
    l23 = math.sqrt((z - x_5) ** 2 + (y - x_4) ** 2 + x ** 2)

    w = max(-1, min(1, x / l23))
    v = max(-1, min(1, (l2 * l2 + l23 * l23 - l3 * l3) / (2 * l2 * l23)))
    u = max(-1, min(1, (l2 ** 2 + l3 ** 2 - l23 ** 2) / (2 * l3 * l2)))

    b = math.asin(round(w, 2)) - math.acos(round(v, 2))
    c = math.pi - math.acos(round(u, 2))
    return round(math.degrees(a)), round(math.degrees(b)), round(math.degrees(c))


def calculate_calibration_angles(calibration_positions):
    """Calculate calibration angle offsets."""
    calibration_angles = [[0, 0, 0] for _ in range(6)]
    current_angles = [[0, 0, 0] for _ in range(6)]
    leg_position = [140, 0, 0]

    for i in range(6):
        calibration_angles[i][0], calibration_angles[i][1], calibration_angles[i][2] = coordinate_to_angle(
            -calibration_positions[i][2], calibration_positions[i][0], calibration_positions[i][1])
        current_angles[i][0], current_angles[i][1], current_angles[i][2] = coordinate_to_angle(
            -leg_position[2], leg_position[0], leg_position[1])

    for i in range(6):
        calibration_angles[i][0] -= current_angles[i][0]
        calibration_angles[i][1] -= current_angles[i][1]
        calibration_angles[i][2] -= current_angles[i][2]

    return calibration_angles


def leg_position_to_angles(leg_position, calibration_angles):
    """Convert leg positions to servo angles for all 6 legs."""
    current_angles = [[0, 0, 0] for _ in range(6)]

    for i in range(6):
        current_angles[i][0], current_angles[i][1], current_angles[i][2] = coordinate_to_angle(
            -leg_position[2], leg_position[0], leg_position[1])

    # Apply calibration offsets (legs 0-2 vs 3-5 have mirrored orientations)
    for i in range(3):
        current_angles[i][0] = restrict_value(current_angles[i][0] + calibration_angles[i][0], 0, 180)
        current_angles[i][1] = restrict_value(90 - (current_angles[i][1] + calibration_angles[i][1]), 0, 180)
        current_angles[i][2] = restrict_value(current_angles[i][2] + calibration_angles[i][2], 0, 180)

        current_angles[i + 3][0] = restrict_value(current_angles[i + 3][0] + calibration_angles[i + 3][0], 0, 180)
        current_angles[i + 3][1] = restrict_value(90 + current_angles[i + 3][1] + calibration_angles[i + 3][1], 0, 180)
        current_angles[i + 3][2] = restrict_value(180 - (current_angles[i + 3][2] + calibration_angles[i + 3][2]), 0, 180)

    return current_angles


def set_all_angles(servo, angles):
    """Set all servo angles at once."""
    for leg in range(6):
        channels = SERVO_CHANNELS[leg]
        servo.set_servo_angle(channels[0], angles[leg][0])
        servo.set_servo_angle(channels[1], angles[leg][1])
        servo.set_servo_angle(channels[2], angles[leg][2])


def smooth_move(servo, start_angles, end_angles, duration=1.0, steps=50):
    """Smoothly interpolate from start to end angles."""
    for step in range(steps + 1):
        t = step / steps
        # Ease in-out for smoother motion
        t = t * t * (3 - 2 * t)

        current = [[0, 0, 0] for _ in range(6)]
        for leg in range(6):
            for joint in range(3):
                current[leg][joint] = start_angles[leg][joint] + t * (end_angles[leg][joint] - start_angles[leg][joint])

        set_all_angles(servo, current)
        time.sleep(duration / steps)


if __name__ == '__main__':
    print("Stand sequence starting...")
    print()

    # Load calibration
    calibration_positions = read_calibration_offsets('point')
    calibration_angles = calculate_calibration_angles(calibration_positions)

    # Initialize servo
    servo = Servo()

    # Home position: x=140, y=0, z=0 (body height 0)
    home_position = [140, 0, 0]
    home_angles = leg_position_to_angles(home_position, calibration_angles)

    # Stand position: x=140, y=0, z=-30 (body raised 30mm)
    stand_position = [140, 0, -30]
    stand_angles = leg_position_to_angles(stand_position, calibration_angles)

    print("Setting home position (body height 0)...")
    set_all_angles(servo, home_angles)

    time.sleep(0.5)

    print("Smoothly raising body to +30mm...")
    smooth_move(servo, home_angles, stand_angles, duration=1.0, steps=50)

    print()
    print("Standing. Holding for 5 seconds...")
    time.sleep(5)
    print("Relaxing servos...")
    servo.relax()
    print("Done.")
