import re

ratios = {
    'R1': 0.82, 'R2': 0.67, 'L1': 0.73, 'L2': 0.67,
    'R4': 0.78, 'R3': 0.80, 'L3': 0.78, 'L4': 0.81
}

rest = {
    'R1': 78, 'R2': 80, 'L1': 98, 'L2': 80,
    'R4': 90, 'R3': 84, 'L3': 80, 'L4': 77
}

def calc_angle(servo, angle):
    return int(round(rest[servo] + (angle - 90) * ratios[servo]))

with open('movement-sequences.h', 'r') as f:
    lines = f.readlines()

new_lines = []
for line in lines:
    if 'setServoAngle' in line:
        # We replace each setServoAngle(S, A) with setServoAngle(S, calc(A))
        # Wait, not all of them! Poses with explicit bases should be replaced by hand.
        pass
    new_lines.append(line)
