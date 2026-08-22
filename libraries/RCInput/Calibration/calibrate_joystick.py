#!/usr/bin/env python3
"""
FASTCASST Joystick Calibration Tool
-----------------------------------
This script detects connected joysticks/gamepads, performs axis and button
calibration, and saves the configuration to a JSON file for flight simulation.
"""

import os
import sys
import json
import time

try:
    import pygame
except ImportError:
    print("Pygame is required for joystick calibration.")
    print("Please install it using: pip3 install pygame or sudo apt install python3-pygame")
    sys.exit(1)


def main():
    pygame.init()
    pygame.joystick.init()

    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        print("----------------------------------------")
        print("----------------------------------------")
        print("Error: No joystick/gamepad detected!")
        print("Please connect a joystick and try again.")
        print("----------------------------------------")
        print("----------------------------------------")
        sys.exit(1)

    print(f"Detected {joystick_count} joystick(s):")
    for i in range(joystick_count):
        js = pygame.joystick.Joystick(i)
        js.init()
        print(f"  [{i}] {js.get_name()}")

    if joystick_count == 1:
        js_index = 0
    else:
        try:
            js_index = int(input("Select joystick index to calibrate [0]: ") or "0")
        except ValueError:
            js_index = 0

    js = pygame.joystick.Joystick(js_index)
    js.init()
    print(f"\nCalibrating Joystick: {js.get_name()}")
    print(f"Axes: {js.get_numaxes()}, Buttons: {js.get_numbuttons()}, Hats: {js.get_numhats()}\n")

    config = {
        "device_name": js.get_name(),
        "deadzone": 0.05,
        "mapping": {
            "pitch": {"axis": 1, "invert": True, "min": -1.0, "max": 1.0},
            "roll": {"axis": 0, "invert": False, "min": -1.0, "max": 1.0},
            "yaw": {"axis": 2, "invert": False, "min": -1.0, "max": 1.0},
            "throttle": {"axis": 3, "invert": True, "min": -1.0, "max": 1.0}
        }
    }

    # Calibration step
    controls = ["roll", "pitch", "yaw", "throttle"]
    for control in controls:
        print(f"--- Calibrating {control.upper()} ---")
        input(f"Move the {control} stick to its center/neutral position and press ENTER...")
        pygame.event.pump()
        center_vals = [js.get_axis(i) for i in range(js.get_numaxes())]

        input(f"Move the {control} stick to its maximum extent and press ENTER...")
        pygame.event.pump()
        max_vals = [js.get_axis(i) for i in range(js.get_numaxes())]

        # Determine which axis changed the most
        max_diff = 0
        detected_axis = 0
        for idx in range(js.get_numaxes()):
            diff = abs(max_vals[idx] - center_vals[idx])
            if diff > max_diff:
                max_diff = diff
                detected_axis = idx

        print(f"Detected axis {detected_axis} for {control}.")
        config["mapping"][control]["axis"] = detected_axis

    output_filename = "joystick_config.json"
    with open(output_filename, "w") as f:
        json.dump(config, f, indent=4)

    print(f"\nCalibration complete! Configuration saved to '{output_filename}'.")


if __name__ == "__main__":
    main()
