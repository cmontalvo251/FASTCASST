## Input Controls & Joystick Calibration

To set up your joystick/gamepad for realistic flight simulation control:

1. Connect your joystick or gamepad.
2. Install pygame dependencies:
   ```
   pip3 install pygame
   ```
3. Run the interactive joystick calibration script:
   ```
   python3 calibrate_joystick.py
   ```
4. Follow the on-screen instructions to map pitch, roll, yaw, and throttle axes.
5. The calibration script generates `joystick_config.json` containing calibrated limits, axis assignments, and deadzone settings for your device.
