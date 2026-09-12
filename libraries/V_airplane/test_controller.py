from controller import CONTROLLER


class FakeRCInput:
    autopilot = 1600
    throttlerc = 0.5
    rollrc = 0.0
    pitchrc = 0.0
    yawrc = 0.0


controller = CONTROLLER()
rcin = FakeRCInput()

controls, defaults, color = controller.loop(
    RunTime=0,
    rcin=rcin,
    rpy=[10.0, 5.0, 0.0],
    g=[2.0, 1.0, 0.0]
)

print("Controls:", [f"{x:.3f}" for x in controls])
print("Defaults:", [f"{x:.3f}" for x in defaults])
print("Colour:", color)
