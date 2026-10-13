import os
# Force PyOpenGL to use the X11 platform backend (fixes Linux/Wayland context errors)
# Will need to change this if you are using a different platform (e.g., Windows or macOS)
os.environ["PYOPENGL_PLATFORM"] = "x11"
import threading
import time
import math
from opengl import glhandle_g, state_mutex, MATLABMatrix

def physics_simulation_loop():
    """Runs your physics engine in the background thread."""
    print("Physics engine thread started.")
    
    # Dummy vehicle state matrices for testing
    cgin = MATLABMatrix(3, 1)
    ptpin = MATLABMatrix(3, 1)
    keyboard_out = [0.0] * 4

    sim_time = 0.0
    while glhandle_g.ok == 0 and glhandle_g.ready == 0:
        time.sleep(0.01)  # Wait for renderer to initialize

    while glhandle_g.ready == 1:
        sim_time += 0.01
        
        # Update vehicle position / attitude from physics calculations here
        cgin.set(1, 1, math.sin(sim_time))
        cgin.set(2, 1, sim_time)
        cgin.set(3, 1, 0.0)
        
        # Push update to shared state safely using UpdateRender
        glhandle_g.state.UpdateRender(sim_time, cgin, ptpin, 1, keyboard_out)
        
        time.sleep(0.01)


if __name__ == "__main__":
    # 1. Start physics engine on a background thread
    physics_thread = threading.Thread(target=physics_simulation_loop, daemon=True)
    physics_thread.start()

    # 2. Start OpenGL main loop on the main thread
    glhandle_g.loop_init(fileroot='../../V_car/', inFarplane=1000, inWidth=800, inHeight=600)