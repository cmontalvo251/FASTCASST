import threading
import sys
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

    if len(sys.argv) > 1:
        # If a command-line argument is provided, use it as the vehicle type
        vehicle_type = sys.argv[1]
        print('Using vehicle type from command line argument:', vehicle_type)
    else:
        # Default vehicle type if no argument is provided
        vehicle_type = 'car'
        print('No vehicle type provided. Using default:', vehicle_type)

    # 2. Start OpenGL main loop on the main thread
    glhandle_g.loop_init(fileroot='../../../',vehicleType=vehicle_type,inFarplane=10000, inWidth=600, inHeight=600)