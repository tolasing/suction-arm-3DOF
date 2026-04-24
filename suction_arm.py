import mujoco
import mujoco.viewer
import numpy as np
import time

model = mujoco.MjModel.from_xml_path('/home/portable/cad_designs/suction_arm/scene.xml')
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    # Record the actual simulation start time
    sim_start_time = time.time()
    
    # Define our movement phases
    move_to_target_time = 2.0  # Seconds to reach target
    hold_time = 1.0            # Seconds to stay there
    return_home_time = 2.0
    return_point_a=3.0     # Seconds to return
    
    while viewer.is_running():
        step_start = time.time()
        elapsed = time.time() - sim_start_time

        # --- CONTROL LOGIC START ---
        
        if elapsed < 1.0:
            # Phase 1: Moving to 1.5 radians
            data.ctrl[0] = 0.75
        elif elapsed < 2.0:
            # Phase 2: Holding at 1.5 radians
            data.ctrl[0] = 0.75
        elif elapsed < 3.0:
            # Phase 3: Returning to 0.0
            data.ctrl[0] = 1.5
        elif elapsed < 4.0:
            # Phase 3: Returning to 0.0
            data.ctrl[0] = 0.0

        elif elapsed < 5.0:
            # Phase 3: Returning to 0.0
            data.ctrl[1] = -0.2

        
        elif elapsed < 6.0:
            # Phase 3: Returning to 0.0
            data.ctrl[1] = 0.4

        elif elapsed < 7.0:
            # Phase 3: Returning to 0.0
            data.ctrl[1] = -0.2

        elif elapsed < 8.0:
            # Phase 3: Returning to 0.0
            data.ctrl[2] = -1.8

        
        elif elapsed < 9.0:
            # Phase 3: Returning to 0.0
            data.ctrl[2] = 0.0

        elif elapsed < 10.0:
            # Phase 3: Returning to 0.0
            data.ctrl[3] = -1.32
        

        elif elapsed < 11.0:
            # Phase 3: Returning to 0.0
            data.ctrl[3] = 1.32        
        
        else:
            # Phase 4: Stay at 0.0 (Stop moving)
            data.ctrl[3] = 0.0
            # Optional: print once when finished
            # print("Movement cycle complete.")

        # --- CONTROL LOGIC END ---

        mujoco.mj_step(model, data)
        viewer.sync()

        # Timing management
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)