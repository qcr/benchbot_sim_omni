import numpy as np
import os
import signal
import sys
import time

from omni.isaac.kit import SimulationApp

ROBOT_NAME = 'robot'
ROBOT_PRIM_PATH = '/%s' % ROBOT_NAME
ROBOT_COMPONENTS = {
    'clock': '/ROS_Clock',
    'diff_base': '%s/ROS_DifferentialBase' % ROBOT_PRIM_PATH,
    'lidar': '%s/ROS_Lidar' % ROBOT_PRIM_PATH,
    'rgbd': '%s/ROS_Camera_Stereo_Left' % ROBOT_PRIM_PATH,
    'tf_sensors': '%s/ROS_Carter_Sensors_Broadcaster' % ROBOT_PRIM_PATH,
    'tf': '%s/ROS_Carter_Broadcaster' % ROBOT_PRIM_PATH
}
UPDATE_DELAY_SECS = 3.0

map_path = os.environ.get('BENCHBOT_MAP_PATH')
robot_path = os.environ.get('BENCHBOT_ROBOT_PATH')
start_pose = os.environ.get('BENCHBOT_START_POSE')


def finish(sim_context, kit):
    print("BENCHBOT: Exit requested. Finishing ...")
    sim_context.stop()
    kit.close()
    sys.exit()  # TODO figure out why this seg faults


def disable_component(prop_path):
    print("DISABLING '%s.enabled'" % prop_path)
    execute("ChangeProperty",
            prop_path=Sdf.Path("%s.enabled" % prop_path),
            value=False,
            prev=None)


def tick_component(prop_path):
    execute("RosBridgeTickComponent", path=prop_path)


if __name__ == '__main__':
    # Handle input arguments (env vars due to python.sh bug)
    if start_pose != None:
        start_pose = np.array([float(x) for x in start_pose.split(',')])

    print("Starting Omniverse-powered Isaac Sim simulation with settings:")
    print("\tmap_path:\t%s" % map_path)
    print("\trobot_path:\t%s" % robot_path)
    print("\tstart_pose:\t%s" % start_pose)

    if map_path is None or not os.path.exists(map_path):
        print("ERROR: map_path '%s' does not exist." % map_path)
        quit()
    if robot_path is None or not os.path.exists(robot_path):
        print("ERROR: robot_path '%s' does not exist." % robot_path)
        quit()
    if start_pose is None or len(start_pose) != 7:
        print("ERROR: Start pose is not 7 comma-separated values.")
        quit()

    # Start the simulator
    k = SimulationApp({
        "renderer": "RayTracedLighting",
        "headless": False,
        "open_usd": map_path
    })

    # Import all required modules, and configure application
    from omni.isaac.core import SimulationContext
    from omni.isaac.core.robots import Robot
    from omni.isaac.core.utils.extensions import enable_extension
    from omni.isaac.core.utils.stage import add_reference_to_stage
    from omni.kit.commands import execute
    from omni.kit.viewport import get_default_viewport_window
    from pxr import Sdf
    enable_extension("omni.isaac.ros_bridge")

    # Configure the interface
    vp = get_default_viewport_window()
    # vp.set_active_camera('/%s/carter_view' % ROBOT_PRIM_PATH)  seems to get overridden??

    # Add robot to the environment at the requested pose
    add_reference_to_stage(usd_path=robot_path, prim_path=ROBOT_PRIM_PATH)
    r = Robot(prim_path=ROBOT_PRIM_PATH, name=ROBOT_NAME)
    r.set_world_pose(position=start_pose[4::] * 100,
                     orientation=start_pose[:4])

    # Disable auto-publishing of all robot components (we'll manually publish
    # at varying frequencies instead)
    for p in ROBOT_COMPONENTS.values():
        disable_component(p)

    # Random number of updates for UI to catch up with things???
    a = time.time()
    while (time.time() - a < UPDATE_DELAY_SECS):
        k.update()

    # Wait until we find a ROS master
    # TODO dynamically incorporate address to look for master...
    print("BENCHBOT: Waiting for ROS master on TODO ...")
    chk = False
    while not chk:
        time.sleep(1)
        _, chk = execute("RosBridgeRosMasterCheck")
    print("BENCHBOT: Found ROS master.")

    # Start the simulation, quitting on SIGINT
    sc = SimulationContext()
    sc.play()
    print("BENCHBOT: Running simulation ...")
    signal.signal(signal.SIGINT, lambda _, __: finish(sc, k))
    while True:
        sc.step()
