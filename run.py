import flask
import numpy as np
import os
import signal
import sys
import time

from gevent import event, pywsgi, signal

from omni.isaac.kit import SimulationApp

DEFAULT_POSE = [1, 0, 0, 0, 0, 0, 0]

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


class SimulatorDaemon:

    def __init__(self, port):
        self.address = 'localhost:%d' % port

        self.inst = None
        self.sim = None

        self.map_usd = None
        self.robot_usd = None
        self.start_pose = None

    def open_usd(self):
        # TODO figure out API calls for this...
        pass

        # TODO start simulation if appropriate

    def place_robot(self):
        p = DEFAULT_POSE if self.start_pose is None else self.start_pose

        # Add robot to the environment at the requested pose
        add_reference_to_stage(usd_path=self.robot_path,
                               prim_path=ROBOT_PRIM_PATH)
        r = Robot(prim_path=ROBOT_PRIM_PATH, name=ROBOT_NAME)
        r.set_world_pose(position=p[4::] * 100, orientation=p[:4])

        # TODO start simulation if appropriate

    def run(self):
        f = flask.Flask(__name__)

        @f.route('/', methods=['GET'])
        def __hello():
            return flask.jsonify("Hello, I am the Omniverse Sim Daemon")

        @f.route('/open_environment', methods=['POST'])
        def __open_env():
            # TODO process JSON
            if self.inst is None:
                print("No simulator running. Stored USD, but not opening.")
                return
            self.open_usd()

        @f.route('/place_robot', methods=['POST'])
        def __place_robot():
            # TODO process JSON
            if self.inst is None:
                print(
                    "No simulator running. Stored USD & pose, but not opening."
                )
                return
            self.place_robot()

        @f.route('/restart', methods=['POST'])
        def __restart():
            stop()
            start()

        @f.route('/start', methods=['POST'])
        def __start():
            start()

        @f.route('/stop', methods=['POST'])
        def __stop():
            stop()

        server = pywsgi.WSGIServer(self.address, f)

    def start(self):
        if not self.inst is None:
            print("Instance already running. Please /stop first.")
            return
        env = {} if self.map_usd is None else {"open_usd": self.map_usd}
        # Start the simulator
        self.inst = SimulationApp({
            "renderer": "RayTracedLighting",
            "headless": False,
            **env
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

        # Insert the robot if it is set
        if self.robot_usd is not None:
            self.place_robot()

        # TODO start simulation if appropriate

    def stop(self):
        if self.inst is None:
            print("No instance is running to stop.")
            return
        self.inst.close()
        self.inst = None


if __name__ == '__main__':
    sd = SimulatorDaemon(port=os.environ.get('PORT'))


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
    i = 0
    while True:
        sc.step()

        # Tick at 60Hz
        tick_component(ROBOT_COMPONENTS['clock'])

        # Tick at 30Hz
        if i % 2 == 0:
            tick_component(ROBOT_COMPONENTS['diff_base'])
            tick_component(ROBOT_COMPONENTS['lidar'])
            tick_component(ROBOT_COMPONENTS['tf'])
            tick_component(ROBOT_COMPONENTS['tf_sensors'])

        # Tick at 10Hz
        if i % 6 == 0:
            tick_component(ROBOT_COMPONENTS['rgbd'])

        i += 1
