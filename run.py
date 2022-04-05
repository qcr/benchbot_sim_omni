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


def disable_component(prop_path):
    print("DISABLING '%s.enabled'" % prop_path)
    execute("ChangeProperty",
            prop_path=Sdf.Path("%s.enabled" % prop_path),
            value=False,
            prev=None)


def tick_component(prop_path):
    execute("RosBridgeTickComponent", path=prop_path)


class SimulatorDaemon:

    def __init__(self, port):
        self.address = 'localhost:%s' % port

        self.inst = None
        self.sim = None
        self.sim_i = 0

        self.map_usd = None
        self.robot_usd = None
        self.start_pose = None

    def open_usd(self):
        # Stop simulation if running
        self.stop_simulation()

        # TODO figure out API calls for this...
        pass

        # Attempt to start the simulation
        start_simulation()

    def place_robot(self):
        # Stop simulation if running
        self.stop_simulation()

        # Add robot to the environment at the requested pose
        p = DEFAULT_POSE if self.start_pose is None else self.start_pose
        add_reference_to_stage(usd_path=self.robot_path,
                               prim_path=ROBOT_PRIM_PATH)
        r = Robot(prim_path=ROBOT_PRIM_PATH, name=ROBOT_NAME)
        r.set_world_pose(position=p[4::] * 100, orientation=p[:4])

        # Disable auto-publishing of all robot components (we'll manually
        # publish at varying frequencies instead)
        for p in ROBOT_COMPONENTS.values():
            disable_component(p)

        # Attempt to start the simulation
        start_simulation()

    def run(self):
        f = flask.Flask(__name__)

        @f.route('/', methods=['GET'])
        def __hello():
            return flask.jsonify("Hello, I am the Omniverse Sim Daemon")

        @f.route('/open_environment', methods=['POST'])
        def __open_env():
            r = flask.request.json
            if 'environment' in r:
                self.map_usd = r['environment']
            if self.inst is None:
                print("No simulator running. Stored USD, but not opening.")
                return
            self.open_usd()

        @f.route('/place_robot', methods=['POST'])
        def __place_robot():
            r = flask.request.json
            if 'robot' in r:
                self.robot_usd = r['robot']
            if 'start_pose' in r:
                self.start_pose = r['start_pose']
            if self.inst is None:
                print(
                    "No simulator running. Stored USD & pose, but not opening."
                )
                return
            self.place_robot()

        @f.route('/restart', methods=['POST'])
        def __restart():
            stop_simulation()
            start_simulation()

        @f.route('/start', methods=['POST'])
        def __start():
            start_simulation()

        @f.route('/stop', methods=['POST'])
        def __stop():
            stop_simulation()

        # Start long-running server
        server = pywsgi.WSGIServer(self.address, f)
        evt = event.Event()
        for s in [signal.SIGINT, signal.SIGQUIT, signal.SIGTERM]:
            signal.signal(s, lambda n, frame: evt.set())
        while not evt.is_set():
            self.tick_simulation()

        # Cleanup
        stop_instance()

    def start_instance(self):
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

        # Attempt to start the simulation
        start_simulation()

    def start_simulation(self):
        if self.sim is not None:
            self.stop_simulation()
        if self.inst is None or self.map_usd is None or self.robot_usd is None:
            print("Can't start simulation. Missing some required state.")
            return

        self.sim_i = 0
        self.sim = SimulationContext()
        self.sim.play()

    def stop_instance(self):
        if self.inst is None:
            print("No instance is running to stop.")
            return
        self.inst.close()
        self.inst = None

    def stop_simulation(self):
        if self.sim is None:
            print("Skipping. No running simulation to stop")
        if self.inst is None:
            print("Skipping. No running simulator found.")
        self.sim.stop()
        self.sim = None  # TODO maybe could reuse with more guarding logic?

    def tick_simulation(self):
        if self.inst is None or self.sim is None:
            return

        sc.step()

        # Tick at 60Hz
        tick_component(ROBOT_COMPONENTS['clock'])

        # Tick at 30Hz
        if self.sim_i % 2 == 0:
            tick_component(ROBOT_COMPONENTS['diff_base'])
            tick_component(ROBOT_COMPONENTS['lidar'])
            tick_component(ROBOT_COMPONENTS['tf'])
            tick_component(ROBOT_COMPONENTS['tf_sensors'])

        # Tick at 10Hz
        if self.sim_i % 6 == 0:
            tick_component(ROBOT_COMPONENTS['rgbd'])

        self.sim_i += 1


if __name__ == '__main__':
    sd = SimulatorDaemon(port=os.environ.get('PORT'))
    sd.run()
